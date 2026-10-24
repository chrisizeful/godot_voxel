#include "voxel_terrain_navigation.h"
#include "../../engine/voxel_engine.h"
#include "../../storage/voxel_buffer.h"
#include "../../storage/voxel_data.h"
#include "../../util/godot/classes/engine.h"
#include "../../util/godot/classes/navigation_region_3d.h"
#include <godot_cpp/classes/navigation_obstacle3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include "../../util/godot/classes/navigation_server_3d.h"
#include "../../util/godot/classes/object.h"
#include "../../util/godot/classes/world_3d.h"
#include "../../util/math/conv.h"
#include "../../util/profiling.h"
#include "../../util/profiling_clock.h"
#include <godot_cpp/classes/worker_thread_pool.hpp>
#include "../fixed_lod/voxel_terrain.h"

namespace zylann::voxel {

namespace {

// Walkable static triangles are split down to this edge length (world units) before testing whether voxels bury them.
constexpr real_t BURIED_SAMPLE_SPACING = 1.0;
// Seconds between looks at the obstacles, and how far one moves before its tiles rebake.
constexpr float OBSTACLE_CHECK_INTERVAL = 0.25f;
constexpr real_t OBSTACLE_MOVE_TOLERANCE = 0.25;
// Sides of the polygon standing in for an obstacle that has a radius but no vertices.
constexpr int OBSTACLE_CIRCLE_SIDES = 8;

inline uint64_t tile_key(Vector2i p) {
	return (uint64_t(uint32_t(p.x)) << 32) | uint64_t(uint32_t(p.y));
}

inline Vector2i key_tile(uint64_t key) {
	return Vector2i(int32_t(uint32_t(key >> 32)), int32_t(uint32_t(key & 0xffffffff)));
}

void copy_navigation_mesh_settings(NavigationMesh &dst, const NavigationMesh &src) {
	dst.set_cell_size(src.get_cell_size());
	dst.set_cell_height(src.get_cell_height());
	dst.set_agent_height(src.get_agent_height());
	dst.set_agent_max_climb(src.get_agent_max_climb());
	dst.set_agent_max_slope(src.get_agent_max_slope());
	// `get_agent_radius` is not `const` due to an oversight in Godot
	dst.set_agent_radius(const_cast<NavigationMesh &>(src).get_agent_radius());
	dst.set_detail_sample_distance(src.get_detail_sample_distance());
	dst.set_detail_sample_max_error(src.get_detail_sample_max_error());
	dst.set_edge_max_error(src.get_edge_max_error());
	dst.set_edge_max_length(src.get_edge_max_length());
	dst.set_filter_low_hanging_obstacles(src.get_filter_low_hanging_obstacles());
	dst.set_filter_ledge_spans(src.get_filter_ledge_spans());
	dst.set_filter_walkable_low_height_spans(src.get_filter_walkable_low_height_spans());
	dst.set_region_merge_size(src.get_region_merge_size());
	dst.set_region_min_size(src.get_region_min_size());
	dst.set_sample_partition_type(src.get_sample_partition_type());
	dst.set_vertices_per_polygon(src.get_vertices_per_polygon());
}

struct Obstruction {
	PackedVector3Array vertices;
	float elevation = 0.f;
	float height = 0.f;
	bool carve = false;
};

// Reads one projected obstruction as NavigationMeshSourceGeometryData3D returns it.
bool read_obstruction(const Dictionary &d, Obstruction &out) {
	const Variant vertices = d.get("vertices", Variant());
	if (vertices.get_type() == Variant::PACKED_VECTOR3_ARRAY) {
		out.vertices = vertices;
	} else if (vertices.get_type() == Variant::PACKED_FLOAT32_ARRAY) {
		const PackedFloat32Array flat = vertices;
		out.vertices.resize(flat.size() / 3);
		for (int i = 0; i + 2 < flat.size(); i += 3) {
			out.vertices.set(i / 3, Vector3(flat[i], flat[i + 1], flat[i + 2]));
		}
	} else {
		return false;
	}
	out.elevation = d.get("elevation", 0.f);
	out.height = d.get("height", 0.f);
	out.carve = d.get("carve", false);
	return out.vertices.size() >= 3;
}

AABB get_obstruction_bounds(const Obstruction &o) {
	AABB box(o.vertices[0], Vector3());
	for (int i = 1; i < o.vertices.size(); ++i) {
		box.expand_to(o.vertices[i]);
	}
	box.position.y = o.elevation;
	box.size.y = o.height;
	return box;
}

// Samples the SDF copied into a buffer around the area being tested, trilinearly, in voxels. Air outside the copy.
struct SdfSampler {
	VoxelBuffer buffer;
	Vector3i origin;
	Transform3D world_to_voxel;
	bool valid = false;

	SdfSampler() : buffer(VoxelBuffer::ALLOCATOR_POOL) {}

	float sample(const Vector3 &world_pos) const {
		if (!valid) {
			return 1.f;
		}
		const Vector3 p = world_to_voxel.xform(world_pos) - to_vec3(origin);
		const Vector3i size = buffer.get_size();
		const Vector3i b = math::floor_to_int(p);
		if (b.x < 0 || b.y < 0 || b.z < 0 || b.x + 1 >= size.x || b.y + 1 >= size.y || b.z + 1 >= size.z) {
			return 1.f;
		}
		const Vector3 f = p - to_vec3(b);
		const unsigned int ch = VoxelBuffer::CHANNEL_SDF;
		const float c000 = buffer.get_voxel_f(b.x, b.y, b.z, ch);
		const float c100 = buffer.get_voxel_f(b.x + 1, b.y, b.z, ch);
		const float c010 = buffer.get_voxel_f(b.x, b.y + 1, b.z, ch);
		const float c110 = buffer.get_voxel_f(b.x + 1, b.y + 1, b.z, ch);
		const float c001 = buffer.get_voxel_f(b.x, b.y, b.z + 1, ch);
		const float c101 = buffer.get_voxel_f(b.x + 1, b.y, b.z + 1, ch);
		const float c011 = buffer.get_voxel_f(b.x, b.y + 1, b.z + 1, ch);
		const float c111 = buffer.get_voxel_f(b.x + 1, b.y + 1, b.z + 1, ch);
		const float x00 = Math::lerp(c000, c100, float(f.x));
		const float x10 = Math::lerp(c010, c110, float(f.x));
		const float x01 = Math::lerp(c001, c101, float(f.x));
		const float x11 = Math::lerp(c011, c111, float(f.x));
		return Math::lerp(Math::lerp(x00, x10, float(f.y)), Math::lerp(x01, x11, float(f.y)), float(f.z));
	}
};

} // namespace

// One tile's bake: assembled on the main thread, run on a worker, applied back on the main thread.
class BakeTileTask {
public:
	struct VoxelSurface {
		PackedVector3Array vertices;
		PackedInt32Array indices;
		// Block origin in terrain-local voxel space.
		Vector3 origin;
	};

	// Inputs, filled on the main thread.
	uint64_t key = 0;
	uint32_t generation = 0;
	Ref<NavigationMesh> navmesh;
	AABB baking_bounds;
	Transform3D terrain_transform;
	StdVector<VoxelSurface> voxel_surfaces;
	PackedFloat32Array static_vertices;
	PackedInt32Array static_indices;
	StdVector<int32_t> static_triangles;
	StdVector<Obstruction> obstructions;
	std::shared_ptr<VoxelData> voxel_data;
	bool drop_buried = true;
	// Height above a walkable static triangle at which solid voxels mean it lies buried.
	real_t buried_probe_height = 1.f;
	real_t walkable_cos = 0.f;

	// Outputs.
	uint32_t bake_usec = 0;

	static void run_static(void *self) {
		static_cast<BakeTileTask *>(self)->run();
	}

	void run() {
		ZN_PROFILE_SCOPE();
		ProfilingClock clock;

		PackedFloat32Array vertices;
		PackedInt32Array indices;
		add_voxel_triangles(vertices, indices);
		add_static_triangles(vertices, indices);

		Ref<NavigationMeshSourceGeometryData3D> source;
		source.instantiate();
		source->set_vertices(vertices);
		source->set_indices(indices);
		for (const Obstruction &o : obstructions) {
			source->add_projected_obstruction(o.vertices, o.elevation, o.height, o.carve);
		}
		NavigationServer3D::get_singleton()->bake_from_source_geometry_data(navmesh, source, Callable());
		bake_usec = clock.get_elapsed_microseconds();
	}

private:

	void add_voxel_triangles(PackedFloat32Array &vertices, PackedInt32Array &indices) const {
		ZN_PROFILE_SCOPE();
		int64_t vertex_count = 0;
		int64_t index_count = 0;
		for (const VoxelSurface &s : voxel_surfaces) {
			vertex_count += s.vertices.size();
			index_count += s.indices.size();
		}
		vertices.resize(vertex_count * 3);
		indices.resize(index_count);
		float *vw = vertices.ptrw();
		int32_t *iw = indices.ptrw();
		int64_t vbase = 0;
		int64_t ii = 0;
		for (const VoxelSurface &s : voxel_surfaces) {
			const Vector3 *vr = s.vertices.ptr();
			for (int i = 0; i < s.vertices.size(); ++i) {
				const Vector3 v = terrain_transform.xform(vr[i] + s.origin);
				vw[(vbase + i) * 3 + 0] = v.x;
				vw[(vbase + i) * 3 + 1] = v.y;
				vw[(vbase + i) * 3 + 2] = v.z;
			}
			// Mesh triangles are wound the other way round from what the baker expects (Godot's own mesh parser swaps
			// them too). Triangles entirely outside the baking area are left out.
			const int32_t *ir = s.indices.ptr();
			for (int i = 0; i + 2 < s.indices.size(); i += 3) {
				const int64_t a = vbase + ir[i];
				const int64_t b = vbase + ir[i + 2];
				const int64_t c = vbase + ir[i + 1];
				AABB tri(Vector3(vw[a * 3], vw[a * 3 + 1], vw[a * 3 + 2]), Vector3());
				tri.expand_to(Vector3(vw[b * 3], vw[b * 3 + 1], vw[b * 3 + 2]));
				tri.expand_to(Vector3(vw[c * 3], vw[c * 3 + 1], vw[c * 3 + 2]));
				if (!tri.grow(0.01).intersects(baking_bounds)) {
					continue;
				}
				iw[ii++] = a;
				iw[ii++] = b;
				iw[ii++] = c;
			}
			vbase += s.vertices.size();
		}
		indices.resize(ii);
	}

	Vector3 static_vertex(int32_t index) const {
		const float *v = static_vertices.ptr() + int64_t(index) * 3;
		return Vector3(v[0], v[1], v[2]);
	}

	bool is_walkable(const Vector3 &a, const Vector3 &b, const Vector3 &c) const {
		const Vector3 n = (b - a).cross(c - a);
		const real_t len = n.length();
		return len > 0.0 && n.y / len >= walkable_cos;
	}

	// Copies the SDF around every walkable static triangle, once, so testing burial reads no shared state per sample.
	void prepare_sdf(SdfSampler &sampler) const {
		if (!drop_buried || voxel_data == nullptr) {
			return;
		}
		AABB box;
		bool any = false;
		for (const int32_t t : static_triangles) {
			const Vector3 a = static_vertex(static_indices[t * 3]);
			const Vector3 b = static_vertex(static_indices[t * 3 + 1]);
			const Vector3 c = static_vertex(static_indices[t * 3 + 2]);
			if (!is_walkable(a, b, c)) {
				continue;
			}
			if (!any) {
				box = AABB(a, Vector3());
				any = true;
			}
			box.expand_to(a);
			box.expand_to(b);
			box.expand_to(c);
		}
		if (!any) {
			return;
		}
		box = box.intersection(baking_bounds);
		box.size.y += buried_probe_height;
		sampler.world_to_voxel = terrain_transform.affine_inverse();
		AABB local = sampler.world_to_voxel.xform(box);
		Vector3i min_pos = math::floor_to_int(local.position) - Vector3i(1, 1, 1);
		Vector3i max_pos = math::floor_to_int(local.position + local.size) + Vector3i(2, 2, 2);
		const Box3i bounds = voxel_data->get_bounds();
		const Box3i clipped = Box3i::from_min_max(min_pos, max_pos).clipped(bounds);
		if (clipped.size.x < 2 || clipped.size.y < 2 || clipped.size.z < 2) {
			return;
		}
		sampler.origin = clipped.position;
		sampler.buffer.create(clipped.size);
		voxel_data->copy(clipped.position, sampler.buffer, 1 << VoxelBuffer::CHANNEL_SDF, false);
		sampler.valid = true;
	}

	void add_static_triangles(PackedFloat32Array &vertices, PackedInt32Array &indices) const {
		ZN_PROFILE_SCOPE();
		if (static_triangles.empty()) {
			return;
		}
		SdfSampler sampler;
		prepare_sdf(sampler);

		StdVector<Vector3> kept;
		kept.reserve(static_triangles.size() * 3);
		StdVector<Vector3> stack;
		for (const int32_t t : static_triangles) {
			const Vector3 a = static_vertex(static_indices[t * 3]);
			const Vector3 b = static_vertex(static_indices[t * 3 + 1]);
			const Vector3 c = static_vertex(static_indices[t * 3 + 2]);
			if (!sampler.valid || !is_walkable(a, b, c)) {
				kept.push_back(a);
				kept.push_back(b);
				kept.push_back(c);
				continue;
			}
			// Split along the longest edge until small enough, then keep the pieces voxels don't bury.
			stack.clear();
			stack.push_back(a);
			stack.push_back(b);
			stack.push_back(c);
			while (!stack.empty()) {
				const Vector3 p2 = stack.back();
				stack.pop_back();
				const Vector3 p1 = stack.back();
				stack.pop_back();
				const Vector3 p0 = stack.back();
				stack.pop_back();
				const real_t l01 = p0.distance_squared_to(p1);
				const real_t l12 = p1.distance_squared_to(p2);
				const real_t l20 = p2.distance_squared_to(p0);
				const real_t longest = MAX(l01, MAX(l12, l20));
				if (longest > BURIED_SAMPLE_SPACING * BURIED_SAMPLE_SPACING) {
					if (longest == l01) {
						const Vector3 m = (p0 + p1) * 0.5;
						stack.insert(stack.end(), { p0, m, p2, m, p1, p2 });
					} else if (longest == l12) {
						const Vector3 m = (p1 + p2) * 0.5;
						stack.insert(stack.end(), { p0, p1, m, p0, m, p2 });
					} else {
						const Vector3 m = (p2 + p0) * 0.5;
						stack.insert(stack.end(), { p0, p1, m, m, p1, p2 });
					}
					continue;
				}
				const Vector3 probe = (p0 + p1 + p2) / 3.0 + Vector3(0, buried_probe_height, 0);
				if (sampler.sample(probe) < 0.f) {
					continue;
				}
				kept.push_back(p0);
				kept.push_back(p1);
				kept.push_back(p2);
			}
		}

		const int64_t vertex_start = vertices.size() / 3;
		const int64_t index_start = indices.size();
		vertices.resize(vertices.size() + kept.size() * 3);
		indices.resize(indices.size() + kept.size());
		float *vw = vertices.ptrw() + vertex_start * 3;
		int32_t *iw = indices.ptrw() + index_start;
		for (size_t i = 0; i < kept.size(); ++i) {
			vw[i * 3 + 0] = kept[i].x;
			vw[i * 3 + 1] = kept[i].y;
			vw[i * 3 + 2] = kept[i].z;
			iw[i] = int32_t(vertex_start + i);
		}
	}
};


VoxelTerrainNavigation::VoxelTerrainNavigation() {
	update_processing_state();
}

VoxelTerrainNavigation::~VoxelTerrainNavigation() {
	collect_bakes(true);
}

void VoxelTerrainNavigation::collect_bakes(bool wait) {
	WorkerThreadPool &pool = *WorkerThreadPool::get_singleton();
	for (size_t i = 0; i < _running_bakes.size();) {
		const RunningBake bake = _running_bakes[i];
		if (!wait && !pool.is_task_completed(bake.task_id)) {
			++i;
			continue;
		}
		pool.wait_for_task_completion(bake.task_id);
		_running_bakes[i] = _running_bakes.back();
		_running_bakes.pop_back();
		_bakes_in_flight = MAX(_bakes_in_flight - 1, 0);
		if (!wait) {
			on_tile_baked(bake.task->key, bake.task->generation, bake.task->navmesh, bake.task->bake_usec);
		}
		ZN_DELETE(bake.task);
	}
}

void VoxelTerrainNavigation::set_enabled(bool p_enabled) {
	if (_enabled == p_enabled) {
		return;
	}
	_enabled = p_enabled;
	for (auto &it : _tiles) {
		if (it.second.region != nullptr) {
			it.second.region->set_enabled(_enabled);
		}
	}
	update_processing_state();
}

bool VoxelTerrainNavigation::is_enabled() const {
	return _enabled;
}

void VoxelTerrainNavigation::set_run_in_editor(bool enable) {
	_run_in_editor = enable;
	update_processing_state();
}

bool VoxelTerrainNavigation::get_run_in_editor() const {
	return _run_in_editor;
}

void VoxelTerrainNavigation::set_template_navigation_mesh(Ref<NavigationMesh> navmesh) {
	_template_navmesh = navmesh;
	_static_dirty = true;
#ifdef TOOLS_ENABLED
	update_configuration_warnings();
#endif
}

Ref<NavigationMesh> VoxelTerrainNavigation::get_template_navigation_mesh() const {
	return _template_navmesh;
}

void VoxelTerrainNavigation::set_static_geometry(Ref<NavigationMeshSourceGeometryData3D> geometry) {
	_static_geometry = geometry;
	_static_dirty = true;
}

Ref<NavigationMeshSourceGeometryData3D> VoxelTerrainNavigation::get_static_geometry() const {
	return _static_geometry;
}

void VoxelTerrainNavigation::set_tile_size_blocks(int size) {
	size = MAX(size, 1);
	if (size == _tile_size_blocks) {
		return;
	}
	_tile_size_blocks = size;
	_static_dirty = true;
#ifdef TOOLS_ENABLED
	update_configuration_warnings();
#endif
}

int VoxelTerrainNavigation::get_tile_size_blocks() const {
	return _tile_size_blocks;
}

void VoxelTerrainNavigation::set_bake_delay(float seconds) {
	_bake_delay = MAX(seconds, 0.f);
}

float VoxelTerrainNavigation::get_bake_delay() const {
	return _bake_delay;
}

void VoxelTerrainNavigation::set_max_bake_interval(float seconds) {
	_max_bake_interval = MAX(seconds, 0.f);
}

float VoxelTerrainNavigation::get_max_bake_interval() const {
	return _max_bake_interval;
}

void VoxelTerrainNavigation::set_max_concurrent_bakes(int count) {
	_max_concurrent_bakes = MAX(count, 1);
}

int VoxelTerrainNavigation::get_max_concurrent_bakes() const {
	return _max_concurrent_bakes;
}

void VoxelTerrainNavigation::set_navigation_layers(uint32_t layers) {
	_navigation_layers = layers;
	for (auto &it : _tiles) {
		if (it.second.region != nullptr) {
			it.second.region->set_navigation_layers(layers);
		}
	}
}

uint32_t VoxelTerrainNavigation::get_navigation_layers() const {
	return _navigation_layers;
}

void VoxelTerrainNavigation::set_drop_buried_static_geometry(bool enable) {
	if (_drop_buried_static_geometry != enable) {
		_drop_buried_static_geometry = enable;
		bake_all();
	}
}

bool VoxelTerrainNavigation::get_drop_buried_static_geometry() const {
	return _drop_buried_static_geometry;
}

void VoxelTerrainNavigation::set_configure_map(bool enable) {
	_configure_map = enable;
}

bool VoxelTerrainNavigation::get_configure_map() const {
	return _configure_map;
}

void VoxelTerrainNavigation::set_obstacle_group(StringName group) {
	_obstacle_group = group;
}

StringName VoxelTerrainNavigation::get_obstacle_group() const {
	return _obstacle_group;
}

// An obstacle's prism reaches a walker's height below it: ground a walker would hit it from is as blocked as ground
// it stands on.
void VoxelTerrainNavigation::update_obstacles() {
	ZN_PROFILE_SCOPE();
	const float reach_down = _template_navmesh->get_agent_height();
	StdUnorderedMap<uint64_t, Obstacle> current;
	if (is_inside_tree() && !_obstacle_group.is_empty()) {
		const TypedArray<Node> nodes = get_tree()->get_nodes_in_group(_obstacle_group);
		for (int i = 0; i < nodes.size(); ++i) {
			const NavigationObstacle3D *node = Object::cast_to<NavigationObstacle3D>(nodes[i]);
			if (node == nullptr || !node->get_affect_navigation_mesh() || !node->is_inside_tree()) {
				continue;
			}
			const Vector3 pos = node->get_global_position();
			Obstacle o;
			const PackedVector3Array local = node->get_vertices();
			if (local.size() >= 3) {
				for (int k = 0; k < local.size(); ++k) {
					o.vertices.push_back(Vector3(pos.x + local[k].x, 0, pos.z + local[k].z));
				}
			} else if (node->get_radius() > 0.f) {
				for (int k = 0; k < OBSTACLE_CIRCLE_SIDES; ++k) {
					const real_t a = Math_TAU * k / OBSTACLE_CIRCLE_SIDES;
					o.vertices.push_back(Vector3(pos.x + Math::cos(a) * node->get_radius(), 0, pos.z + Math::sin(a) * node->get_radius()));
				}
			} else {
				continue;
			}
			o.elevation = pos.y - reach_down;
			o.height = node->get_height() + reach_down;
			o.carve = node->get_carve_navigation_mesh();
			AABB box(o.vertices[0], Vector3());
			for (int k = 1; k < o.vertices.size(); ++k) {
				box.expand_to(o.vertices[k]);
			}
			box.position.y = o.elevation;
			box.size.y = o.height;
			o.bounds = box;
			current[uint64_t(node->get_instance_id())] = o;
		}
	}
	for (const auto &it : current) {
		auto old = _obstacles.find(it.first);
		if (old == _obstacles.end()) {
			mark_dirty_in_aabb(it.second.bounds);
			continue;
		}
		const AABB &a = old->second.bounds;
		const AABB &b = it.second.bounds;
		if (a.position.distance_to(b.position) > OBSTACLE_MOVE_TOLERANCE ||
			(a.size - b.size).length() > OBSTACLE_MOVE_TOLERANCE || old->second.carve != it.second.carve) {
			mark_dirty_in_aabb(a);
			mark_dirty_in_aabb(b);
		} else {
			// Not moved enough to rebake: keep baking it where it was, so the navmesh and the record agree.
			current[it.first] = old->second;
		}
	}
	for (const auto &it : _obstacles) {
		if (current.find(it.first) == current.end()) {
			mark_dirty_in_aabb(it.second.bounds);
		}
	}
	_obstacles.swap(current);
}

void VoxelTerrainNavigation::bake_all() {
	for (auto &it : _tiles) {
		mark_dirty(it.second);
		it.second.quiet_for = _bake_delay;
		it.second.dirty_for = _max_bake_interval;
	}
}

int VoxelTerrainNavigation::get_pending_tile_count() const {
	int count = (_static_dirty && is_running()) ? 1 : 0;
	for (const auto &it : _tiles) {
		if (it.second.dirty_for >= 0.f || it.second.baking) {
			++count;
		}
	}
	return count;
}

Dictionary VoxelTerrainNavigation::get_statistics() const {
	Dictionary d;
	d["tiles"] = int64_t(_tiles.size());
	d["pending_tiles"] = get_pending_tile_count();
	d["bakes_in_flight"] = _bakes_in_flight;
	d["bake_count"] = int64_t(_bake_count);
	d["bake_usec_last"] = _bake_usec_last;
	d["bake_usec_max"] = _bake_usec_max;
	d["bake_usec_average"] = _bake_count > 0 ? int64_t(_bake_usec_total / _bake_count) : int64_t(0);
	d["apply_usec_max"] = _apply_usec_max;
	d["dropped_results"] = _dropped_results;
	d["edit_changes"] = int64_t(_edit_changes);
	d["block_changes"] = int64_t(_block_changes);
	return d;
}

void VoxelTerrainNavigation::update_processing_state() {
	set_process_internal(is_running());
}

bool VoxelTerrainNavigation::is_running() const {
	if (!_enabled) {
		return false;
	}
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint() && !_run_in_editor) {
		return false;
	}
#endif
	return true;
}

VoxelTerrain *VoxelTerrainNavigation::get_terrain() const {
	return Object::cast_to<VoxelTerrain>(get_parent());
}

void VoxelTerrainNavigation::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			update_processing_state();
			break;
		case NOTIFICATION_INTERNAL_PROCESS:
			process(get_process_delta_time());
			break;
		case NOTIFICATION_EXIT_TREE:
			detach();
			break;
		default:
			break;
	}
}

void VoxelTerrainNavigation::attach(VoxelTerrain &terrain) {
	detach();
	_terrain_id = terrain.get_instance_id();
	terrain.set_mesh_change_recording(true);
	_static_dirty = true;
}

void VoxelTerrainNavigation::detach() {
	Object *obj = ObjectDB::get_instance(_terrain_id);
	VoxelTerrain *terrain = Object::cast_to<VoxelTerrain>(obj);
	if (terrain != nullptr) {
		terrain->set_mesh_change_recording(false);
	}
	_terrain_id = ObjectID();
	reset_tiles();
}

void VoxelTerrainNavigation::reset_tiles() {
	for (auto &it : _tiles) {
		if (it.second.region != nullptr) {
			it.second.region->queue_free();
		}
	}
	_tiles.clear();
	// Bakes still running find their tile gone and are dropped.
}

real_t VoxelTerrainNavigation::get_tile_world_size() const {
	const VoxelTerrain *terrain = get_terrain();
	const int block_size = terrain != nullptr ? terrain->get_mesh_block_size() : 16;
	return real_t(_tile_size_blocks * block_size) * _terrain_transform.basis.get_scale().x;
}

float VoxelTerrainNavigation::get_border_size() const {
	if (_template_navmesh.is_null()) {
		return 1.f;
	}
	// Room for the agent's radius to be eroded off the tile edge from geometry beyond it, rounded up to whole cells so
	// neighbouring tiles' grids line up.
	const float cell = _template_navmesh->get_cell_size();
	const float radius = _template_navmesh->get_agent_radius();
	return Math::ceil((radius + 2.f * cell) / cell) * cell;
}

Vector2i VoxelTerrainNavigation::world_to_tile(const Vector3 &pos) const {
	const real_t size = get_tile_world_size();
	const Vector3 origin = _terrain_transform.origin;
	return Vector2i(int(Math::floor((pos.x - origin.x) / size)), int(Math::floor((pos.z - origin.z) / size)));
}

AABB VoxelTerrainNavigation::get_tile_bounds(Vector2i tile_pos) const {
	const real_t size = get_tile_world_size();
	const Vector3 origin = _terrain_transform.origin;
	return AABB(Vector3(origin.x + tile_pos.x * size, _min_y, origin.z + tile_pos.y * size),
			Vector3(size, _max_y - _min_y, size));
}

VoxelTerrainNavigation::Tile &VoxelTerrainNavigation::get_or_create_tile(Vector2i tile_pos) {
	const uint64_t key = tile_key(tile_pos);
	auto it = _tiles.find(key);
	if (it != _tiles.end()) {
		return it->second;
	}
	Tile &tile = _tiles[key];
	tile.region = memnew(NavigationRegion3D);
	// Tiles are baked in world space and meet edge to edge on a shared cell grid, which the map merges by itself.
	tile.region->set_as_top_level(true);
	tile.region->set_use_edge_connections(false);
	tile.region->set_navigation_layers(_navigation_layers);
	tile.region->set_enabled(_enabled);
	add_child(tile.region);
	tile.region->set_global_transform(Transform3D());
	return tile;
}

void VoxelTerrainNavigation::mark_dirty(Tile &tile) {
	if (tile.dirty_for < 0.f) {
		tile.dirty_for = 0.f;
	}
	tile.quiet_for = 0.f;
	if (tile.baking) {
		tile.changed_while_baking = true;
	}
}

void VoxelTerrainNavigation::mark_dirty_in_aabb(const AABB &world_box) {
	const float border = get_border_size();
	const Vector2i from = world_to_tile(world_box.position - Vector3(border, 0, border));
	const Vector2i to = world_to_tile(world_box.position + world_box.size + Vector3(border, 0, border));
	for (int z = from.y; z <= to.y; ++z) {
		for (int x = from.x; x <= to.x; ++x) {
			mark_dirty(get_or_create_tile(Vector2i(x, z)));
		}
	}
}

void VoxelTerrainNavigation::rebuild_static_buckets() {
	ZN_PROFILE_SCOPE();
	_static_dirty = false;
	VoxelTerrain *terrain = get_terrain();
	if (terrain == nullptr || _template_navmesh.is_null()) {
		return;
	}
	reset_tiles();
	_terrain_transform = terrain->get_global_transform();

	// Vertical extent: the terrain's bounds and the static geometry's together.
	const Box3i bounds = terrain->get_bounds();
	const AABB local_bounds(to_vec3(bounds.position), to_vec3(bounds.size));
	AABB extent = _terrain_transform.xform(local_bounds);
	if (_static_geometry.is_valid() && _static_geometry->has_data()) {
		extent = extent.merge(_static_geometry->get_bounds());
	}
	_min_y = extent.position.y - 1.0;
	_max_y = extent.position.y + extent.size.y + 1.0;

	if (_configure_map && is_inside_tree()) {
		Ref<World3D> world = get_world_3d();
		if (world.is_valid()) {
			NavigationServer3D &ns = *NavigationServer3D::get_singleton();
			const RID map = world->get_navigation_map();
			ns.map_set_cell_size(map, _template_navmesh->get_cell_size());
			ns.map_set_cell_height(map, _template_navmesh->get_cell_height());
		}
	}

	const float border = get_border_size();
	if (_static_geometry.is_valid()) {
		const PackedFloat32Array vertices = _static_geometry->get_vertices();
		const PackedInt32Array indices = _static_geometry->get_indices();
		const float *v = vertices.ptr();
		for (int t = 0; t + 2 < indices.size(); t += 3) {
			AABB box(Vector3(v[indices[t] * 3], v[indices[t] * 3 + 1], v[indices[t] * 3 + 2]), Vector3());
			for (int k = 1; k < 3; ++k) {
				const int i = indices[t + k];
				box.expand_to(Vector3(v[i * 3], v[i * 3 + 1], v[i * 3 + 2]));
			}
			const Vector2i from = world_to_tile(box.position - Vector3(border, 0, border));
			const Vector2i to = world_to_tile(box.position + box.size + Vector3(border, 0, border));
			for (int z = from.y; z <= to.y; ++z) {
				for (int x = from.x; x <= to.x; ++x) {
					get_or_create_tile(Vector2i(x, z)).static_triangles.push_back(t / 3);
				}
			}
		}
		const Array obstructions = _static_geometry->get_projected_obstructions();
		for (int i = 0; i < obstructions.size(); ++i) {
			Obstruction o;
			if (!read_obstruction(obstructions[i], o)) {
				continue;
			}
			const AABB box = get_obstruction_bounds(o);
			const Vector2i from = world_to_tile(box.position - Vector3(border, 0, border));
			const Vector2i to = world_to_tile(box.position + box.size + Vector3(border, 0, border));
			for (int z = from.y; z <= to.y; ++z) {
				for (int x = from.x; x <= to.x; ++x) {
					get_or_create_tile(Vector2i(x, z)).static_obstructions.push_back(i);
				}
			}
		}
	}

	// Whatever is meshed already counts as changed; later changes come from the terrain's record.
	const int block_size = terrain->get_mesh_block_size();
	terrain->get_mesh_map().for_each_block([this, block_size](const VoxelMeshBlockVT &block) {
		const AABB local(to_vec3(block.position * block_size), Vector3(block_size, block_size, block_size));
		mark_dirty_in_aabb(_terrain_transform.xform(local));
	});
	for (auto &it : _tiles) {
		mark_dirty(it.second);
	}
}

void VoxelTerrainNavigation::process(float delta_time) {
	ZN_PROFILE_SCOPE();
	collect_bakes(false);
	if (!is_running() || _template_navmesh.is_null()) {
		return;
	}
	VoxelTerrain *terrain = get_terrain();
	if (terrain == nullptr) {
		return;
	}
	if (ObjectID(terrain->get_instance_id()) != _terrain_id) {
		attach(*terrain);
	}
	if (!terrain->get_global_transform().is_equal_approx(_terrain_transform)) {
		_static_dirty = true;
	}
	if (_static_dirty) {
		terrain->consume_mesh_changes(_changes);
		rebuild_static_buckets();
	}

	// Only the tiles whose baking area the changed voxels reach: an edit inside one tile leaves its neighbours alone.
	_changes.clear();
	terrain->consume_mesh_changes(_changes);
	const int block_size = terrain->get_mesh_block_size();
	for (const VoxelTerrain::MeshChange &change : _changes) {
		if (change.voxel_box.size == Vector3i(block_size, block_size, block_size)) {
			++_block_changes;
		} else {
			++_edit_changes;
		}
		const AABB local(to_vec3(change.voxel_box.position), to_vec3(change.voxel_box.size));
		mark_dirty_in_aabb(_terrain_transform.xform(local));
	}

	_obstacle_check_in -= delta_time;
	if (_obstacle_check_in <= 0.f) {
		_obstacle_check_in = OBSTACLE_CHECK_INTERVAL;
		update_obstacles();
	}

	// Longest-waiting first, as many at once as allowed.
	StdVector<std::pair<float, uint64_t>> ready;
	for (auto &it : _tiles) {
		Tile &tile = it.second;
		if (tile.dirty_for < 0.f) {
			continue;
		}
		tile.dirty_for += delta_time;
		tile.quiet_for += delta_time;
		if (!tile.baking && (tile.quiet_for >= _bake_delay || tile.dirty_for >= _max_bake_interval)) {
			ready.push_back({ tile.dirty_for, it.first });
		}
	}
	if (ready.empty() || _bakes_in_flight >= _max_concurrent_bakes) {
		return;
	}
	std::sort(ready.begin(), ready.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
	for (const auto &r : ready) {
		if (_bakes_in_flight >= _max_concurrent_bakes) {
			break;
		}
		start_bake(r.second, _tiles[r.second]);
	}
}

void VoxelTerrainNavigation::start_bake(uint64_t key, Tile &tile) {
	ZN_PROFILE_SCOPE();
	VoxelTerrain *terrain = get_terrain();
	ZN_ASSERT_RETURN(terrain != nullptr);

	const AABB tile_bounds = get_tile_bounds(key_tile(key));
	const float border = get_border_size();
	const AABB baking_bounds = tile_bounds.grow(border);

	BakeTileTask *task = ZN_NEW(BakeTileTask);
	task->key = key;
	task->generation = ++_bake_generation;
	tile.generation = task->generation;
	task->baking_bounds = baking_bounds;
	task->terrain_transform = _terrain_transform;
	task->voxel_data = terrain->get_storage_shared();
	task->drop_buried = _drop_buried_static_geometry;
	task->buried_probe_height = _template_navmesh->get_agent_height() * 0.5f;
	task->walkable_cos = Math::cos(Math::deg_to_rad(real_t(_template_navmesh->get_agent_max_slope())));

	task->navmesh.instantiate();
	copy_navigation_mesh_settings(**task->navmesh, **_template_navmesh);
	task->navmesh->set_filter_baking_aabb(baking_bounds);
	task->navmesh->set_filter_baking_aabb_offset(Vector3());
	task->navmesh->set_border_size(border);

	// Voxel triangles of every mesh block overlapping the baking area. Packed arrays are shared, not copied.
	const int block_size = terrain->get_mesh_block_size();
	const AABB local = _terrain_transform.affine_inverse().xform(baking_bounds);
	const Vector3i bmin = math::floor_to_int(local.position / real_t(block_size));
	const Vector3i bmax = math::floor_to_int((local.position + local.size) / real_t(block_size));
	const VoxelMeshMap<VoxelMeshBlockVT> &mesh_map = terrain->get_mesh_map();
	for (int z = bmin.z; z <= bmax.z; ++z) {
		for (int x = bmin.x; x <= bmax.x; ++x) {
			for (int y = bmin.y; y <= bmax.y; ++y) {
				const VoxelMeshBlockVT *block = mesh_map.get_block(Vector3i(x, y, z));
				if (block == nullptr) {
					continue;
				}
				const Vector3 origin = to_vec3(block->position * block_size);
				for (const VoxelMeshGeometryCache::Surface &s : block->geometry_cache.surfaces) {
					if (s.indices.size() >= 3) {
						task->voxel_surfaces.push_back(BakeTileTask::VoxelSurface{ s.vertices, s.indices, origin });
					}
				}
			}
		}
	}

	if (_static_geometry.is_valid()) {
		task->static_vertices = _static_geometry->get_vertices();
		task->static_indices = _static_geometry->get_indices();
		task->static_triangles = tile.static_triangles;
		if (!tile.static_obstructions.empty()) {
			const Array obstructions = _static_geometry->get_projected_obstructions();
			for (const int32_t i : tile.static_obstructions) {
				Obstruction o;
				if (read_obstruction(obstructions[i], o)) {
					task->obstructions.push_back(o);
				}
			}
		}
	}

	for (const auto &it : _obstacles) {
		if (it.second.bounds.intersects(baking_bounds)) {
			Obstruction o;
			o.vertices = it.second.vertices;
			o.elevation = it.second.elevation;
			o.height = it.second.height;
			o.carve = it.second.carve;
			task->obstructions.push_back(o);
		}
	}

	tile.baking = true;
	tile.changed_while_baking = false;
	++_bakes_in_flight;
	const int64_t task_id = WorkerThreadPool::get_singleton()->add_native_task(
			&BakeTileTask::run_static, task, false, "VoxelTerrainNavigation tile bake"
	);
	_running_bakes.push_back({ task_id, task });
}

void VoxelTerrainNavigation::on_tile_baked(uint64_t key, uint32_t generation, Ref<NavigationMesh> navmesh, uint32_t bake_usec) {
	ZN_PROFILE_SCOPE();
	auto it = _tiles.find(key);
	if (it == _tiles.end() || it->second.generation != generation || !it->second.baking) {
		++_dropped_results;
		return;
	}
	Tile &tile = it->second;
	tile.baking = false;

	++_bake_count;
	_bake_usec_total += bake_usec;
	_bake_usec_last = bake_usec;
	_bake_usec_max = MAX(_bake_usec_max, bake_usec);

	ProfilingClock clock;
	tile.region->set_navigation_mesh(navmesh);
	_apply_usec_max = MAX(_apply_usec_max, uint32_t(clock.get_elapsed_microseconds()));
	tile.baked_once = true;

	if (!tile.changed_while_baking) {
		tile.dirty_for = -1.f;
	}
	tile.changed_while_baking = false;
}

#ifdef TOOLS_ENABLED

#if defined(ZN_GODOT)
PackedStringArray VoxelTerrainNavigation::get_configuration_warnings() const {
	PackedStringArray warnings;
	get_configuration_warnings(warnings);
	return warnings;
}
#elif defined(ZN_GODOT_EXTENSION)
PackedStringArray VoxelTerrainNavigation::_get_configuration_warnings() const {
	PackedStringArray warnings;
	get_configuration_warnings(warnings);
	return warnings;
}
#endif

void VoxelTerrainNavigation::get_configuration_warnings(PackedStringArray &warnings) const {
	if (get_terrain() == nullptr) {
		warnings.append(String("{0} must be a child of a VoxelTerrain.").format(varray(get_class_static())));
	}
	if (_template_navmesh.is_null()) {
		warnings.append("No template navigation mesh is assigned, so there are no settings to bake with.");
		return;
	}
	const VoxelTerrain *terrain = get_terrain();
	if (terrain != nullptr) {
		const real_t tile = real_t(_tile_size_blocks * terrain->get_mesh_block_size()) *
				terrain->get_global_transform().basis.get_scale().x;
		const real_t cells = tile / _template_navmesh->get_cell_size();
		if (Math::abs(cells - Math::round(cells)) > 0.001) {
			warnings.append(String("Tiles are {0} m across, which isn't a whole number of the template's cell_size ({1}), "
								   "so neighbouring tiles won't join up. Pick a cell_size that divides it.")
									.format(varray(tile, _template_navmesh->get_cell_size())));
		}
	}
}

#endif

void VoxelTerrainNavigation::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &VoxelTerrainNavigation::set_enabled);
	ClassDB::bind_method(D_METHOD("is_enabled"), &VoxelTerrainNavigation::is_enabled);

	ClassDB::bind_method(D_METHOD("set_run_in_editor", "enabled"), &VoxelTerrainNavigation::set_run_in_editor);
	ClassDB::bind_method(D_METHOD("get_run_in_editor"), &VoxelTerrainNavigation::get_run_in_editor);

	ClassDB::bind_method(
			D_METHOD("set_template_navigation_mesh", "navmesh"), &VoxelTerrainNavigation::set_template_navigation_mesh
	);
	ClassDB::bind_method(
			D_METHOD("get_template_navigation_mesh"), &VoxelTerrainNavigation::get_template_navigation_mesh
	);

	ClassDB::bind_method(D_METHOD("set_static_geometry", "geometry"), &VoxelTerrainNavigation::set_static_geometry);
	ClassDB::bind_method(D_METHOD("get_static_geometry"), &VoxelTerrainNavigation::get_static_geometry);

	ClassDB::bind_method(D_METHOD("set_tile_size_blocks", "size"), &VoxelTerrainNavigation::set_tile_size_blocks);
	ClassDB::bind_method(D_METHOD("get_tile_size_blocks"), &VoxelTerrainNavigation::get_tile_size_blocks);

	ClassDB::bind_method(D_METHOD("set_bake_delay", "seconds"), &VoxelTerrainNavigation::set_bake_delay);
	ClassDB::bind_method(D_METHOD("get_bake_delay"), &VoxelTerrainNavigation::get_bake_delay);

	ClassDB::bind_method(D_METHOD("set_max_bake_interval", "seconds"), &VoxelTerrainNavigation::set_max_bake_interval);
	ClassDB::bind_method(D_METHOD("get_max_bake_interval"), &VoxelTerrainNavigation::get_max_bake_interval);

	ClassDB::bind_method(
			D_METHOD("set_max_concurrent_bakes", "count"), &VoxelTerrainNavigation::set_max_concurrent_bakes
	);
	ClassDB::bind_method(D_METHOD("get_max_concurrent_bakes"), &VoxelTerrainNavigation::get_max_concurrent_bakes);

	ClassDB::bind_method(D_METHOD("set_navigation_layers", "layers"), &VoxelTerrainNavigation::set_navigation_layers);
	ClassDB::bind_method(D_METHOD("get_navigation_layers"), &VoxelTerrainNavigation::get_navigation_layers);

	ClassDB::bind_method(
			D_METHOD("set_drop_buried_static_geometry", "enabled"),
			&VoxelTerrainNavigation::set_drop_buried_static_geometry
	);
	ClassDB::bind_method(
			D_METHOD("get_drop_buried_static_geometry"), &VoxelTerrainNavigation::get_drop_buried_static_geometry
	);

	ClassDB::bind_method(D_METHOD("set_configure_map", "enabled"), &VoxelTerrainNavigation::set_configure_map);
	ClassDB::bind_method(D_METHOD("get_configure_map"), &VoxelTerrainNavigation::get_configure_map);

	ClassDB::bind_method(D_METHOD("set_obstacle_group", "group"), &VoxelTerrainNavigation::set_obstacle_group);
	ClassDB::bind_method(D_METHOD("get_obstacle_group"), &VoxelTerrainNavigation::get_obstacle_group);

	ClassDB::bind_method(D_METHOD("bake_all"), &VoxelTerrainNavigation::bake_all);
	ClassDB::bind_method(D_METHOD("get_pending_tile_count"), &VoxelTerrainNavigation::get_pending_tile_count);
	ClassDB::bind_method(D_METHOD("get_statistics"), &VoxelTerrainNavigation::get_statistics);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "is_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "run_in_editor"), "set_run_in_editor", "get_run_in_editor");
	ADD_PROPERTY(
			PropertyInfo(
					Variant::OBJECT,
					"template_navigation_mesh",
					PROPERTY_HINT_RESOURCE_TYPE,
					NavigationMesh::get_class_static(),
					PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT
			),
			"set_template_navigation_mesh",
			"get_template_navigation_mesh"
	);
	ADD_PROPERTY(
			PropertyInfo(
					Variant::OBJECT,
					"static_geometry",
					PROPERTY_HINT_RESOURCE_TYPE,
					NavigationMeshSourceGeometryData3D::get_class_static()
			),
			"set_static_geometry",
			"get_static_geometry"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "tile_size_blocks", PROPERTY_HINT_RANGE, "1,8,1"),
			"set_tile_size_blocks",
			"get_tile_size_blocks"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "bake_delay", PROPERTY_HINT_RANGE, "0,5,0.05,or_greater,suffix:s"),
			"set_bake_delay",
			"get_bake_delay"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "max_bake_interval", PROPERTY_HINT_RANGE, "0,10,0.05,or_greater,suffix:s"),
			"set_max_bake_interval",
			"get_max_bake_interval"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "max_concurrent_bakes", PROPERTY_HINT_RANGE, "1,16,1"),
			"set_max_concurrent_bakes",
			"get_max_concurrent_bakes"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "navigation_layers", PROPERTY_HINT_LAYERS_3D_NAVIGATION),
			"set_navigation_layers",
			"get_navigation_layers"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::BOOL, "drop_buried_static_geometry"),
			"set_drop_buried_static_geometry",
			"get_drop_buried_static_geometry"
	);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "configure_map"), "set_configure_map", "get_configure_map");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "obstacle_group"), "set_obstacle_group", "get_obstacle_group");
}

} // namespace zylann::voxel
