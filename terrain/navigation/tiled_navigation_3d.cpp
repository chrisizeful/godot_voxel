#include "tiled_navigation_3d.h"
#include "../../util/godot/classes/engine.h"
#include "../../util/godot/core/array.h"
#include "../../util/godot/classes/array_mesh.h"
#include "../../util/godot/classes/navigation_server_3d.h"
#include "../../util/godot/classes/object.h"
#include "../../util/godot/classes/project_settings.h"
#include "../../util/godot/core/callable_mp.h"
#include "../../util/godot/core/random_pcg.h"
#include "../../util/godot/classes/standard_material_3d.h"
#include "../../util/godot/classes/world_3d.h"
#include "../../util/profiling.h"
#include "../../util/profiling_clock.h"
#include <godot_cpp/classes/navigation_obstacle3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/worker_thread_pool.hpp>

namespace zylann::voxel {

namespace {

// Walkable static triangles are split down to this edge length (world units) before testing whether they're buried.
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

// Reads one projected obstruction as NavigationMeshSourceGeometryData3D returns it.
bool read_obstruction(const Dictionary &d, TileObstruction &out) {
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

AABB get_obstruction_bounds(const TileObstruction &o) {
	AABB box(o.vertices[0], Vector3());
	for (int i = 1; i < o.vertices.size(); ++i) {
		box.expand_to(o.vertices[i]);
	}
	box.position.y = o.elevation;
	box.size.y = o.height;
	return box;
}

} // namespace

// TileBakeTask

void TileBakeTask::run_static(void *self) {
	static_cast<TileBakeTask *>(self)->run();
}

void TileBakeTask::run() {
	ZN_PROFILE_SCOPE();
	ProfilingClock clock;

	PackedFloat32Array vertices;
	PackedInt32Array indices;
	add_own_triangles(vertices, indices);
	add_fixed_triangles(vertices, indices);

	// append_arrays(): set_indices() refuses more indices than vertex floats, which indexed meshes have.
	Ref<NavigationMeshSourceGeometryData3D> source;
	source.instantiate();
	source->append_arrays(vertices, indices);
	for (const TileObstruction &o : obstructions) {
		source->add_projected_obstruction(o.vertices, o.elevation, o.height, o.carve);
	}
	NavigationServer3D::get_singleton()->bake_from_source_geometry_data(navmesh, source, Callable());
	bake_usec = clock.get_elapsed_microseconds();
}

bool TileBakeTask::is_walkable(const Vector3 &a, const Vector3 &b, const Vector3 &c) const {
	const Vector3 n = (b - a).cross(c - a);
	const real_t len = n.length();
	return len > 0.0 && n.y / len >= walkable_cos;
}

void TileBakeTask::add_fixed_triangles(PackedFloat32Array &vertices, PackedInt32Array &indices) {
	ZN_PROFILE_SCOPE();
	StdVector<Vector3> candidates;
	candidates.reserve(static_triangles.size() * 3 + extra_indices.size());
	const float *sv = static_vertices.ptr();
	for (const int32_t t : static_triangles) {
		for (int k = 0; k < 3; ++k) {
			const int64_t i = int64_t(static_indices[t * 3 + k]) * 3;
			candidates.push_back(Vector3(sv[i], sv[i + 1], sv[i + 2]));
		}
	}
	const float *ev = extra_vertices.ptr();
	for (int t = 0; t + 2 < extra_indices.size(); t += 3) {
		for (int k = 0; k < 3; ++k) {
			const int64_t i = int64_t(extra_indices[t + k]) * 3;
			candidates.push_back(Vector3(ev[i], ev[i + 1], ev[i + 2]));
		}
	}
	if (candidates.empty()) {
		return;
	}

	// Only walkable triangles can be buried floors.
	AABB walkable_box;
	bool any_walkable = false;
	for (size_t t = 0; t + 2 < candidates.size(); t += 3) {
		if (!is_walkable(candidates[t], candidates[t + 1], candidates[t + 2])) {
			continue;
		}
		if (!any_walkable) {
			walkable_box = AABB(candidates[t], Vector3());
			any_walkable = true;
		}
		walkable_box.expand_to(candidates[t]);
		walkable_box.expand_to(candidates[t + 1]);
		walkable_box.expand_to(candidates[t + 2]);
	}
	const bool test_burial = any_walkable && prepare_burial(walkable_box.intersection(baking_bounds));

	StdVector<Vector3> kept;
	kept.reserve(candidates.size());
	StdVector<Vector3> stack;
	for (size_t t = 0; t + 2 < candidates.size(); t += 3) {
		const Vector3 a = candidates[t];
		const Vector3 b = candidates[t + 1];
		const Vector3 c = candidates[t + 2];
		if (!test_burial || !is_walkable(a, b, c)) {
			kept.push_back(a);
			kept.push_back(b);
			kept.push_back(c);
			continue;
		}
		// Split along the longest edge until small enough, then keep the pieces that aren't buried.
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
			if (is_buried((p0 + p1 + p2) / 3.0)) {
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

// TiledNavigation3D

TiledNavigation3D::TiledNavigation3D() {
	update_processing_state();
}

TiledNavigation3D::~TiledNavigation3D() {
	collect_bakes(true);
}

void TiledNavigation3D::set_enabled(bool p_enabled) {
	if (_enabled == p_enabled) {
		return;
	}
	_enabled = p_enabled;
	NavigationServer3D &ns = *NavigationServer3D::get_singleton();
	for (auto &it : _tiles) {
		if (it.second.region.is_valid()) {
			ns.region_set_enabled(it.second.region, _enabled);
		}
		apply_debug_materials(it.second);
	}
	update_processing_state();
}

bool TiledNavigation3D::is_enabled() const {
	return _enabled;
}

void TiledNavigation3D::set_run_in_editor(bool enable) {
	_run_in_editor = enable;
	update_processing_state();
}

bool TiledNavigation3D::get_run_in_editor() const {
	return _run_in_editor;
}

void TiledNavigation3D::set_template_navigation_mesh(Ref<NavigationMesh> navmesh) {
	_template_navmesh = navmesh;
	request_layout();
#ifdef TOOLS_ENABLED
	update_configuration_warnings();
#endif
}

Ref<NavigationMesh> TiledNavigation3D::get_template_navigation_mesh() const {
	return _template_navmesh;
}

void TiledNavigation3D::set_static_geometry(Ref<NavigationMeshSourceGeometryData3D> geometry) {
	_static_geometry = geometry;
	request_layout();
}

Ref<NavigationMeshSourceGeometryData3D> TiledNavigation3D::get_static_geometry() const {
	return _static_geometry;
}

void TiledNavigation3D::set_source_geometry_root(NodePath path) {
	_source_geometry_root = path;
	_parsed_geometry.unref();
	request_layout();
}

NodePath TiledNavigation3D::get_source_geometry_root() const {
	return _source_geometry_root;
}

void TiledNavigation3D::set_tile_size(real_t size) {
	_tile_size = MAX(size, real_t(1.0));
	request_layout();
#ifdef TOOLS_ENABLED
	update_configuration_warnings();
#endif
}

real_t TiledNavigation3D::get_tile_size() const {
	return _tile_size;
}

void TiledNavigation3D::set_bake_delay(float seconds) {
	_bake_delay = MAX(seconds, 0.f);
}

float TiledNavigation3D::get_bake_delay() const {
	return _bake_delay;
}

void TiledNavigation3D::set_max_bake_interval(float seconds) {
	_max_bake_interval = MAX(seconds, 0.f);
}

float TiledNavigation3D::get_max_bake_interval() const {
	return _max_bake_interval;
}

void TiledNavigation3D::set_max_concurrent_bakes(int count) {
	_max_concurrent_bakes = MAX(count, 1);
}

int TiledNavigation3D::get_max_concurrent_bakes() const {
	return _max_concurrent_bakes;
}

void TiledNavigation3D::set_navigation_layers(uint32_t layers) {
	_navigation_layers = layers;
	for (auto &it : _tiles) {
		if (it.second.region.is_valid()) {
			NavigationServer3D::get_singleton()->region_set_navigation_layers(it.second.region, layers);
		}
	}
}

uint32_t TiledNavigation3D::get_navigation_layers() const {
	return _navigation_layers;
}

void TiledNavigation3D::set_configure_map(bool enable) {
	_configure_map = enable;
}

bool TiledNavigation3D::get_configure_map() const {
	return _configure_map;
}

void TiledNavigation3D::set_obstacle_group(StringName group) {
	_obstacle_group = group;
}

StringName TiledNavigation3D::get_obstacle_group() const {
	return _obstacle_group;
}

void TiledNavigation3D::set_navigation_map(RID map) {
	_navigation_map = map;
	request_layout();
}

RID TiledNavigation3D::get_navigation_map() const {
	if (_navigation_map.is_valid()) {
		return _navigation_map;
	}
	if (is_inside_tree()) {
		Ref<World3D> world = get_world_3d();
		if (world.is_valid()) {
			return world->get_navigation_map();
		}
	}
	return RID();
}

void TiledNavigation3D::set_tile_geometry_callback(Callable callback) {
	_tile_geometry_callback = callback;
	bake_all();
}

Callable TiledNavigation3D::get_tile_geometry_callback() const {
	return _tile_geometry_callback;
}

void TiledNavigation3D::set_obstacle_settle_time(float seconds) {
	_obstacle_settle_time = MAX(seconds, 0.f);
}

float TiledNavigation3D::get_obstacle_settle_time() const {
	return _obstacle_settle_time;
}

void TiledNavigation3D::set_use_edge_connections(bool enable) {
	_use_edge_connections = enable;
	for (auto &it : _tiles) {
		if (it.second.region.is_valid()) {
			NavigationServer3D::get_singleton()->region_set_use_edge_connections(it.second.region, enable);
		}
	}
}

bool TiledNavigation3D::get_use_edge_connections() const {
	return _use_edge_connections;
}

void TiledNavigation3D::set_area(AABB area) {
	_area = area;
	request_layout();
}

AABB TiledNavigation3D::get_area() const {
	return _area;
}

void TiledNavigation3D::bake_all() {
	for (auto &it : _tiles) {
		mark_dirty(it.second);
		it.second.quiet_for = _bake_delay;
		it.second.dirty_for = _max_bake_interval;
	}
}

int TiledNavigation3D::get_pending_tile_count() const {
	int count = (_layout_due && is_running()) ? 1 : 0;
	for (const auto &it : _tiles) {
		if (it.second.dirty_for >= 0.f || it.second.baking) {
			++count;
		}
	}
	return count;
}

Dictionary TiledNavigation3D::get_statistics() const {
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
	int64_t carving = 0;
	for (const auto &it : _obstacles) {
		carving += it.second.carving ? 1 : 0;
	}
	d["obstacles"] = carving;
	d["layouts"] = _layout_count;
	d["fixed_triangles"] = _fixed_triangle_count;
	add_statistics(d);
	return d;
}

void TiledNavigation3D::request_layout() {
	_layout_due = true;
}

void TiledNavigation3D::update_processing_state() {
	set_process_internal(is_running());
}

bool TiledNavigation3D::is_running() const {
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

void TiledNavigation3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			update_processing_state();
			request_layout();
			if (is_debug_drawn()) {
				NavigationServer3D::get_singleton()->connect(
						"map_changed", callable_mp(this, &TiledNavigation3D::_on_navigation_map_changed));
			}
			break;
		case NOTIFICATION_INTERNAL_PROCESS:
			process(get_process_delta_time());
			break;
		case NOTIFICATION_EXIT_TREE: {
			reset_tiles();
			_debug_connections.destroy();
			const Callable on_map_changed = callable_mp(this, &TiledNavigation3D::_on_navigation_map_changed);
			if (NavigationServer3D::get_singleton()->is_connected("map_changed", on_map_changed)) {
				NavigationServer3D::get_singleton()->disconnect("map_changed", on_map_changed);
			}
		} break;
		case NOTIFICATION_VISIBILITY_CHANGED:
			for (auto &it : _tiles) {
				if (it.second.debug_mesh.is_valid()) {
					it.second.debug_mesh.set_visible(is_visible_in_tree());
				}
			}
			if (_debug_connections.is_valid()) {
				_debug_connections.set_visible(is_visible_in_tree());
			}
			break;
		default:
			break;
	}
}

real_t TiledNavigation3D::get_tile_world_size() const {
	return _tile_size;
}

Vector3 TiledNavigation3D::get_grid_origin() const {
	return Vector3();
}

void TiledNavigation3D::reset_tiles() {
	for (auto &it : _tiles) {
		free_tile(it.second);
	}
	_tiles.clear();
	// Bakes still running find their tile gone and are dropped.
}

float TiledNavigation3D::get_border_size() const {
	if (_template_navmesh.is_null()) {
		return 1.f;
	}
	// Room for the agent's radius to be eroded off the tile edge from geometry beyond it, plus Recast's three cells for
	// tiles, rounded up to whole cells so neighbouring tiles' grids line up (less a hair, so a whole number in floats
	// isn't rounded up a cell).
	const float cell = _template_navmesh->get_cell_size();
	const float radius = _template_navmesh->get_agent_radius();
	return Math::ceil((radius + 3.f * cell) / cell - 0.001f) * cell;
}

Vector2i TiledNavigation3D::world_to_tile(const Vector3 &pos) const {
	const real_t size = get_tile_world_size();
	const Vector3 origin = get_grid_origin();
	return Vector2i(int(Math::floor((pos.x - origin.x) / size)), int(Math::floor((pos.z - origin.z) / size)));
}

AABB TiledNavigation3D::get_tile_bounds(Vector2i tile_pos) const {
	const real_t size = get_tile_world_size();
	const Vector3 origin = get_grid_origin();
	return AABB(Vector3(origin.x + tile_pos.x * size, _min_y, origin.z + tile_pos.y * size),
			Vector3(size, _max_y - _min_y, size));
}

TiledNavigation3D::Tile &TiledNavigation3D::get_or_create_tile(Vector2i tile_pos) {
	const uint64_t key = tile_key(tile_pos);
	auto it = _tiles.find(key);
	if (it != _tiles.end()) {
		return it->second;
	}
	Tile &tile = _tiles[key];
	// Tiles are baked in world space (an identity transform) and meet edge to edge on a shared cell grid, which the map
	// merges by itself.
	NavigationServer3D &ns = *NavigationServer3D::get_singleton();
	tile.region = ns.region_create();
	ns.region_set_owner_id(tile.region, get_instance_id());
	ns.region_set_use_edge_connections(tile.region, _use_edge_connections);
	ns.region_set_navigation_layers(tile.region, _navigation_layers);
	ns.region_set_enabled(tile.region, _enabled);
	ns.region_set_map(tile.region, get_navigation_map());
	return tile;
}

void TiledNavigation3D::free_tile(Tile &tile) {
	if (tile.region.is_valid()) {
		zylann::godot::free_navigation_server_rid(*NavigationServer3D::get_singleton(), tile.region);
		tile.region = RID();
	}
	tile.debug_mesh.destroy();
	tile.debug_arrays.unref();
}

// Drawn as NavigationRegion3D draws its navmesh: in the editor, and in a game run with Visible Navigation.
bool TiledNavigation3D::is_debug_drawn() const {
	if (!is_inside_tree()) {
		return false;
	}
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		return true;
	}
#endif
	return get_tree()->is_debugging_navigation_hint();
}

void TiledNavigation3D::make_debug_materials() {
	if (_debug_face_material.is_valid()) {
		return;
	}
	ProjectSettings &settings = *ProjectSettings::get_singleton();
	const String prefix = "debug/shapes/navigation/3d/";
	auto make = [&settings, &prefix](const char *color_setting, bool transparent, const char *xray_setting) {
		Ref<StandardMaterial3D> material;
		material.instantiate();
		material->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
		if (transparent) {
			material->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
		}
		material->set_albedo(settings.get_setting(prefix + String(color_setting)));
		material->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
		if (xray_setting != nullptr && bool(settings.get_setting(prefix + String(xray_setting)))) {
			material->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
		}
		return material;
	};
	_debug_face_material = make("geometry_face_color", true, nullptr);
	_debug_face_material->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
	if (bool(settings.get_setting(prefix + String("enable_geometry_face_random_color")))) {
		_debug_face_material->set_flag(StandardMaterial3D::FLAG_SRGB_VERTEX_COLOR, true);
		_debug_face_material->set_flag(StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	}
	_debug_face_disabled_material = make("geometry_face_disabled_color", true, nullptr);
	_debug_edge_material = make("geometry_edge_color", false, "enable_edge_lines_xray");
	_debug_edge_disabled_material = make("geometry_edge_disabled_color", false, "enable_edge_lines_xray");
	_debug_connections_material = make("edge_connection_color", false, "enable_edge_connections_xray");
	_debug_connections_material->set_render_priority(StandardMaterial3D::RENDER_PRIORITY_MAX - 2);
}

void TiledNavigation3D::apply_debug_materials(Tile &tile) {
	if (tile.debug_arrays.is_null()) {
		return;
	}
	tile.debug_arrays->surface_set_material(0, _enabled ? _debug_face_material : _debug_face_disabled_material);
	if (tile.debug_arrays->get_surface_count() > 1) {
		tile.debug_arrays->surface_set_material(1, _enabled ? _debug_edge_material : _debug_edge_disabled_material);
	}
}

void TiledNavigation3D::update_tile_debug(Tile &tile, NavigationMesh *navmesh) {
	if (!is_debug_drawn() || navmesh == nullptr || navmesh->get_polygon_count() == 0) {
		tile.debug_mesh.destroy();
		tile.debug_arrays.unref();
		return;
	}
	make_debug_materials();
	ProjectSettings &settings = *ProjectSettings::get_singleton();
	const Color face_color = settings.get_setting("debug/shapes/navigation/3d/geometry_face_color");
	const bool random_faces = settings.get_setting("debug/shapes/navigation/3d/enable_geometry_face_random_color");
	const bool edge_lines = settings.get_setting("debug/shapes/navigation/3d/enable_edge_lines");

	const auto vertices = navmesh->get_vertices();
	PackedVector3Array face_points;
	PackedColorArray face_colors;
	PackedVector3Array edge_points;
	// Same tints every rebake, so a tile doesn't flicker as it rebakes.
	RandomPCG rand;
	rand.seed(1);
	for (int p = 0; p < navmesh->get_polygon_count(); ++p) {
		const auto polygon = navmesh->get_polygon(p);
		if (polygon.size() < 3) {
			continue;
		}
		Color color = face_color;
		if (random_faces) {
			// As NavigationRegion3D tints them: hue and value moved a little from the setting's.
			color.set_hsv(face_color.get_h() + rand.random(-1.0, 1.0) * 0.1, face_color.get_s(),
					face_color.get_v() + rand.random(-1.0, 1.0) * 0.2);
			color.a = face_color.a;
		}
		for (int i = 1; i + 1 < polygon.size(); ++i) {
			face_points.push_back(vertices[polygon[0]]);
			face_points.push_back(vertices[polygon[i]]);
			face_points.push_back(vertices[polygon[i + 1]]);
			for (int k = 0; k < 3 && random_faces; ++k) {
				face_colors.push_back(color);
			}
		}
		if (edge_lines) {
			for (int i = 0; i < polygon.size(); ++i) {
				edge_points.push_back(vertices[polygon[i]]);
				edge_points.push_back(vertices[polygon[(i + 1) % polygon.size()]]);
			}
		}
	}
	Ref<ArrayMesh> mesh;
	mesh.instantiate();
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = face_points;
	if (random_faces) {
		arrays[Mesh::ARRAY_COLOR] = face_colors;
	}
	mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	if (!edge_points.is_empty()) {
		Array lines;
		lines.resize(Mesh::ARRAY_MAX);
		lines[Mesh::ARRAY_VERTEX] = edge_points;
		mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, lines);
	}
	tile.debug_arrays = mesh;
	apply_debug_materials(tile);
	if (!tile.debug_mesh.is_valid()) {
		tile.debug_mesh.create();
		tile.debug_mesh.set_world(get_world_3d().ptr());
		tile.debug_mesh.set_transform(Transform3D());
		tile.debug_mesh.set_cast_shadows_setting(RenderingServerEnums::SHADOW_CASTING_SETTING_OFF);
		tile.debug_mesh.set_visible(is_visible_in_tree());
	}
	tile.debug_mesh.set_mesh(mesh);
}

void TiledNavigation3D::_on_navigation_map_changed(RID map) {
	if (map == get_navigation_map()) {
		update_debug_connections();
	}
}

// Every tile's connections to other regions, drawn as NavigationRegion3D draws its own.
void TiledNavigation3D::update_debug_connections() {
	NavigationServer3D &ns = *NavigationServer3D::get_singleton();
	const RID map = get_navigation_map();
	const bool shown = is_debug_drawn() && _use_edge_connections && map.is_valid() && ns.map_get_use_edge_connections(map) &&
			bool(ProjectSettings::get_singleton()->get_setting("debug/shapes/navigation/3d/enable_edge_connections"));
	PackedVector3Array points;
	if (shown) {
		const real_t half_margin = ns.map_get_edge_connection_margin(map) * 0.5;
		for (auto &it : _tiles) {
			const RID region = it.second.region;
			if (!region.is_valid()) {
				continue;
			}
			for (int i = 0; i < ns.region_get_connections_count(region); ++i) {
				const Vector3 start = ns.region_get_connection_pathway_start(region, i);
				const Vector3 end = ns.region_get_connection_pathway_end(region, i);
				const Vector3 right = start.direction_to(end).cross(Vector3(0, 1, 0));
				points.push_back(start);
				points.push_back(end);
				points.push_back(start - right * half_margin);
				points.push_back(start + right * half_margin);
				points.push_back(end + right * half_margin);
				points.push_back(end - right * half_margin);
			}
		}
	}
	if (points.is_empty()) {
		_debug_connections.destroy();
		return;
	}
	make_debug_materials();
	Ref<ArrayMesh> mesh;
	mesh.instantiate();
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = points;
	mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);
	mesh->surface_set_material(0, _debug_connections_material);
	if (!_debug_connections.is_valid()) {
		_debug_connections.create();
		_debug_connections.set_world(get_world_3d().ptr());
		_debug_connections.set_transform(Transform3D());
		_debug_connections.set_cast_shadows_setting(RenderingServerEnums::SHADOW_CASTING_SETTING_OFF);
		_debug_connections.set_visible(is_visible_in_tree());
	}
	_debug_connections.set_mesh(mesh);
}

void TiledNavigation3D::mark_dirty(Tile &tile) {
	if (tile.dirty_for < 0.f) {
		tile.dirty_for = 0.f;
	}
	tile.quiet_for = 0.f;
	if (tile.baking) {
		tile.changed_while_baking = true;
	}
}

void TiledNavigation3D::mark_dirty_in_aabb(const AABB &world_box) {
	const float border = get_border_size();
	const Vector2i from = world_to_tile(world_box.position - Vector3(border, 0, border));
	const Vector2i to = world_to_tile(world_box.position + world_box.size + Vector3(border, 0, border));
	for (int z = from.y; z <= to.y; ++z) {
		for (int x = from.x; x <= to.x; ++x) {
			mark_dirty(get_or_create_tile(Vector2i(x, z)));
		}
	}
}

Ref<NavigationMeshSourceGeometryData3D> TiledNavigation3D::get_fixed_geometry() const {
	return _static_geometry.is_valid() ? _static_geometry : _parsed_geometry;
}

// Groups span the whole tree: an obstacle counts only in the World3D this node is in, so worlds side by side (each under
// its own viewport) don't carve each other's navmeshes.
bool TiledNavigation3D::is_in_own_world(const Node3D &node) const {
	return node.is_inside_tree() && node.get_world_3d() == get_world_3d();
}

// Parses source_geometry_root once, as the template says to; obstacles this node carves are switched off meanwhile, so
// someone sitting down at load isn't baked in for good.
void TiledNavigation3D::parse_source_geometry() {
	_parsed_geometry.unref();
	Node *root = _source_geometry_root.is_empty() ? nullptr : get_node_or_null(_source_geometry_root);
	if (root == nullptr || _template_navmesh.is_null()) {
		return;
	}
	StdVector<NavigationObstacle3D *> carving;
	if (!_obstacle_group.is_empty()) {
		const TypedArray<Node> nodes = get_tree()->get_nodes_in_group(_obstacle_group);
		for (int i = 0; i < nodes.size(); ++i) {
			NavigationObstacle3D *node = Object::cast_to<NavigationObstacle3D>(nodes[i]);
			if (node != nullptr && node->get_affect_navigation_mesh() && is_in_own_world(*node)) {
				node->set_affect_navigation_mesh(false);
				carving.push_back(node);
			}
		}
	}
	Ref<NavigationMeshSourceGeometryData3D> local;
	local.instantiate();
	NavigationServer3D::get_singleton()->parse_source_geometry_data(_template_navmesh, local, root);
	for (NavigationObstacle3D *node : carving) {
		node->set_affect_navigation_mesh(true);
	}

	// The parse is relative to the root (as a region bakes in its own space); tiles are in world space.
	const Node3D *root_3d = Object::cast_to<Node3D>(root);
	const Transform3D to_world = root_3d != nullptr ? root_3d->get_global_transform() : Transform3D();
	PackedFloat32Array vertices = local->get_vertices();
	float *v = vertices.ptrw();
	for (int i = 0; i + 2 < vertices.size(); i += 3) {
		const Vector3 w = to_world.xform(Vector3(v[i], v[i + 1], v[i + 2]));
		v[i] = w.x;
		v[i + 1] = w.y;
		v[i + 2] = w.z;
	}
	_parsed_geometry.instantiate();
	_parsed_geometry->append_arrays(vertices, local->get_indices());
	const Array obstructions = local->get_projected_obstructions();
	for (int i = 0; i < obstructions.size(); ++i) {
		TileObstruction o;
		if (!read_obstruction(obstructions[i], o)) {
			continue;
		}
		const real_t base_y = to_world.xform(Vector3(o.vertices[0].x, o.elevation, o.vertices[0].z)).y;
		for (int k = 0; k < o.vertices.size(); ++k) {
			o.vertices.set(k, to_world.xform(o.vertices[k]));
		}
		_parsed_geometry->add_projected_obstruction(o.vertices, base_y, o.height, o.carve);
	}
}

void TiledNavigation3D::lay_out_tiles() {
	ZN_PROFILE_SCOPE();
	_layout_due = false;
	if (_template_navmesh.is_null() || !is_inside_tree()) {
		return;
	}
	reset_tiles();
	if (_static_geometry.is_null()) {
		parse_source_geometry();
	}
	const Ref<NavigationMeshSourceGeometryData3D> fixed = get_fixed_geometry();
	++_layout_count;
	_fixed_triangle_count = fixed.is_valid() ? fixed->get_indices().size() / 3 : -1;

	// Vertical extent: the fixed geometry's and the subclass's together.
	AABB extent;
	bool has_extent = false;
	if (fixed.is_valid() && fixed->has_data()) {
		extent = fixed->get_bounds();
		has_extent = true;
	}
	merge_extent(extent, has_extent);
	if (_area.size.x > 0 && _area.size.z > 0) {
		extent = has_extent ? extent.merge(_area) : _area;
		has_extent = true;
	}
	// On a world-aligned grid of cell heights, so every tile (of every node on the map) slices heights the same way.
	const real_t cell_height = _template_navmesh->get_cell_height();
	_min_y = Math::floor(((has_extent ? extent.position.y : real_t(0)) - 1.0) / cell_height) * cell_height;
	_max_y = Math::ceil(((has_extent ? extent.position.y + extent.size.y : real_t(0)) + 1.0) / cell_height) * cell_height;

	const RID map = get_navigation_map();
	if (_configure_map && map.is_valid()) {
		NavigationServer3D &ns = *NavigationServer3D::get_singleton();
		ns.map_set_cell_size(map, _template_navmesh->get_cell_size());
		ns.map_set_cell_height(map, _template_navmesh->get_cell_height());
	}

	const float border = get_border_size();
	if (fixed.is_valid()) {
		const PackedFloat32Array vertices = fixed->get_vertices();
		const PackedInt32Array indices = fixed->get_indices();
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
		const Array obstructions = fixed->get_projected_obstructions();
		for (int i = 0; i < obstructions.size(); ++i) {
			TileObstruction o;
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

	if (_area.size.x > 0 && _area.size.z > 0) {
		const Vector2i from = world_to_tile(_area.position);
		const Vector2i to = world_to_tile(_area.position + _area.size);
		for (int z = from.y; z <= to.y; ++z) {
			for (int x = from.x; x <= to.x; ++x) {
				get_or_create_tile(Vector2i(x, z));
			}
		}
	}

	on_tiles_laid_out();
	for (auto &it : _tiles) {
		mark_dirty(it.second);
	}
	// Obstacles are carved into the tiles as they are now, so all of them count as new.
	_obstacles.clear();
	_obstacle_check_in = 0.f;
}

// An obstacle's prism reaches a walker's height below it: ground a walker would hit it from is as blocked as ground
// it stands on.
void TiledNavigation3D::update_obstacles() {
	ZN_PROFILE_SCOPE();
	const float reach_down = _template_navmesh->get_agent_height();
	StdUnorderedMap<uint64_t, Obstacle> current;
	if (is_inside_tree() && !_obstacle_group.is_empty()) {
		const TypedArray<Node> nodes = get_tree()->get_nodes_in_group(_obstacle_group);
		for (int i = 0; i < nodes.size(); ++i) {
			const NavigationObstacle3D *node = Object::cast_to<NavigationObstacle3D>(nodes[i]);
			if (node == nullptr || !node->get_affect_navigation_mesh() || !is_in_own_world(*node)) {
				continue;
			}
			const Vector3 pos = node->get_global_position();
			Obstacle o;
			const PackedVector3Array local = node->get_vertices();
			// As Godot's own obstacle parser does: projected onto the ground, so turned about Y and scaled, nothing else.
			const Vector3 scale = node->get_global_basis().get_scale().abs().maxf(0.001);
			if (local.size() >= 3) {
				const Transform3D xform(Basis().scaled(scale).rotated(Vector3(0, 1, 0), node->get_global_rotation().y), pos);
				for (int k = 0; k < local.size(); ++k) {
					const Vector3 v = xform.xform(local[k]);
					o.obstruction.vertices.push_back(Vector3(v.x, 0, v.z));
				}
			} else if (node->get_radius() > 0.f) {
				const real_t radius = node->get_radius() * scale[scale.max_axis_index()];
				for (int k = 0; k < OBSTACLE_CIRCLE_SIDES; ++k) {
					const real_t a = Math_TAU * k / OBSTACLE_CIRCLE_SIDES;
					o.obstruction.vertices.push_back(Vector3(pos.x + Math::cos(a) * radius, 0, pos.z + Math::sin(a) * radius));
				}
			} else {
				continue;
			}
			o.obstruction.elevation = pos.y - reach_down;
			o.obstruction.height = node->get_height() + reach_down;
			o.obstruction.carve = node->get_carve_navigation_mesh();
			o.bounds = get_obstruction_bounds(o.obstruction);
			current[uint64_t(node->get_instance_id())] = o;
		}
	}
	// Carving is for what stands still: something on the move (a vehicle's driver, a burrower charging) is left to
	// avoidance, or every tile along its way would rebake as it passes.
	for (auto &it : current) {
		Obstacle &now = it.second;
		auto old = _obstacles.find(it.first);
		if (old == _obstacles.end()) {
			continue;
		}
		const Obstacle &was = old->second;
		const bool moved = was.bounds.position.distance_to(now.bounds.position) > OBSTACLE_MOVE_TOLERANCE ||
				(was.bounds.size - now.bounds.size).length() > OBSTACLE_MOVE_TOLERANCE ||
				was.obstruction.carve != now.obstruction.carve;
		if (moved) {
			if (was.carving) {
				mark_dirty_in_aabb(was.bounds);
			}
			continue;
		}
		// Not moved enough to count: keep it where it was, so the navmesh and the record agree.
		const float still_for = was.still_for + OBSTACLE_CHECK_INTERVAL;
		const bool carving = was.carving || still_for >= _obstacle_settle_time;
		now = was;
		now.still_for = still_for;
		if (carving && !was.carving) {
			mark_dirty_in_aabb(now.bounds);
		}
		now.carving = carving;
	}
	for (const auto &it : _obstacles) {
		if (it.second.carving && current.find(it.first) == current.end()) {
			mark_dirty_in_aabb(it.second.bounds);
		}
	}
	_obstacles.swap(current);
}

void TiledNavigation3D::process(float delta_time) {
	ZN_PROFILE_SCOPE();
	collect_bakes(false);
	if (!is_running() || _template_navmesh.is_null()) {
		return;
	}
	update_before_bakes();
	if (_layout_due) {
		lay_out_tiles();
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

TileBakeTask *TiledNavigation3D::create_bake_task(const AABB &baking_bounds) {
	return ZN_NEW(TileBakeTask);
}

void TiledNavigation3D::start_bake(uint64_t key, Tile &tile) {
	ZN_PROFILE_SCOPE();
	const AABB tile_bounds = get_tile_bounds(key_tile(key));
	const float border = get_border_size();
	const AABB baking_bounds = tile_bounds.grow(border);

	TileBakeTask *task = create_bake_task(baking_bounds);
	task->key = key;
	task->generation = ++_bake_generation;
	tile.generation = task->generation;
	task->baking_bounds = baking_bounds;
	task->walkable_cos = Math::cos(Math::deg_to_rad(real_t(_template_navmesh->get_agent_max_slope())));

	task->navmesh.instantiate();
	copy_navigation_mesh_settings(**task->navmesh, **_template_navmesh);
	task->navmesh->set_filter_baking_aabb(baking_bounds);
	task->navmesh->set_filter_baking_aabb_offset(Vector3());
	task->navmesh->set_border_size(border);
	// Tile edges only stay on the tile's bounds (so neighbours' edges meet) with this at 1 or under.
	task->navmesh->set_edge_max_error(MIN(task->navmesh->get_edge_max_error(), 1.0f));

	const Ref<NavigationMeshSourceGeometryData3D> fixed = get_fixed_geometry();
	if (fixed.is_valid()) {
		task->static_vertices = fixed->get_vertices();
		task->static_indices = fixed->get_indices();
		task->static_triangles = tile.static_triangles;
		if (!tile.static_obstructions.empty()) {
			const Array obstructions = fixed->get_projected_obstructions();
			for (const int32_t i : tile.static_obstructions) {
				TileObstruction o;
				if (read_obstruction(obstructions[i], o)) {
					task->obstructions.push_back(o);
				}
			}
		}
	}

	if (_tile_geometry_callback.is_valid()) {
		Ref<NavigationMeshSourceGeometryData3D> extra;
		extra.instantiate();
		_tile_geometry_callback.call(baking_bounds, extra);
		task->extra_vertices = extra->get_vertices();
		task->extra_indices = extra->get_indices();
	}

	for (const auto &it : _obstacles) {
		if (it.second.carving && it.second.bounds.intersects(baking_bounds)) {
			task->obstructions.push_back(it.second.obstruction);
		}
	}

	tile.baking = true;
	tile.changed_while_baking = false;
	++_bakes_in_flight;
	const int64_t task_id = WorkerThreadPool::get_singleton()->add_native_task(
			&TileBakeTask::run_static, task, false, "TiledNavigation3D tile bake"
	);
	_running_bakes.push_back({ task_id, task });
}

void TiledNavigation3D::collect_bakes(bool wait) {
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

void TiledNavigation3D::on_tile_baked(uint64_t key, uint32_t generation, Ref<NavigationMesh> navmesh, uint32_t bake_usec) {
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
	NavigationServer3D::get_singleton()->region_set_navigation_mesh(tile.region, navmesh);
	_apply_usec_max = MAX(_apply_usec_max, uint32_t(clock.get_elapsed_microseconds()));
	update_tile_debug(tile, navmesh.ptr());

	if (!tile.changed_while_baking) {
		tile.dirty_for = -1.f;
	}
	tile.changed_while_baking = false;
}

#ifdef TOOLS_ENABLED

#if defined(ZN_GODOT)
PackedStringArray TiledNavigation3D::get_configuration_warnings() const {
	PackedStringArray warnings;
	get_configuration_warnings(warnings);
	return warnings;
}
#elif defined(ZN_GODOT_EXTENSION)
PackedStringArray TiledNavigation3D::_get_configuration_warnings() const {
	PackedStringArray warnings;
	get_configuration_warnings(warnings);
	return warnings;
}
#endif

void TiledNavigation3D::get_configuration_warnings(PackedStringArray &warnings) const {
	if (_template_navmesh.is_null()) {
		warnings.append("No template navigation mesh is assigned, so there are no settings to bake with.");
		return;
	}
	const real_t tile = get_tile_world_size();
	const real_t cells = tile / _template_navmesh->get_cell_size();
	if (Math::abs(cells - Math::round(cells)) > 0.001) {
		warnings.append(String("Tiles are {0} m across, which isn't a whole number of the template's cell_size ({1}), "
							   "so neighbouring tiles won't join up. Pick a cell_size that divides it.")
								.format(varray(tile, _template_navmesh->get_cell_size())));
	}
}

#endif

void TiledNavigation3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &TiledNavigation3D::set_enabled);
	ClassDB::bind_method(D_METHOD("is_enabled"), &TiledNavigation3D::is_enabled);

	ClassDB::bind_method(D_METHOD("set_run_in_editor", "enabled"), &TiledNavigation3D::set_run_in_editor);
	ClassDB::bind_method(D_METHOD("get_run_in_editor"), &TiledNavigation3D::get_run_in_editor);

	ClassDB::bind_method(
			D_METHOD("set_template_navigation_mesh", "navmesh"), &TiledNavigation3D::set_template_navigation_mesh
	);
	ClassDB::bind_method(D_METHOD("get_template_navigation_mesh"), &TiledNavigation3D::get_template_navigation_mesh);

	ClassDB::bind_method(D_METHOD("set_static_geometry", "geometry"), &TiledNavigation3D::set_static_geometry);
	ClassDB::bind_method(D_METHOD("get_static_geometry"), &TiledNavigation3D::get_static_geometry);

	ClassDB::bind_method(D_METHOD("set_source_geometry_root", "path"), &TiledNavigation3D::set_source_geometry_root);
	ClassDB::bind_method(D_METHOD("get_source_geometry_root"), &TiledNavigation3D::get_source_geometry_root);

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &TiledNavigation3D::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &TiledNavigation3D::get_tile_size);

	ClassDB::bind_method(D_METHOD("set_bake_delay", "seconds"), &TiledNavigation3D::set_bake_delay);
	ClassDB::bind_method(D_METHOD("get_bake_delay"), &TiledNavigation3D::get_bake_delay);

	ClassDB::bind_method(D_METHOD("set_max_bake_interval", "seconds"), &TiledNavigation3D::set_max_bake_interval);
	ClassDB::bind_method(D_METHOD("get_max_bake_interval"), &TiledNavigation3D::get_max_bake_interval);

	ClassDB::bind_method(D_METHOD("set_max_concurrent_bakes", "count"), &TiledNavigation3D::set_max_concurrent_bakes);
	ClassDB::bind_method(D_METHOD("get_max_concurrent_bakes"), &TiledNavigation3D::get_max_concurrent_bakes);

	ClassDB::bind_method(D_METHOD("set_navigation_layers", "layers"), &TiledNavigation3D::set_navigation_layers);
	ClassDB::bind_method(D_METHOD("get_navigation_layers"), &TiledNavigation3D::get_navigation_layers);

	ClassDB::bind_method(D_METHOD("set_configure_map", "enabled"), &TiledNavigation3D::set_configure_map);
	ClassDB::bind_method(D_METHOD("get_configure_map"), &TiledNavigation3D::get_configure_map);

	ClassDB::bind_method(D_METHOD("set_obstacle_group", "group"), &TiledNavigation3D::set_obstacle_group);
	ClassDB::bind_method(D_METHOD("get_obstacle_group"), &TiledNavigation3D::get_obstacle_group);

	ClassDB::bind_method(D_METHOD("set_navigation_map", "map"), &TiledNavigation3D::set_navigation_map);
	ClassDB::bind_method(D_METHOD("get_navigation_map"), &TiledNavigation3D::get_navigation_map);

	ClassDB::bind_method(
			D_METHOD("set_tile_geometry_callback", "callback"), &TiledNavigation3D::set_tile_geometry_callback
	);
	ClassDB::bind_method(D_METHOD("get_tile_geometry_callback"), &TiledNavigation3D::get_tile_geometry_callback);

	ClassDB::bind_method(
			D_METHOD("set_obstacle_settle_time", "seconds"), &TiledNavigation3D::set_obstacle_settle_time
	);
	ClassDB::bind_method(D_METHOD("get_obstacle_settle_time"), &TiledNavigation3D::get_obstacle_settle_time);

	ClassDB::bind_method(
			D_METHOD("set_use_edge_connections", "enabled"), &TiledNavigation3D::set_use_edge_connections
	);
	ClassDB::bind_method(D_METHOD("get_use_edge_connections"), &TiledNavigation3D::get_use_edge_connections);

	ClassDB::bind_method(D_METHOD("set_area", "area"), &TiledNavigation3D::set_area);
	ClassDB::bind_method(D_METHOD("get_area"), &TiledNavigation3D::get_area);

	ClassDB::bind_method(D_METHOD("bake_all"), &TiledNavigation3D::bake_all);
	ClassDB::bind_method(D_METHOD("get_pending_tile_count"), &TiledNavigation3D::get_pending_tile_count);
	ClassDB::bind_method(D_METHOD("get_statistics"), &TiledNavigation3D::get_statistics);

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
			PropertyInfo(Variant::NODE_PATH, "source_geometry_root"), "set_source_geometry_root", "get_source_geometry_root"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "tile_size", PROPERTY_HINT_RANGE, "1,128,0.5,or_greater,suffix:m"),
			"set_tile_size",
			"get_tile_size"
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
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "configure_map"), "set_configure_map", "get_configure_map");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "obstacle_group"), "set_obstacle_group", "get_obstacle_group");
	ADD_PROPERTY(PropertyInfo(Variant::AABB, "area"), "set_area", "get_area");
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "obstacle_settle_time", PROPERTY_HINT_RANGE, "0,5,0.05,or_greater,suffix:s"),
			"set_obstacle_settle_time",
			"get_obstacle_settle_time"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::BOOL, "use_edge_connections"), "set_use_edge_connections", "get_use_edge_connections"
	);
}

} // namespace zylann::voxel
