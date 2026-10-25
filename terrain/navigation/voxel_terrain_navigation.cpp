#include "voxel_terrain_navigation.h"
#include "../../storage/voxel_buffer.h"
#include "../../storage/voxel_data.h"
#include "../../util/godot/classes/object.h"
#include "../../util/math/conv.h"
#include "../../util/profiling.h"

namespace zylann::voxel {

namespace {

// A tile's bake with the terrain's triangles added, and walkable fixed geometry tested against the voxels' SDF: buried
// where solid voxels stand half an agent's height above it.
class VoxelTileBakeTask : public TileBakeTask {
public:
	struct VoxelSurface {
		PackedVector3Array vertices;
		PackedInt32Array indices;
		// Block origin in terrain-local voxel space.
		Vector3 origin;
	};

	Transform3D terrain_transform;
	StdVector<VoxelSurface> voxel_surfaces;
	std::shared_ptr<VoxelData> voxel_data;
	bool drop_buried = true;
	real_t buried_probe_height = 1.f;

protected:
	void add_own_triangles(PackedFloat32Array &vertices, PackedInt32Array &indices) override {
		ZN_PROFILE_SCOPE();
		int64_t vertex_count = 0;
		int64_t index_count = 0;
		for (const VoxelSurface &s : voxel_surfaces) {
			vertex_count += s.vertices.size();
			index_count += s.indices.size();
		}
		const int64_t vertex_start = vertices.size() / 3;
		const int64_t index_start = indices.size();
		vertices.resize(vertices.size() + vertex_count * 3);
		indices.resize(index_start + index_count);
		float *vw = vertices.ptrw();
		int32_t *iw = indices.ptrw();
		int64_t vbase = vertex_start;
		int64_t ii = index_start;
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
				iw[ii++] = int32_t(a);
				iw[ii++] = int32_t(b);
				iw[ii++] = int32_t(c);
			}
			vbase += s.vertices.size();
		}
		indices.resize(ii);
	}

	// Copies the SDF around the walkable fixed geometry once, so testing burial reads no shared state per sample.
	bool prepare_burial(const AABB &walkable_box) override {
		if (!drop_buried || voxel_data == nullptr) {
			return false;
		}
		AABB box = walkable_box;
		box.size.y += buried_probe_height;
		_world_to_voxel = terrain_transform.affine_inverse();
		const AABB local = _world_to_voxel.xform(box);
		const Vector3i min_pos = math::floor_to_int(local.position) - Vector3i(1, 1, 1);
		const Vector3i max_pos = math::floor_to_int(local.position + local.size) + Vector3i(2, 2, 2);
		const Box3i clipped = Box3i::from_min_max(min_pos, max_pos).clipped(voxel_data->get_bounds());
		if (clipped.size.x < 2 || clipped.size.y < 2 || clipped.size.z < 2) {
			return false;
		}
		_sdf_origin = clipped.position;
		_sdf.create(clipped.size);
		voxel_data->copy(clipped.position, _sdf, 1 << VoxelBuffer::CHANNEL_SDF, false);
		return true;
	}

	bool is_buried(const Vector3 &floor_point) const override {
		return sample_sdf(floor_point + Vector3(0, buried_probe_height, 0)) < 0.f;
	}

private:
	// The copied SDF at a world point, trilinearly, in voxels; air outside the copy.
	float sample_sdf(const Vector3 &world_pos) const {
		const Vector3 p = _world_to_voxel.xform(world_pos) - to_vec3(_sdf_origin);
		const Vector3i size = _sdf.get_size();
		const Vector3i b = math::floor_to_int(p);
		if (b.x < 0 || b.y < 0 || b.z < 0 || b.x + 1 >= size.x || b.y + 1 >= size.y || b.z + 1 >= size.z) {
			return 1.f;
		}
		const Vector3 f = p - to_vec3(b);
		const unsigned int ch = VoxelBuffer::CHANNEL_SDF;
		const float x00 = Math::lerp(_sdf.get_voxel_f(b.x, b.y, b.z, ch), _sdf.get_voxel_f(b.x + 1, b.y, b.z, ch), float(f.x));
		const float x10 =
				Math::lerp(_sdf.get_voxel_f(b.x, b.y + 1, b.z, ch), _sdf.get_voxel_f(b.x + 1, b.y + 1, b.z, ch), float(f.x));
		const float x01 =
				Math::lerp(_sdf.get_voxel_f(b.x, b.y, b.z + 1, ch), _sdf.get_voxel_f(b.x + 1, b.y, b.z + 1, ch), float(f.x));
		const float x11 = Math::lerp(
				_sdf.get_voxel_f(b.x, b.y + 1, b.z + 1, ch), _sdf.get_voxel_f(b.x + 1, b.y + 1, b.z + 1, ch), float(f.x)
		);
		return Math::lerp(Math::lerp(x00, x10, float(f.y)), Math::lerp(x01, x11, float(f.y)), float(f.z));
	}

	VoxelBuffer _sdf = VoxelBuffer(VoxelBuffer::ALLOCATOR_POOL);
	Vector3i _sdf_origin;
	Transform3D _world_to_voxel;
};

} // namespace

void VoxelTerrainNavigation::set_tile_size_blocks(int size) {
	size = MAX(size, 1);
	if (size == _tile_size_blocks) {
		return;
	}
	_tile_size_blocks = size;
	request_layout();
#ifdef TOOLS_ENABLED
	update_configuration_warnings();
#endif
}

int VoxelTerrainNavigation::get_tile_size_blocks() const {
	return _tile_size_blocks;
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

VoxelTerrain *VoxelTerrainNavigation::get_terrain() const {
	return Object::cast_to<VoxelTerrain>(get_parent());
}

void VoxelTerrainNavigation::_notification(int p_what) {
	if (p_what == NOTIFICATION_EXIT_TREE) {
		detach();
	}
}

// With a terrain, tiles are mesh blocks wide; tile_size is the base class's, for levels without one.
void VoxelTerrainNavigation::_validate_property(PropertyInfo &p_property) const {
	if (p_property.name == StringName("tile_size")) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void VoxelTerrainNavigation::attach(VoxelTerrain &terrain) {
	detach();
	_terrain_id = terrain.get_instance_id();
	terrain.set_mesh_change_recording(true);
	request_layout();
}

void VoxelTerrainNavigation::detach() {
	VoxelTerrain *terrain = Object::cast_to<VoxelTerrain>(ObjectDB::get_instance(_terrain_id));
	if (terrain != nullptr) {
		terrain->set_mesh_change_recording(false);
	}
	_terrain_id = ObjectID();
}

real_t VoxelTerrainNavigation::get_tile_world_size() const {
	const VoxelTerrain *terrain = get_terrain();
	if (terrain == nullptr) {
		return TiledNavigation3D::get_tile_world_size();
	}
	return real_t(_tile_size_blocks * terrain->get_mesh_block_size()) * _terrain_transform.basis.get_scale().x;
}

Vector3 VoxelTerrainNavigation::get_grid_origin() const {
	return _terrain_transform.origin;
}

void VoxelTerrainNavigation::merge_extent(AABB &extent, bool &has) const {
	const VoxelTerrain *terrain = get_terrain();
	if (terrain == nullptr) {
		return;
	}
	const Box3i bounds = terrain->get_bounds();
	const AABB terrain_box = _terrain_transform.xform(AABB(to_vec3(bounds.position), to_vec3(bounds.size)));
	extent = has ? extent.merge(terrain_box) : terrain_box;
	has = true;
}

void VoxelTerrainNavigation::update_before_bakes() {
	VoxelTerrain *terrain = get_terrain();
	if (terrain == nullptr) {
		if (_terrain_id.is_valid()) {
			detach();
			request_layout();
		}
		return;
	}
	if (ObjectID(terrain->get_instance_id()) != _terrain_id) {
		attach(*terrain);
	}
	if (!terrain->get_global_transform().is_equal_approx(_terrain_transform)) {
		_terrain_transform = terrain->get_global_transform();
		request_layout();
	}
	_changes.clear();
	terrain->consume_mesh_changes(_changes);
	// A fresh layout counts everything meshed as changed anyway.
	if (is_layout_due()) {
		return;
	}
	// Only the tiles whose baking area the changed voxels reach: an edit inside one tile leaves its neighbours alone.
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
}

void VoxelTerrainNavigation::on_tiles_laid_out() {
	VoxelTerrain *terrain = get_terrain();
	if (terrain == nullptr) {
		return;
	}
	const int block_size = terrain->get_mesh_block_size();
	terrain->get_mesh_map().for_each_block([this, block_size](const VoxelMeshBlockVT &block) {
		const AABB local(to_vec3(block.position * block_size), Vector3(block_size, block_size, block_size));
		mark_dirty_in_aabb(_terrain_transform.xform(local));
	});
}

TileBakeTask *VoxelTerrainNavigation::create_bake_task(const AABB &baking_bounds) {
	VoxelTileBakeTask *task = ZN_NEW(VoxelTileBakeTask);
	VoxelTerrain *terrain = get_terrain();
	if (terrain == nullptr) {
		return task;
	}
	task->terrain_transform = _terrain_transform;
	task->voxel_data = terrain->get_storage_shared();
	task->drop_buried = _drop_buried_static_geometry;
	task->buried_probe_height = get_template_navigation_mesh()->get_agent_height() * 0.5f;

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
						task->voxel_surfaces.push_back(VoxelTileBakeTask::VoxelSurface{ s.vertices, s.indices, origin });
					}
				}
			}
		}
	}
	return task;
}

void VoxelTerrainNavigation::add_statistics(Dictionary &d) const {
	d["edit_changes"] = int64_t(_edit_changes);
	d["block_changes"] = int64_t(_block_changes);
}

#ifdef TOOLS_ENABLED

void VoxelTerrainNavigation::get_configuration_warnings(PackedStringArray &warnings) const {
	TiledNavigation3D::get_configuration_warnings(warnings);
	if (get_terrain() == nullptr) {
		warnings.append(String("{0} must be a child of a VoxelTerrain; without one, use {1}.")
								.format(varray(get_class_static(), TiledNavigation3D::get_class_static())));
	}
}

#endif

void VoxelTerrainNavigation::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_tile_size_blocks", "size"), &VoxelTerrainNavigation::set_tile_size_blocks);
	ClassDB::bind_method(D_METHOD("get_tile_size_blocks"), &VoxelTerrainNavigation::get_tile_size_blocks);

	ClassDB::bind_method(
			D_METHOD("set_drop_buried_static_geometry", "enabled"),
			&VoxelTerrainNavigation::set_drop_buried_static_geometry
	);
	ClassDB::bind_method(
			D_METHOD("get_drop_buried_static_geometry"), &VoxelTerrainNavigation::get_drop_buried_static_geometry
	);

	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "tile_size_blocks", PROPERTY_HINT_RANGE, "1,8,1"),
			"set_tile_size_blocks",
			"get_tile_size_blocks"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::BOOL, "drop_buried_static_geometry"),
			"set_drop_buried_static_geometry",
			"get_drop_buried_static_geometry"
	);
}

} // namespace zylann::voxel
