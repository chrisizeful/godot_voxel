#ifndef VOXEL_TERRAIN_NAVIGATION_H
#define VOXEL_TERRAIN_NAVIGATION_H

#include "../fixed_lod/voxel_terrain.h"
#include "tiled_navigation_3d.h"

namespace zylann::voxel {

// A TiledNavigation3D that also bakes the VoxelTerrain it's a child of: tiles are columns of mesh blocks, each rebaked
// when the voxels reaching it change (edited, or streamed in or out), with the terrain's own triangles. Walkable fixed
// geometry the voxels now bury is left out, so filling a road in takes it off the navmesh and digging it out puts it back.
class VoxelTerrainNavigation : public TiledNavigation3D {
	GDCLASS(VoxelTerrainNavigation, TiledNavigation3D)
public:
	void set_tile_size_blocks(int size);
	int get_tile_size_blocks() const;

	void set_drop_buried_static_geometry(bool enable);
	bool get_drop_buried_static_geometry() const;

#ifdef TOOLS_ENABLED
	void get_configuration_warnings(PackedStringArray &warnings) const override;
#endif

protected:
	void _notification(int p_what);
	void _validate_property(PropertyInfo &p_property) const;

	real_t get_tile_world_size() const override;
	Vector3 get_grid_origin() const override;
	void merge_extent(AABB &extent, bool &has) const override;
	void update_before_bakes() override;
	void on_tiles_laid_out() override;
	TileBakeTask *create_bake_task(const AABB &baking_bounds) override;
	void add_statistics(Dictionary &d) const override;

private:
	VoxelTerrain *get_terrain() const;
	void attach(VoxelTerrain &terrain);
	void detach();

	static void _bind_methods();

	ObjectID _terrain_id;
	// Terrain placement the tiles were laid out for.
	Transform3D _terrain_transform;
	int _tile_size_blocks = 1;
	bool _drop_buried_static_geometry = true;
	StdVector<VoxelTerrain::MeshChange> _changes;
	uint64_t _edit_changes = 0;
	uint64_t _block_changes = 0;
};

} // namespace zylann::voxel

#endif // VOXEL_TERRAIN_NAVIGATION_H
