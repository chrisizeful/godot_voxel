#ifndef VOXEL_TERRAIN_NAVIGATION_H
#define VOXEL_TERRAIN_NAVIGATION_H

#include "../../util/containers/std_unordered_map.h"
#include "../../util/containers/std_vector.h"
#include "../../util/godot/classes/navigation_mesh.h"
#include "../../util/godot/classes/navigation_mesh_source_geometry_data_3d.h"
#include "../../util/godot/classes/node_3d.h"
#include "../../util/godot/macros.h"
#include "../../util/math/vector2i.h"
#include "../fixed_lod/voxel_terrain.h"

ZN_GODOT_FORWARD_DECLARE(class NavigationRegion3D)

namespace zylann::voxel {

class BakeTileTask;

// Keeps a navigation mesh in step with a VoxelTerrain at runtime, as square tiles (columns of mesh blocks) rebaked on
// worker threads when the voxels in them change. Each tile bakes the terrain's triangles together with fixed scene
// geometry given as `static_geometry` (buildings, roads, obstructions, baked offline), minus any walkable part of that
// geometry the voxels have since buried, so the navmesh matches what can actually be walked. Things that move but stand
// in the way (a creature dug into a wall) carve it too: NavigationObstacle3D nodes in `obstacle_group` with
// affect_navigation_mesh on, rebaked where they leave and arrive.
class VoxelTerrainNavigation : public Node3D {
	GDCLASS(VoxelTerrainNavigation, Node3D)
public:
	VoxelTerrainNavigation();
	~VoxelTerrainNavigation();

	void set_enabled(bool p_enabled);
	bool is_enabled() const;

	void set_run_in_editor(bool enable);
	bool get_run_in_editor() const;

	void set_template_navigation_mesh(Ref<NavigationMesh> navmesh);
	Ref<NavigationMesh> get_template_navigation_mesh() const;

	void set_static_geometry(Ref<NavigationMeshSourceGeometryData3D> geometry);
	Ref<NavigationMeshSourceGeometryData3D> get_static_geometry() const;

	void set_tile_size_blocks(int size);
	int get_tile_size_blocks() const;

	void set_bake_delay(float seconds);
	float get_bake_delay() const;

	void set_max_bake_interval(float seconds);
	float get_max_bake_interval() const;

	void set_max_concurrent_bakes(int count);
	int get_max_concurrent_bakes() const;

	void set_navigation_layers(uint32_t layers);
	uint32_t get_navigation_layers() const;

	void set_drop_buried_static_geometry(bool enable);
	bool get_drop_buried_static_geometry() const;

	void set_configure_map(bool enable);
	bool get_configure_map() const;

	void set_obstacle_group(StringName group);
	StringName get_obstacle_group() const;

	// Marks every tile for rebaking now, without waiting for bake_delay.
	void bake_all();
	// Tiles waiting to bake or baking. Zero once the navmesh matches the terrain.
	int get_pending_tile_count() const;
	Dictionary get_statistics() const;

#ifdef TOOLS_ENABLED
#if defined(ZN_GODOT)
	PackedStringArray get_configuration_warnings() const override;
#elif defined(ZN_GODOT_EXTENSION)
	PackedStringArray _get_configuration_warnings() const override;
#endif
	virtual void get_configuration_warnings(PackedStringArray &warnings) const;
#endif


protected:
	void _notification(int p_what);

private:
	struct Tile {
		NavigationRegion3D *region = nullptr;
		// Indices of triangles (into static_geometry's indices, divided by 3) and projected obstructions touching it.
		StdVector<int32_t> static_triangles;
		StdVector<int32_t> static_obstructions;
		// Seconds since it first changed, and since it last changed; negative while it matches the terrain.
		float dirty_for = -1.f;
		float quiet_for = 0.f;
		bool baking = false;
		// Changed again while baking, so the result is already stale.
		bool changed_while_baking = false;
		bool baked_once = false;
		uint32_t generation = 0;
	};

	void update_processing_state();
	void process(float delta_time);
	VoxelTerrain *get_terrain() const;
	bool is_running() const;
	void attach(VoxelTerrain &terrain);
	void detach();
	void reset_tiles();
	void rebuild_static_buckets();
	void mark_dirty_in_aabb(const AABB &world_box);
	Tile &get_or_create_tile(Vector2i tile_pos);
	void mark_dirty(Tile &tile);
	AABB get_tile_bounds(Vector2i tile_pos) const;
	float get_border_size() const;
	real_t get_tile_world_size() const;
	Vector2i world_to_tile(const Vector3 &pos) const;
	void start_bake(uint64_t key, Tile &tile);
	void update_obstacles();
	// Applies bakes that have finished on worker threads; with wait, first waits for every one still running.
	void collect_bakes(bool wait);
	void on_tile_baked(uint64_t key, uint32_t generation, Ref<NavigationMesh> navmesh, uint32_t bake_usec);

	static void _bind_methods();

	Ref<NavigationMesh> _template_navmesh;
	Ref<NavigationMeshSourceGeometryData3D> _static_geometry;
	StdUnorderedMap<uint64_t, Tile> _tiles;
	ObjectID _terrain_id;
	// Terrain placement the tiles were laid out for, and its vertical extent in world space.
	Transform3D _terrain_transform;
	real_t _min_y = 0.f;
	real_t _max_y = 0.f;
	int _tile_size_blocks = 1;
	float _bake_delay = 0.5f;
	float _max_bake_interval = 2.f;
	int _max_concurrent_bakes = 4;
	uint32_t _navigation_layers = 1;
	int _bakes_in_flight = 0;
	// Bakes running on Godot's WorkerThreadPool (low priority, so the voxel engine's own threads stay free for meshing).
	struct RunningBake {
		int64_t task_id;
		BakeTileTask *task;
	};
	StdVector<RunningBake> _running_bakes;
	// Numbers every bake, so a result for a tile reset since is recognised and dropped.
	uint32_t _bake_generation = 0;
	bool _enabled = true;
	bool _run_in_editor = false;
	bool _drop_buried_static_geometry = true;
	bool _configure_map = true;
	bool _static_dirty = true;
	StdVector<VoxelTerrain::MeshChange> _changes;
	StringName _obstacle_group = "navigation_obstacles";
	// Obstacles carved as of the last check, by instance id: the world-space prism baked and its bounds.
	struct Obstacle {
		PackedVector3Array vertices;
		float elevation = 0.f;
		float height = 0.f;
		bool carve = false;
		AABB bounds;
	};
	StdUnorderedMap<uint64_t, Obstacle> _obstacles;
	float _obstacle_check_in = 0.f;

	// Statistics
	uint64_t _bake_count = 0;
	uint64_t _bake_usec_total = 0;
	uint32_t _bake_usec_max = 0;
	uint32_t _bake_usec_last = 0;
	uint32_t _apply_usec_max = 0;
	uint32_t _dropped_results = 0;
	uint64_t _edit_changes = 0;
	uint64_t _block_changes = 0;
};

} // namespace zylann::voxel

#endif // VOXEL_TERRAIN_NAVIGATION_H
