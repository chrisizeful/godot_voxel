#ifndef TILED_NAVIGATION_3D_H
#define TILED_NAVIGATION_3D_H

#include "../../util/containers/std_unordered_map.h"
#include "../../util/containers/std_vector.h"
#include "../../util/godot/classes/navigation_mesh.h"
#include "../../util/godot/classes/navigation_mesh_source_geometry_data_3d.h"
#include "../../util/godot/classes/node_3d.h"
#include "../../util/godot/macros.h"
#include "../../util/math/vector2i.h"

ZN_GODOT_FORWARD_DECLARE(class NavigationRegion3D)

namespace zylann::voxel {

// A prism carved out of a tile's navmesh, as NavigationMeshSourceGeometryData3D's projected obstructions are.
struct TileObstruction {
	PackedVector3Array vertices;
	float elevation = 0.f;
	float height = 0.f;
	bool carve = false;
};

// One tile's bake: assembled on the main thread, run on a worker, applied back on the main thread. Subclasses add
// triangles of their own and say what buries a floor.
class TileBakeTask {
public:
	virtual ~TileBakeTask() {}

	// Inputs, filled on the main thread.
	uint64_t key = 0;
	uint32_t generation = 0;
	Ref<NavigationMesh> navmesh;
	AABB baking_bounds;
	PackedFloat32Array static_vertices;
	PackedInt32Array static_indices;
	StdVector<int32_t> static_triangles;
	StdVector<TileObstruction> obstructions;
	// Geometry the game added for this tile alone (tile_geometry_callback), treated like the fixed geometry.
	PackedFloat32Array extra_vertices;
	PackedInt32Array extra_indices;
	real_t walkable_cos = 0.f;

	// Outputs.
	uint32_t bake_usec = 0;

	static void run_static(void *self);
	void run();

protected:
	// Adds triangles only this kind of tile has (a terrain's), in world space and in the baker's winding.
	virtual void add_own_triangles(PackedFloat32Array &vertices, PackedInt32Array &indices) {}
	// Gets ready to test the walkable static triangles inside [param walkable_box] for burial; false when nothing can
	// bury them here, which skips the test.
	virtual bool prepare_burial(const AABB &walkable_box) {
		return false;
	}
	// Whether the floor at [param floor_point] lies buried, so it isn't walkable.
	virtual bool is_buried(const Vector3 &floor_point) const {
		return false;
	}

private:
	bool is_walkable(const Vector3 &a, const Vector3 &b, const Vector3 &c) const;
	// The fixed geometry touching this tile and the extra geometry, as triangle corners, without what's buried.
	void add_fixed_triangles(PackedFloat32Array &vertices, PackedInt32Array &indices);
};

// Keeps a navigation mesh in step with a level at runtime, as square tiles of navigation region rebaked on worker
// threads when what's in them changes: fixed geometry (static_geometry, or parsed once from source_geometry_root) with
// NavigationObstacle3D nodes in obstacle_group carved where they stand, rebaked where they arrive and leave. Subclasses
// add geometry that changes by itself (VoxelTerrainNavigation: a voxel terrain's).
class TiledNavigation3D : public Node3D {
	GDCLASS(TiledNavigation3D, Node3D)
public:
	TiledNavigation3D();
	~TiledNavigation3D();

	void set_enabled(bool p_enabled);
	bool is_enabled() const;

	void set_run_in_editor(bool enable);
	bool get_run_in_editor() const;

	void set_template_navigation_mesh(Ref<NavigationMesh> navmesh);
	Ref<NavigationMesh> get_template_navigation_mesh() const;

	void set_static_geometry(Ref<NavigationMeshSourceGeometryData3D> geometry);
	Ref<NavigationMeshSourceGeometryData3D> get_static_geometry() const;

	void set_source_geometry_root(NodePath path);
	NodePath get_source_geometry_root() const;

	void set_tile_size(real_t size);
	real_t get_tile_size() const;

	void set_bake_delay(float seconds);
	float get_bake_delay() const;

	void set_max_bake_interval(float seconds);
	float get_max_bake_interval() const;

	void set_max_concurrent_bakes(int count);
	int get_max_concurrent_bakes() const;

	void set_navigation_layers(uint32_t layers);
	uint32_t get_navigation_layers() const;

	void set_configure_map(bool enable);
	bool get_configure_map() const;

	void set_obstacle_group(StringName group);
	StringName get_obstacle_group() const;

	void set_navigation_map(RID map);
	RID get_navigation_map() const;

	void set_tile_geometry_callback(Callable callback);
	Callable get_tile_geometry_callback() const;

	void set_area(AABB area);
	AABB get_area() const;

	void set_obstacle_settle_time(float seconds);
	float get_obstacle_settle_time() const;

	void set_use_edge_connections(bool enable);
	bool get_use_edge_connections() const;

	// Marks every tile for rebaking now, without waiting for bake_delay.
	void bake_all();
	// Tiles waiting to bake or baking. Zero once the navmesh matches the level.
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

	// Hooks for subclasses.
	// Tile width in world units, and where the tile grid starts.
	virtual real_t get_tile_world_size() const;
	virtual Vector3 get_grid_origin() const;
	// Grows [param extent] by what the subclass bakes besides the fixed geometry; [param has] says whether it holds one.
	virtual void merge_extent(AABB &extent, bool &has) const {}
	// Called once a frame before rebuilding (when due) and scheduling: notice changes, mark_dirty_in_aabb() them.
	virtual void update_before_bakes() {}
	// The tiles were laid out afresh; everything the subclass bakes counts as changed.
	virtual void on_tiles_laid_out() {}
	// A task for a tile, with the subclass's own inputs filled in.
	virtual TileBakeTask *create_bake_task(const AABB &baking_bounds);
	virtual void add_statistics(Dictionary &d) const {}

	bool is_running() const;
	bool is_layout_due() const {
		return _layout_due;
	}
	void mark_dirty_in_aabb(const AABB &world_box);
	void request_layout();
	void update_processing_state();

private:
	struct Tile {
		NavigationRegion3D *region = nullptr;
		// Indices of triangles (into the fixed geometry's indices, divided by 3) and projected obstructions touching it.
		StdVector<int32_t> static_triangles;
		StdVector<int32_t> static_obstructions;
		// Seconds since it first changed, and since it last changed; negative while it matches the level.
		float dirty_for = -1.f;
		float quiet_for = 0.f;
		bool baking = false;
		// Changed again while baking, so the result is already stale.
		bool changed_while_baking = false;
		uint32_t generation = 0;
	};

	// Obstacles carved as of the last check, by instance id: the world-space prism baked and its bounds.
	struct Obstacle {
		TileObstruction obstruction;
		AABB bounds;
		// How long it has stayed put; it carves only once that reaches obstacle_settle_time.
		float still_for = 0.f;
		bool carving = false;
	};

	struct RunningBake {
		int64_t task_id;
		TileBakeTask *task;
	};

	void process(float delta_time);
	void reset_tiles();
	void lay_out_tiles();
	void parse_source_geometry();
	Ref<NavigationMeshSourceGeometryData3D> get_fixed_geometry() const;
	void update_obstacles();
	Tile &get_or_create_tile(Vector2i tile_pos);
	void mark_dirty(Tile &tile);
	AABB get_tile_bounds(Vector2i tile_pos) const;
	float get_border_size() const;
	Vector2i world_to_tile(const Vector3 &pos) const;
	void start_bake(uint64_t key, Tile &tile);
	void collect_bakes(bool wait);
	void on_tile_baked(uint64_t key, uint32_t generation, Ref<NavigationMesh> navmesh, uint32_t bake_usec);

	static void _bind_methods();

	Ref<NavigationMesh> _template_navmesh;
	Ref<NavigationMeshSourceGeometryData3D> _static_geometry;
	NodePath _source_geometry_root;
	Ref<NavigationMeshSourceGeometryData3D> _parsed_geometry;
	StdUnorderedMap<uint64_t, Tile> _tiles;
	StdUnorderedMap<uint64_t, Obstacle> _obstacles;
	StdVector<RunningBake> _running_bakes;
	StringName _obstacle_group = "navigation_obstacles";
	RID _navigation_map;
	Callable _tile_geometry_callback;
	// Covered by tiles even where no fixed geometry reaches (ground the callback adds).
	AABB _area;
	float _obstacle_settle_time = 1.f;
	bool _use_edge_connections = true;
	real_t _tile_size = 16.0;
	// Vertical extent of the tiles, in world space.
	real_t _min_y = 0.f;
	real_t _max_y = 0.f;
	float _bake_delay = 0.5f;
	float _max_bake_interval = 2.f;
	float _obstacle_check_in = 0.f;
	int _max_concurrent_bakes = 4;
	int _bakes_in_flight = 0;
	uint32_t _navigation_layers = 1;
	// Numbers every bake, so a result for a tile reset since is recognised and dropped.
	uint32_t _bake_generation = 0;
	bool _enabled = true;
	bool _run_in_editor = false;
	bool _configure_map = true;
	bool _layout_due = true;

	// Statistics
	uint64_t _bake_count = 0;
	uint64_t _bake_usec_total = 0;
	uint32_t _bake_usec_max = 0;
	uint32_t _bake_usec_last = 0;
	uint32_t _apply_usec_max = 0;
	uint32_t _dropped_results = 0;
	uint32_t _layout_count = 0;
	int64_t _fixed_triangle_count = 0;
};

} // namespace zylann::voxel

#endif // TILED_NAVIGATION_3D_H
