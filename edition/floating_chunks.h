#ifndef VOXEL_FLOATING_CHUNKS_H
#define VOXEL_FLOATING_CHUNKS_H

#include "../meshers/voxel_mesher.h"
#include "../util/godot/core/array.h"
#include "../util/godot/core/transform_3d.h"
#include "../util/math/box3i.h"
#include "../util/containers/std_vector.h"

ZN_GODOT_FORWARD_DECLARE(class Node);

namespace zylann::voxel {

class VoxelTool;

Array separate_floating_chunks(
		VoxelTool &voxel_tool,
		Box3i world_box,
		Node *parent_node,
		Transform3D terrain_transform,
		Ref<VoxelMesher> mesher,
		Array materials
);

// Finds groups of connected solid voxels touching `seed_box` that are not connected to the rest of the volume.
// Each group is flood-filled from its seed; it counts as anchored (and is skipped) if it reaches an area the tool can't
// edit (unloaded or out of bounds), or grows past `max_island_voxels`. Unlike `separate_floating_chunks`, islands are
// not limited to a box, only by size.
// Solid means negative SDF when the tool's channel is SDF, otherwise any value other than the tool's eraser value.
StdVector<StdVector<Vector3i>> find_floating_islands(
		const VoxelTool &voxel_tool,
		const Box3i seed_box,
		const uint32_t max_island_voxels
);

} // namespace zylann::voxel

#endif // VOXEL_FLOATING_CHUNKS_H
