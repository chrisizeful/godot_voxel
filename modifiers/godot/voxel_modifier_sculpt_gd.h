#ifndef VOXEL_MODIFIER_SCULPT_GD_H
#define VOXEL_MODIFIER_SCULPT_GD_H

#include "../voxel_sculpt_data.h"
#include "voxel_modifier_gd.h"

namespace zylann::voxel::godot {

// Hand-sculpted changes over what lies below it in the volume (the generator and the modifiers before it), stored in
// a VoxelSculptData; an editor brush, or a script, sculpts it with sculpt_sphere(). It ignores operation and
// smoothness, and its own transform: the data is in the volume's voxel grid.
class VoxelModifierSculpt : public VoxelModifier {
	GDCLASS(VoxelModifierSculpt, VoxelModifier);

public:
	enum BrushMode {
		BRUSH_ADD,
		BRUSH_REMOVE,
		BRUSH_SMOOTH,
		BRUSH_PAINT,
		BRUSH_ERASE,
		BRUSH_MODE_COUNT
	};

	Ref<VoxelSculptData> get_data() const;
	void set_data(Ref<VoxelSculptData> data);

	// Sculpts a sphere at `center` (in the volume's local voxel space) of `radius` voxels: ADD raises a sphere,
	// REMOVE digs one, SMOOTH blurs the surface, PAINT sets `material` inside it (-1 clears it), ERASE takes the
	// sculpt back towards what lies below. `strength` blends each step, 0 to 1.
	void sculpt_sphere(Vector3 center, float radius, BrushMode mode, float strength, int material);

protected:
	zylann::voxel::VoxelModifier *create(zylann::voxel::VoxelModifierStack &modifiers, uint32_t id) override;

private:
	void on_data_changed();
	static void _bind_methods();

	Ref<VoxelSculptData> _data;
};

} // namespace zylann::voxel::godot

VARIANT_ENUM_CAST(zylann::voxel::godot::VoxelModifierSculpt::BrushMode);

#endif // VOXEL_MODIFIER_SCULPT_GD_H
