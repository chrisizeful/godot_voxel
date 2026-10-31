#ifndef VOXEL_MODIFIER_PATH_GD_H
#define VOXEL_MODIFIER_PATH_GD_H

#include "../../util/godot/core/packed_arrays.h"
#include "voxel_modifier_gd.h"

namespace zylann::voxel::godot {

// A rounded box swept along a polyline (a road's bed, a tunnel, a trench).
class VoxelModifierPath : public VoxelModifier {
	GDCLASS(VoxelModifierPath, VoxelModifier);

public:
	PackedVector3Array get_points() const;
	void set_points(PackedVector3Array points);

	float get_width() const;
	void set_width(float width);

	float get_bottom() const;
	void set_bottom(float bottom);

	float get_top() const;
	void set_top(float top);

	float get_rounding() const;
	void set_rounding(float rounding);

protected:
	zylann::voxel::VoxelModifier *create(zylann::voxel::VoxelModifierStack &modifiers, uint32_t id) override;

private:
	void update_profile();
	static void _bind_methods();

	PackedVector3Array _points;
	float _width = 4.f;
	float _bottom = -1.f;
	float _top = 0.f;
	float _rounding = 0.25f;
};

} // namespace zylann::voxel::godot

#endif // VOXEL_MODIFIER_PATH_GD_H
