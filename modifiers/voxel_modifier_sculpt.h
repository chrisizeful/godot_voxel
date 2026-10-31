#ifndef VOXEL_MODIFIER_SCULPT_H
#define VOXEL_MODIFIER_SCULPT_H

#include "voxel_modifier.h"
#include "voxel_sculpt_data.h"

namespace zylann::voxel {

// Adds a VoxelSculptData's distances to what lies below it, and sets its materials. The data is in the volume's own
// voxel grid, so the modifier's transform doesn't move it.
class VoxelModifierSculpt : public VoxelModifier {
public:
	Type get_type() const override {
		return TYPE_SCULPT;
	};

	bool is_sdf() const override {
		return false;
	}

	bool writes_material() const override;

	void set_data(Ref<VoxelSculptData> data);
	// Call after the data's chunks changed, so the bounds follow.
	void update_data_bounds();

	void apply(VoxelModifierContext ctx) const override;

#ifdef VOXEL_ENABLE_GPU
	bool supports_gpu() const override {
		return false;
	}
	void get_shader_data(ShaderData &out_shader_data) override {}
#endif

protected:
	void update_aabb() override;

private:
	Ref<VoxelSculptData> _data;
};

} // namespace zylann::voxel

#endif // VOXEL_MODIFIER_SCULPT_H
