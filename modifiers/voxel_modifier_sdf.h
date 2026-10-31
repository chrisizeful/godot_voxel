#ifndef VOXEL_MODIFIER_SDF_H
#define VOXEL_MODIFIER_SDF_H

#include "../util/math/sdf.h"
#include "voxel_modifier.h"

namespace zylann::voxel {

class VoxelModifierSdf : public VoxelModifier {
public:
	enum Operation { //
		OP_ADD = 0,
		OP_SUBTRACT = 1,
		// Leaves the distance as it is, and only writes the material inside the shape.
		OP_PAINT = 2
	};

	inline Operation get_operation() const {
		return _operation;
	}

	void set_operation(Operation op);

	inline float get_smoothness() const {
		return _smoothness;
	}

	void set_smoothness(float p_smoothness);

	bool is_sdf() const override {
		return true;
	}

	// Material index written inside the shape when adding or painting, or -1 to leave the material as it is.
	inline int get_material() const {
		return _material;
	}
	void set_material(int material);

	bool writes_material() const override {
		return _material >= 0 && _operation != OP_SUBTRACT;
	}

#ifdef VOXEL_ENABLE_GPU
	bool supports_gpu() const override {
		return _operation != OP_PAINT;
	}
#endif

protected:
#ifdef VOXEL_ENABLE_GPU
	void update_base_shader_data_no_lock();
#endif

	// Applies the operation and material with the signed distance of the shape, given by `shape(position)`.
	// The caller holds the read lock.
	template <typename F>
	void apply_shape(VoxelModifierContext ctx, F shape) const {
		const float smoothness = _smoothness;
		const bool paint = writes_material() && ctx.material.size() == ctx.sdf.size();
		for (unsigned int i = 0; i < ctx.sdf.size(); ++i) {
			const float sd = shape(ctx.positions[i]);
			switch (_operation) {
				case OP_ADD:
					ctx.sdf[i] = smoothness > 0.f ? math::sdf_smooth_union(ctx.sdf[i], sd, smoothness)
												  : math::min(ctx.sdf[i], sd);
					break;
				case OP_SUBTRACT:
					ctx.sdf[i] = smoothness > 0.f ? math::sdf_smooth_subtract(ctx.sdf[i], sd, smoothness)
												  : math::max(ctx.sdf[i], -sd);
					break;
				case OP_PAINT:
					break;
			}
			if (paint && sd < 0.f) {
				ctx.material[i] = static_cast<uint8_t>(_material);
			}
		}
	}

private:
	Operation _operation = OP_ADD;
	float _smoothness = 0.f;
	int _material = -1;
	// float _margin = 0.f;
};

} // namespace zylann::voxel

#endif // VOXEL_MODIFIER_SDF_H
