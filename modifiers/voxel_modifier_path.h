#ifndef VOXEL_MODIFIER_PATH_H
#define VOXEL_MODIFIER_PATH_H

#include "../util/containers/std_vector.h"
#include "voxel_modifier_sdf.h"

namespace zylann::voxel {

// A rounded box swept along a polyline: its cross-section spans `width` across and from `bottom` to `top` along the
// path's up (Y, kept level), with its edges rounded by `rounding`. Points are in the modifier's local space.
class VoxelModifierPath : public VoxelModifierSdf {
public:
	Type get_type() const override {
		return TYPE_PATH;
	};

	void set_points(Span<const Vector3f> points);
	void set_profile(float width, float bottom, float top, float rounding);

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
	struct Segment {
		Vector3f origin; // Middle of the cross-section's centre at the segment's start
		Vector3f along;
		Vector3f up;
		Vector3f side;
		float start; // Along `along`, from `origin`: where the box starts and ends
		float end;
		Vector3f aabb_min; // Local bounds, grown by the profile
		Vector3f aabb_max;
	};

	void update_segments();

	StdVector<Vector3f> _points;
	StdVector<Segment> _segments;
	float _width = 4.f;
	float _bottom = -1.f;
	float _top = 0.f;
	float _rounding = 0.25f;
};

} // namespace zylann::voxel

#endif // VOXEL_MODIFIER_PATH_H
