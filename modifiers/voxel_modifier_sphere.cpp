#include "voxel_modifier_sphere.h"
#include "../engine/voxel_engine.h"
#include "../util/godot/core/packed_arrays.h"
#include "../util/math/conv.h"
#include "../util/math/sdf.h"
#include "../util/memory/memory.h"
#include "../util/profiling.h"

namespace zylann::voxel {

void VoxelModifierSphere::set_radius(float radius) {
	RWLockWrite wlock(_rwlock);
	if (radius == _radius) {
		return;
	}
	_radius = radius;
#ifdef VOXEL_ENABLE_GPU
	_shader_data_need_update = true;
#endif
	update_aabb();
}

float VoxelModifierSphere::get_radius() const {
	return _radius;
}

void VoxelModifierSphere::update_aabb() {
	const float extent = _radius * 1.25f + get_smoothness();
	const float extent2 = 2.0 * extent;
	_aabb = AABB(get_transform().origin - Vector3(extent, extent, extent), Vector3(extent2, extent2, extent2));
}

void VoxelModifierSphere::apply(VoxelModifierContext ctx) const {
	ZN_PROFILE_SCOPE();
	RWLockRead rlock(_rwlock);
	const Vector3f center = to_vec3f(get_transform().origin);
	const float radius = _radius;
	// TODO Support transform scale
	apply_shape(ctx, [center, radius](const Vector3f pos) { return math::sdf_sphere(pos, center, radius); });
}

#ifdef VOXEL_ENABLE_GPU

void VoxelModifierSphere::get_shader_data(ShaderData &out_shader_data) {
	struct SphereParams {
		float radius;
	};

	RWLockWrite wlock(_rwlock);

	if (_shader_data_need_update || _shader_data == nullptr) {
		update_base_shader_data_no_lock();

		SphereParams sphere_params;
		sphere_params.radius = _radius;
		PackedByteArray pba;
		zylann::godot::copy_bytes_to(pba, sphere_params);

		if (_shader_data->params.size() < 2) {
			std::shared_ptr<ComputeShaderResource> res = ComputeShaderResourceFactory::create_storage_buffer(pba);
			_shader_data->params.push_back(ComputeShaderParameter{ 5, res });

		} else if (_shader_data_need_update) {
			ZN_ASSERT(_shader_data->params.size() == 2);
			ComputeShaderResource::update_storage_buffer(_shader_data->params[1].resource, pba);
		}

		_shader_data_need_update = false;
	}

	out_shader_data.modifier_type = get_type();
	out_shader_data.params = _shader_data;
}

#endif

} // namespace zylann::voxel
