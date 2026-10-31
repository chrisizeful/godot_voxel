#include "voxel_modifier_sculpt.h"
#include "../util/math/conv.h"
#include "../util/profiling.h"

namespace zylann::voxel {

bool VoxelModifierSculpt::writes_material() const {
	RWLockRead rlock(_rwlock);
	return _data.is_valid() && _data->has_material();
}

void VoxelModifierSculpt::set_data(Ref<VoxelSculptData> data) {
	RWLockWrite wlock(_rwlock);
	_data = data;
	update_aabb();
}

void VoxelModifierSculpt::update_data_bounds() {
	RWLockWrite wlock(_rwlock);
	update_aabb();
}

void VoxelModifierSculpt::update_aabb() {
	if (_data.is_null()) {
		_aabb = AABB();
		return;
	}
	Box3i bounds;
	{
		RWLockRead dlock(_data->get_lock());
		bounds = _data->get_bounds();
	}
	_aabb = AABB(to_vec3(bounds.position), to_vec3(bounds.size));
}

void VoxelModifierSculpt::apply(VoxelModifierContext ctx) const {
	ZN_PROFILE_SCOPE();
	RWLockRead rlock(_rwlock);
	if (_data.is_null()) {
		return;
	}
	const VoxelSculptData &data = **_data;
	RWLockRead dlock(data.get_lock());

	const bool material = ctx.material.size() == ctx.sdf.size();
	const VoxelSculptData::Chunk *chunk = nullptr;
	Vector3i chunk_pos(0x7fffffff, 0, 0);

	for (unsigned int i = 0; i < ctx.sdf.size(); ++i) {
		const Vector3i v = math::floor_to_int(ctx.positions[i] + Vector3f(0.5f));
		const Vector3i cpos(v.x >> VoxelSculptData::CHUNK_SIZE_PO2, v.y >> VoxelSculptData::CHUNK_SIZE_PO2,
				v.z >> VoxelSculptData::CHUNK_SIZE_PO2);
		if (cpos != chunk_pos) {
			chunk_pos = cpos;
			chunk = data.get_chunk(cpos);
		}
		if (chunk == nullptr) {
			continue;
		}
		const unsigned int index = VoxelSculptData::get_index(v - cpos * VoxelSculptData::CHUNK_SIZE);
		ctx.sdf[i] = VoxelSculptData::apply(*chunk, index, ctx.sdf[i]);
		if (material && chunk->material[index] != 0) {
			ctx.material[i] = chunk->material[index] - 1;
		}
	}
}

} // namespace zylann::voxel
