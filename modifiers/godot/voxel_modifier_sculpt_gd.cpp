#include "voxel_modifier_sculpt_gd.h"
#include "../../util/godot/core/callable_mp.h"
#include "../../util/math/conv.h"
#include "../voxel_modifier_sculpt.h"

namespace zylann::voxel::godot {

namespace {

zylann::voxel::VoxelModifierSculpt *get_sculpt(VoxelNode &volume, uint32_t id) {
	return get_modifier<zylann::voxel::VoxelModifierSculpt>(volume, id, zylann::voxel::VoxelModifier::TYPE_SCULPT);
}

} // namespace

Ref<VoxelSculptData> VoxelModifierSculpt::get_data() const {
	return _data;
}

void VoxelModifierSculpt::set_data(Ref<VoxelSculptData> data) {
	if (data == _data) {
		return;
	}
	const Callable changed = callable_mp(this, &VoxelModifierSculpt::on_data_changed);
	if (_data.is_valid() && _data->is_connected("changed", changed)) {
		_data->disconnect("changed", changed);
	}
	_data = data;
	if (_data.is_valid()) {
		_data->connect("changed", changed);
	}
	if (_volume == nullptr) {
		return;
	}
	zylann::voxel::VoxelModifierSculpt *sculpt = get_sculpt(*_volume, _modifier_id);
	ZN_ASSERT_RETURN(sculpt != nullptr);
	const AABB prev_aabb = sculpt->get_aabb();
	sculpt->set_data(_data);
	post_edit_modifier(*_volume, prev_aabb);
	post_edit_modifier(*_volume, sculpt->get_aabb());
}

// Chunks set from outside (undo, a script): everything the data covers before and after is made again.
void VoxelModifierSculpt::on_data_changed() {
	if (_volume == nullptr) {
		return;
	}
	zylann::voxel::VoxelModifierSculpt *sculpt = get_sculpt(*_volume, _modifier_id);
	ZN_ASSERT_RETURN(sculpt != nullptr);
	const AABB prev_aabb = sculpt->get_aabb();
	sculpt->update_data_bounds();
	post_edit_modifier(*_volume, prev_aabb);
	post_edit_modifier(*_volume, sculpt->get_aabb());
}

void VoxelModifierSculpt::sculpt_sphere(Vector3 center, float radius, BrushMode mode, float strength, int material) {
	ERR_FAIL_COND_MSG(_volume == nullptr, "The sculpt must be a child of a voxel volume.");
	ERR_FAIL_COND(_data.is_null());
	ERR_FAIL_INDEX(mode, BRUSH_MODE_COUNT);
	radius = math::max(radius, 0.5f);
	strength = math::clamp(strength, 0.f, 1.f);

	VoxelData &voxel_data = _volume->get_storage();
	zylann::voxel::VoxelModifierSculpt *sculpt = get_sculpt(*_volume, _modifier_id);
	ZN_ASSERT_RETURN(sculpt != nullptr);

	// Two voxels past the sphere: the shapes' distances near their surface, and smoothing's neighbours.
	const Vector3 reach(radius + 2.f, radius + 2.f, radius + 2.f);
	const Box3i box = Box3i::from_min_max(math::floor_to_int(center - reach), math::ceil_to_int(center + reach));
	const Vector3i size = box.size;
	const auto index = [size](const Vector3i p) { return p.x + size.x * (p.y + size.y * p.z); };

	VoxelSculptData &data = **_data;
	const float scale = VoxelSculptData::DISTANCE_SCALE;
	const float none = VoxelSculptData::NO_SHAPE / scale;
	const int cs = VoxelSculptData::CHUNK_SIZE;
	const int po2 = VoxelSculptData::CHUNK_SIZE_PO2;
	const auto chunk_of = [po2](const Vector3i v) { return Vector3i(v.x >> po2, v.y >> po2, v.z >> po2); };
	const auto shape_value = [scale](float sd) -> int16_t {
		const float q = Math::round(sd * scale);
		return q >= VoxelSculptData::NO_SHAPE ? VoxelSculptData::NO_SHAPE
											  : static_cast<int16_t>(math::max(q, -32767.f));
	};

	// Smoothing blurs what is there now: what lies below (the generator, then the modifiers before it) and the
	// sculpt.
	StdVector<float> current;
	if (mode == BRUSH_SMOOTH) {
		zylann::voxel::VoxelBuffer below(zylann::voxel::VoxelBuffer::ALLOCATOR_POOL);
		const zylann::voxel::VoxelFormat format = voxel_data.get_format();
		below.create(size, &format);
		const Ref<VoxelGenerator> generator = voxel_data.get_generator();
		if (generator.is_valid()) {
			VoxelGenerator::VoxelQueryData q{ below, box.position, 0 };
			generator->generate_block(q);
		}
		voxel_data.get_modifiers().apply(below, AABB(to_vec3(box.position), to_vec3(size)), sculpt->get_order());
		current.resize(Vector3iUtil::get_volume_u64(size));
		RWLockRead rlock(data.get_lock());
		for (int z = 0; z < size.z; ++z) {
			for (int y = 0; y < size.y; ++y) {
				for (int x = 0; x < size.x; ++x) {
					const Vector3i v = box.position + Vector3i(x, y, z);
					float sd = below.get_voxel_f(x, y, z, zylann::voxel::VoxelBuffer::CHANNEL_SDF);
					const Vector3i cpos = chunk_of(v);
					const VoxelSculptData::Chunk *chunk = data.get_chunk(cpos);
					if (chunk != nullptr) {
						sd = VoxelSculptData::apply(*chunk, VoxelSculptData::get_index(v - cpos * cs), sd);
					}
					current[index(Vector3i(x, y, z))] = sd;
				}
			}
		}
	}

	{
		RWLockWrite wlock(data.get_lock());

		for (int z = 1; z + 1 < size.z; ++z) {
			for (int y = 1; y + 1 < size.y; ++y) {
				for (int x = 1; x + 1 < size.x; ++x) {
					const Vector3i p(x, y, z);
					const Vector3i v = box.position + p;
					const float distance = (Vector3(v) - center).length();
					const float sd = distance - radius;
					// Full in the middle, nothing at the edge, for the blending brushes.
					const float falloff = math::clamp(1.f - (distance / radius) * (distance / radius), 0.f, 1.f);
					// The shapes reach past the sphere; the other brushes stop at its edge.
					const bool reaches = mode == BRUSH_ADD || mode == BRUSH_REMOVE || falloff > 0.f;
					if (!reaches) {
						continue;
					}
					const Vector3i cpos = chunk_of(v);
					const unsigned int i = VoxelSculptData::get_index(v - cpos * cs);
					const VoxelSculptData::Chunk *existing = data.get_chunk(cpos);
					float add = existing == nullptr ? none : existing->add[i] / scale;
					float cut = existing == nullptr ? none : existing->cut[i] / scale;
					float offset = existing == nullptr ? 0.f : existing->offset[i] / scale;
					int paint = -1;
					switch (mode) {
						case BRUSH_ADD:
							// Adding inside the sphere also takes back what was cut there.
							add = math::lerp(add, math::min(add, sd), strength);
							cut = math::lerp(cut, math::max(cut, -sd), strength);
							break;
						case BRUSH_REMOVE:
							cut = math::lerp(cut, math::min(cut, sd), strength);
							add = math::lerp(add, math::max(add, -sd), strength);
							break;
						case BRUSH_SMOOTH: {
							const float average = (current[index(p + Vector3i(1, 0, 0))] +
														  current[index(p - Vector3i(1, 0, 0))] +
														  current[index(p + Vector3i(0, 1, 0))] +
														  current[index(p - Vector3i(0, 1, 0))] +
														  current[index(p + Vector3i(0, 0, 1))] +
														  current[index(p - Vector3i(0, 0, 1))]) /
									6.f;
							offset += (average - current[index(p)]) * strength * falloff;
						} break;
						case BRUSH_PAINT:
							if (sd < 0.f) {
								paint = material < 0 ? 0 : math::min(material, 254) + 1;
							}
							break;
						case BRUSH_ERASE: {
							const float w = strength * falloff;
							add = math::lerp(add, none, w);
							cut = math::lerp(cut, none, w);
							offset = math::lerp(offset, 0.f, w);
							if (w > 0.5f) {
								add = none;
								cut = none;
								offset = 0.f;
								paint = 0;
							}
						} break;
						default:
							break;
					}
					const int16_t add_q = shape_value(add);
					const int16_t cut_q = shape_value(cut);
					const int16_t offset_q =
							static_cast<int16_t>(math::clamp(Math::round(offset * scale), -32767.f, 32767.f));
					const bool no_shape = add_q == VoxelSculptData::NO_SHAPE && cut_q == VoxelSculptData::NO_SHAPE;
					if (existing == nullptr && no_shape && offset_q == 0 && paint <= 0) {
						continue;
					}
					VoxelSculptData::Chunk &chunk = data.get_or_create_chunk(cpos);
					chunk.add[i] = add_q;
					chunk.cut[i] = cut_q;
					chunk.offset[i] = offset_q;
					if (paint >= 0) {
						chunk.material[i] = paint;
					}
				}
			}
		}

		// Chunks the brush took back to nothing go.
		box.downscaled(cs).for_each_cell([&data](const Vector3i cpos) {
			const VoxelSculptData::Chunk *chunk = data.get_chunk(cpos);
			if (chunk != nullptr && chunk->is_empty()) {
				data.erase_chunk(cpos);
			}
		});
		data.update_bounds();
	}

	sculpt->update_data_bounds();
	post_edit_modifier(*_volume, AABB(to_vec3(box.position), to_vec3(size)));
}

zylann::voxel::VoxelModifier *VoxelModifierSculpt::create(zylann::voxel::VoxelModifierStack &modifiers, uint32_t id) {
	zylann::voxel::VoxelModifierSculpt *sculpt = modifiers.add_modifier<zylann::voxel::VoxelModifierSculpt>(id);
	sculpt->set_data(_data);
	return sculpt;
}

void VoxelModifierSculpt::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_data", "data"), &VoxelModifierSculpt::set_data);
	ClassDB::bind_method(D_METHOD("get_data"), &VoxelModifierSculpt::get_data);
	ClassDB::bind_method(
			D_METHOD("sculpt_sphere", "center", "radius", "mode", "strength", "material"),
			&VoxelModifierSculpt::sculpt_sphere
	);

	ADD_PROPERTY(
			PropertyInfo(Variant::OBJECT, "data", PROPERTY_HINT_RESOURCE_TYPE, VoxelSculptData::get_class_static()),
			"set_data",
			"get_data"
	);

	BIND_ENUM_CONSTANT(BRUSH_ADD);
	BIND_ENUM_CONSTANT(BRUSH_REMOVE);
	BIND_ENUM_CONSTANT(BRUSH_SMOOTH);
	BIND_ENUM_CONSTANT(BRUSH_PAINT);
	BIND_ENUM_CONSTANT(BRUSH_ERASE);
	BIND_ENUM_CONSTANT(BRUSH_MODE_COUNT);
}

} // namespace zylann::voxel::godot
