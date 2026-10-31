#include "voxel_modifier_path_gd.h"
#include "../../util/math/conv.h"
#include "../voxel_modifier_path.h"

namespace zylann::voxel::godot {

namespace {

zylann::voxel::VoxelModifierPath *get_path_modifier(VoxelNode &volume, uint32_t id) {
	return get_modifier<zylann::voxel::VoxelModifierPath>(volume, id, zylann::voxel::VoxelModifier::TYPE_PATH);
}

StdVector<Vector3f> to_points(const PackedVector3Array &points) {
	StdVector<Vector3f> out;
	out.resize(points.size());
	for (int i = 0; i < points.size(); ++i) {
		out[i] = to_vec3f(points[i]);
	}
	return out;
}

} // namespace

PackedVector3Array VoxelModifierPath::get_points() const {
	return _points;
}

void VoxelModifierPath::set_points(PackedVector3Array points) {
	_points = points;
	if (_volume == nullptr) {
		return;
	}
	zylann::voxel::VoxelModifierPath *path = get_path_modifier(*_volume, _modifier_id);
	ZN_ASSERT_RETURN(path != nullptr);
	const AABB prev_aabb = path->get_aabb();
	const StdVector<Vector3f> local_points = to_points(_points);
	path->set_points(to_span_const(local_points));
	post_edit_modifier(*_volume, prev_aabb);
	post_edit_modifier(*_volume, path->get_aabb());
}

float VoxelModifierPath::get_width() const {
	return _width;
}

void VoxelModifierPath::set_width(float width) {
	_width = math::max(width, 0.f);
	update_profile();
}

float VoxelModifierPath::get_bottom() const {
	return _bottom;
}

void VoxelModifierPath::set_bottom(float bottom) {
	_bottom = bottom;
	update_profile();
}

float VoxelModifierPath::get_top() const {
	return _top;
}

void VoxelModifierPath::set_top(float top) {
	_top = top;
	update_profile();
}

float VoxelModifierPath::get_rounding() const {
	return _rounding;
}

void VoxelModifierPath::set_rounding(float rounding) {
	_rounding = math::max(rounding, 0.f);
	update_profile();
}

void VoxelModifierPath::update_profile() {
	if (_volume == nullptr) {
		return;
	}
	zylann::voxel::VoxelModifierPath *path = get_path_modifier(*_volume, _modifier_id);
	ZN_ASSERT_RETURN(path != nullptr);
	const AABB prev_aabb = path->get_aabb();
	path->set_profile(_width, _bottom, _top, _rounding);
	post_edit_modifier(*_volume, prev_aabb);
	post_edit_modifier(*_volume, path->get_aabb());
}

zylann::voxel::VoxelModifier *VoxelModifierPath::create(zylann::voxel::VoxelModifierStack &modifiers, uint32_t id) {
	zylann::voxel::VoxelModifierPath *path = modifiers.add_modifier<zylann::voxel::VoxelModifierPath>(id);
	path->set_profile(_width, _bottom, _top, _rounding);
	const StdVector<Vector3f> local_points = to_points(_points);
	path->set_points(to_span_const(local_points));
	return path;
}

void VoxelModifierPath::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_points", "points"), &VoxelModifierPath::set_points);
	ClassDB::bind_method(D_METHOD("get_points"), &VoxelModifierPath::get_points);
	ClassDB::bind_method(D_METHOD("set_width", "width"), &VoxelModifierPath::set_width);
	ClassDB::bind_method(D_METHOD("get_width"), &VoxelModifierPath::get_width);
	ClassDB::bind_method(D_METHOD("set_bottom", "bottom"), &VoxelModifierPath::set_bottom);
	ClassDB::bind_method(D_METHOD("get_bottom"), &VoxelModifierPath::get_bottom);
	ClassDB::bind_method(D_METHOD("set_top", "top"), &VoxelModifierPath::set_top);
	ClassDB::bind_method(D_METHOD("get_top"), &VoxelModifierPath::get_top);
	ClassDB::bind_method(D_METHOD("set_rounding", "rounding"), &VoxelModifierPath::set_rounding);
	ClassDB::bind_method(D_METHOD("get_rounding"), &VoxelModifierPath::get_rounding);

	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "points"), "set_points", "get_points");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width", PROPERTY_HINT_RANGE, "0.0,100.0,0.1,or_greater"),
			"set_width", "get_width");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bottom", PROPERTY_HINT_RANGE, "-100.0,100.0,0.1,or_less,or_greater"),
			"set_bottom", "get_bottom");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "top", PROPERTY_HINT_RANGE, "-100.0,100.0,0.1,or_less,or_greater"),
			"set_top", "get_top");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rounding", PROPERTY_HINT_RANGE, "0.0,10.0,0.05,or_greater"),
			"set_rounding", "get_rounding");
}

} // namespace zylann::voxel::godot
