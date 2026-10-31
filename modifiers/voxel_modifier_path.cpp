#include "voxel_modifier_path.h"
#include "../util/math/conv.h"
#include "../util/profiling.h"

namespace zylann::voxel {

void VoxelModifierPath::set_points(Span<const Vector3f> points) {
	RWLockWrite wlock(_rwlock);
	_points.resize(points.size());
	for (unsigned int i = 0; i < points.size(); ++i) {
		_points[i] = points[i];
	}
	update_segments();
	update_aabb();
}

void VoxelModifierPath::set_profile(float width, float bottom, float top, float rounding) {
	RWLockWrite wlock(_rwlock);
	_width = math::max(width, 0.f);
	_bottom = math::min(bottom, top);
	_top = math::max(bottom, top);
	_rounding = math::clamp(rounding, 0.f, math::min(_width, _top - _bottom) * 0.5f);
	update_segments();
	update_aabb();
}

void VoxelModifierPath::update_segments() {
	_segments.clear();
	const float half_width = _width * 0.5f;
	const float centre = (_top + _bottom) * 0.5f;
	const float reach = math::max(half_width, (_top - _bottom) * 0.5f);
	for (unsigned int i = 0; i + 1 < _points.size(); ++i) {
		const Vector3f a = _points[i];
		const Vector3f b = _points[i + 1];
		const Vector3f d = b - a;
		const float length = math::length(d);
		if (length < 0.0001f) {
			continue;
		}
		Segment s;
		s.along = d / length;
		Vector3f side = math::cross(s.along, Vector3f(0, 1, 0));
		if (math::length(side) < 0.0001f) {
			side = Vector3f(0, 0, 1);
		}
		s.side = math::normalized(side);
		s.up = math::cross(s.side, s.along);
		s.origin = a + s.up * centre;
		// Inner joints overlap by the half width so the outside of a bend has no notch; the path's ends stay square.
		s.start = i == 0 ? 0.f : -half_width;
		s.end = i + 2 == _points.size() ? length : length + half_width;
		const Vector3f grow(reach + _rounding);
		s.aabb_min = math::min(a, b) - grow - Vector3f(half_width);
		s.aabb_max = math::max(a, b) + grow + Vector3f(half_width);
		s.aabb_min.y = math::min(a.y, b.y) + _bottom - half_width;
		s.aabb_max.y = math::max(a.y, b.y) + _top + half_width;
		_segments.push_back(s);
	}
}

void VoxelModifierPath::update_aabb() {
	if (_segments.size() == 0) {
		_aabb = AABB(get_transform().origin, Vector3());
		return;
	}
	Vector3f lo = _segments[0].aabb_min;
	Vector3f hi = _segments[0].aabb_max;
	for (const Segment &s : _segments) {
		lo = math::min(lo, s.aabb_min);
		hi = math::max(hi, s.aabb_max);
	}
	const Vector3f grow(get_smoothness());
	_aabb = get_transform().xform(AABB(to_vec3(lo - grow), to_vec3(hi - lo + grow * 2.f)));
}

void VoxelModifierPath::apply(VoxelModifierContext ctx) const {
	ZN_PROFILE_SCOPE();
	RWLockRead rlock(_rwlock);
	if (_segments.size() == 0 || ctx.positions.size() == 0) {
		return;
	}

	const Transform3D to_local = get_transform().affine_inverse();

	// Only the segments near this area take part.
	Vector3f lo = ctx.positions[0];
	Vector3f hi = ctx.positions[0];
	for (const Vector3f p : ctx.positions) {
		lo = math::min(lo, p);
		hi = math::max(hi, p);
	}
	const AABB area = to_local.xform(AABB(to_vec3(lo), to_vec3(hi - lo)));
	const Vector3f area_min = to_vec3f(area.position) - Vector3f(get_smoothness());
	const Vector3f area_max = to_vec3f(area.position + area.size) + Vector3f(get_smoothness());
	thread_local StdVector<const Segment *> tls_near;
	tls_near.clear();
	for (const Segment &s : _segments) {
		if (s.aabb_max.x < area_min.x || s.aabb_max.y < area_min.y || s.aabb_max.z < area_min.z ||
			s.aabb_min.x > area_max.x || s.aabb_min.y > area_max.y || s.aabb_min.z > area_max.z) {
			continue;
		}
		tls_near.push_back(&s);
	}
	if (tls_near.size() == 0) {
		return;
	}

	const float half_width = _width * 0.5f;
	const float half_height = (_top - _bottom) * 0.5f;
	const float rounding = _rounding;
	const StdVector<const Segment *> &near = tls_near;

	apply_shape(ctx, [&near, &to_local, half_width, half_height, rounding](const Vector3f world_pos) {
		const Vector3f pos = to_vec3f(to_local.xform(to_vec3(world_pos)));
		float best = 1e9f;
		for (const Segment *s : near) {
			const Vector3f d = pos - s->origin;
			const float half_length = (s->end - s->start) * 0.5f;
			// Rounded box in the segment's frame (x along, y up, z side).
			const Vector3f q = Vector3f(
					math::abs(math::dot(d, s->along) - (s->start + half_length)) - half_length + rounding,
					math::abs(math::dot(d, s->up)) - half_height + rounding,
					math::abs(math::dot(d, s->side)) - half_width + rounding
			);
			const float outside = math::length(math::max(q, Vector3f(0.f)));
			const float inside = math::min(math::max(q.x, math::max(q.y, q.z)), 0.f);
			best = math::min(best, outside + inside - rounding);
		}
		return best;
	});
}

} // namespace zylann::voxel
