#include "concave_polygon_shape_3d.h"
#include "../../math/conv.h"
#include "../../profiling.h"
#include "../core/packed_arrays.h"
#include "collision_shape_3d.h"
#include "mesh.h"

namespace zylann::godot {

namespace {

// Jolt's own test (IndexedTriangle::IsDegenerate): the triangle spans no area.
inline bool is_degenerate_triangle(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
	return (b - a).cross(c - a).length_squared() < 1e-12f;
}

// Drops degenerate triangles from a deindexed face list, in place. A mesher can emit them (a collapsed quad, a crossing
// at a cell corner); Jolt refuses a mesh with nothing else in it, so they go before the shape is made.
void remove_degenerate_faces(PackedVector3Array &face_points) {
	Vector3 *w = face_points.ptrw();
	const int count = face_points.size();
	int kept = 0;
	for (int i = 0; i + 2 < count; i += 3) {
		if (is_degenerate_triangle(w[i], w[i + 1], w[i + 2])) {
			continue;
		}
		if (kept != i) {
			w[kept] = w[i];
			w[kept + 1] = w[i + 1];
			w[kept + 2] = w[i + 2];
		}
		kept += 3;
	}
	face_points.resize(kept);
}

// The shape for a deindexed face list, or null when no real triangle is left.
Ref<ConcavePolygonShape3D> make_shape(PackedVector3Array face_points) {
	remove_degenerate_faces(face_points);
	Ref<ConcavePolygonShape3D> shape;
	if (face_points.size() < 3) {
		return shape;
	}
	ZN_PROFILE_SCOPE_NAMED("Godot shape");
	shape.instantiate();
	shape->set_faces(face_points);
	return shape;
}

} // namespace

Ref<ConcavePolygonShape3D> create_concave_polygon_shape(const Span<const Array> surfaces) {
	// Faster version of Mesh::create_trimesh_shape(), because `create_trimesh_shape` creates a Trimesh internally along
	// the way, which is super slow
	// See https://github.com/Zylann/godot_voxel/issues/54

	ZN_PROFILE_SCOPE();

	PackedVector3Array face_points;
	int face_points_size = 0;

	// find the correct size for face_points
	for (unsigned int i = 0; i < surfaces.size(); i++) {
		const Array &surface_arrays = surfaces[i];
		if (surface_arrays.size() == 0) {
			// That surface is empty
			continue;
		}
		// If the surface is not empty then it must have an expected amount of data arrays
		ERR_CONTINUE(surface_arrays.size() != Mesh::ARRAY_MAX);
		PackedInt32Array indices = surface_arrays[Mesh::ARRAY_INDEX];
		face_points_size += indices.size();
	}
	face_points.resize(face_points_size);

	if (face_points_size < 3) {
		return Ref<ConcavePolygonShape3D>();
	}

	// Deindex surfaces into a single one
	unsigned int face_points_offset = 0;
	for (unsigned int i = 0; i < surfaces.size(); i++) {
		const Array &surface_arrays = surfaces[i];
		if (surface_arrays.size() == 0) {
			continue;
		}
		PackedVector3Array positions = surface_arrays[Mesh::ARRAY_VERTEX];
		PackedInt32Array indices = surface_arrays[Mesh::ARRAY_INDEX];

		ERR_FAIL_COND_V(positions.size() < 3, Ref<ConcavePolygonShape3D>());
		ERR_FAIL_COND_V(indices.size() < 3, Ref<ConcavePolygonShape3D>());
		ERR_FAIL_COND_V(indices.size() % 3 != 0, Ref<ConcavePolygonShape3D>());

		unsigned int face_points_count = face_points_offset + indices.size();

		{
			Vector3 *w = face_points.ptrw();
			const int *index_r = indices.ptr();
			const Vector3 *position_r = positions.ptr();

			for (unsigned int p = face_points_offset; p < face_points_count; ++p) {
				const int ii = p - face_points_offset;
#ifdef DEBUG_ENABLED
				CRASH_COND(ii < 0 || ii >= indices.size());
#endif
				const int index = index_r[ii];
#ifdef DEBUG_ENABLED
				CRASH_COND(index < 0 || index >= positions.size());
#endif
				w[p] = position_r[index];
			}
		}

		face_points_offset += indices.size();
	}

	return make_shape(face_points);
}

PackedVector3Array deindex_mesh_to_packed_vector3_array(
		const Span<const Vector3f> positions,
		const Span<const int> indices
) {
	ZN_PROFILE_SCOPE();

	PackedVector3Array face_points;
	face_points.resize(indices.size());
	{
		Vector3 *w = face_points.ptrw();
		for (unsigned int ii = 0; ii < indices.size(); ++ii) {
			const int index = indices[ii];
			w[ii] = to_vec3(positions[index]);
		}
	}
	return face_points;
}

PackedVector3Array deindex_mesh_to_packed_vector3_array(
		const Span<const Vector3> vertices,
		const Span<const int32_t> indices
) {
	ZN_PROFILE_SCOPE();

	PackedVector3Array face_points;
	face_points.resize(indices.size());

	{
		Span<Vector3> dst(face_points.ptrw(), face_points.size());
		for (unsigned int ii = 0; ii < indices.size(); ++ii) {
			const int index = indices[ii];
			dst[ii] = vertices[index];
		}
	}

	return face_points;
}

Ref<ConcavePolygonShape3D> create_concave_polygon_shape(
		const Span<const Vector3f> positions,
		const Span<const int> indices
) {
	ZN_PROFILE_SCOPE();

	if (indices.size() < 3) {
		return Ref<ConcavePolygonShape3D>();
	}

	ERR_FAIL_COND_V(positions.size() < 3, Ref<ConcavePolygonShape3D>());
	ERR_FAIL_COND_V(indices.size() < 3, Ref<ConcavePolygonShape3D>());
	ERR_FAIL_COND_V(indices.size() % 3 != 0, Ref<ConcavePolygonShape3D>());

	const PackedVector3Array face_points = deindex_mesh_to_packed_vector3_array(positions, indices);

	return make_shape(face_points);
}

// This variant may use a lower index count so a subset of the mesh is used to create the collision shape.
Ref<ConcavePolygonShape3D> create_concave_polygon_shape(
		const Array &surface_arrays,
		const unsigned int vertex_count,
		const unsigned int index_count
) {
	ZN_PROFILE_SCOPE();

	Ref<ConcavePolygonShape3D> shape;

	if (surface_arrays.size() == 0) {
		// Empty
		return shape;
	}
	ZN_ASSERT(surface_arrays.size() == Mesh::ARRAY_MAX);

	const PackedInt32Array indices = surface_arrays[Mesh::ARRAY_INDEX];
	ERR_FAIL_COND_V(index_count > static_cast<unsigned int>(indices.size()), shape);
	if (indices.size() < 3) {
		// Empty
		return shape;
	}

	const PackedVector3Array positions = surface_arrays[Mesh::ARRAY_VERTEX];
	ERR_FAIL_COND_V(vertex_count > static_cast<unsigned int>(positions.size()), shape);

	ERR_FAIL_COND_V(positions.size() < 3, shape);
	ERR_FAIL_COND_V(indices.size() < 3, shape);
	ERR_FAIL_COND_V(indices.size() % 3 != 0, shape);

	const PackedVector3Array face_points = deindex_mesh_to_packed_vector3_array(
			to_span(positions).sub(0, vertex_count), //
			to_span(indices).sub(0, index_count)
	);

	return make_shape(face_points);
}

} // namespace zylann::godot
