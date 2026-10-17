#include "voxel_mesher_dual_contouring.h"
#include "../../storage/voxel_buffer.h"
#include "../../util/containers/span.h"
#include "../../util/containers/std_vector.h"
#include "../../util/godot/classes/rendering_server.h"
#include "../../util/godot/core/packed_arrays.h"
#include "../../util/math/vector3f.h"
#include "../../util/profiling.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

using namespace zylann::godot;

namespace zylann::voxel {

namespace {

// Voxels needed before and after the meshed area: cells at -1 own the quads on the block's lower faces, and normals
// are central differences, which reach one voxel further.
constexpr int PADDING_MIN = 2;
constexpr int PADDING_MAX = 2;

struct V3 {
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;
};

inline V3 operator+(V3 a, V3 b) {
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}
inline V3 operator-(V3 a, V3 b) {
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}
inline V3 operator*(V3 a, float s) {
	return { a.x * s, a.y * s, a.z * s };
}
inline float dot(V3 a, V3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline V3 cross(V3 a, V3 b) {
	return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline V3 normalized_or_up(V3 v) {
	const float len = std::sqrt(dot(v, v));
	if (len < 1e-12f) {
		return { 0.f, 1.f, 0.f };
	}
	return v * (1.f / len);
}
inline float get_axis(V3 v, int axis) {
	return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}
inline void set_axis(V3 &v, int axis, float value) {
	if (axis == 0) {
		v.x = value;
	} else if (axis == 1) {
		v.y = value;
	} else {
		v.z = value;
	}
}

// Symmetric 3x3 matrix stored as its 6 unique entries: xx, xy, xz, yy, yz, zz.
struct Sym3 {
	float m[6] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

	void add_outer(V3 n) {
		m[0] += n.x * n.x;
		m[1] += n.x * n.y;
		m[2] += n.x * n.z;
		m[3] += n.y * n.y;
		m[4] += n.y * n.z;
		m[5] += n.z * n.z;
	}

	V3 mul(V3 v) const {
		return { m[0] * v.x + m[1] * v.y + m[2] * v.z, m[1] * v.x + m[3] * v.y + m[4] * v.z,
			m[2] * v.x + m[4] * v.y + m[5] * v.z };
	}
};

// Solves A x = b in the least-squares sense with a truncated pseudo-inverse, using Jacobi eigen-decomposition of the
// symmetric matrix. Eigenvalues below `threshold` times the largest are dropped, which keeps the solution near the
// origin along directions the planes don't constrain.
V3 solve_pseudo_inverse(const Sym3 &sym, V3 b, float threshold) {
	float a[3][3] = { { sym.m[0], sym.m[1], sym.m[2] }, { sym.m[1], sym.m[3], sym.m[4] },
		{ sym.m[2], sym.m[4], sym.m[5] } };
	float v[3][3] = { { 1.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, { 0.f, 0.f, 1.f } };

	for (int sweep = 0; sweep < 8; ++sweep) {
		const float off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
		if (off < 1e-14f) {
			break;
		}
		for (int p = 0; p < 2; ++p) {
			for (int q = p + 1; q < 3; ++q) {
				if (std::abs(a[p][q]) < 1e-12f) {
					continue;
				}
				const float theta = (a[q][q] - a[p][p]) / (2.f * a[p][q]);
				const float t = (theta >= 0.f ? 1.f : -1.f) / (std::abs(theta) + std::sqrt(theta * theta + 1.f));
				const float c = 1.f / std::sqrt(t * t + 1.f);
				const float s = t * c;
				for (int k = 0; k < 3; ++k) {
					const float akp = a[k][p];
					const float akq = a[k][q];
					a[k][p] = c * akp - s * akq;
					a[k][q] = s * akp + c * akq;
				}
				for (int k = 0; k < 3; ++k) {
					const float apk = a[p][k];
					const float aqk = a[q][k];
					a[p][k] = c * apk - s * aqk;
					a[q][k] = s * apk + c * aqk;
				}
				for (int k = 0; k < 3; ++k) {
					const float vkp = v[k][p];
					const float vkq = v[k][q];
					v[k][p] = c * vkp - s * vkq;
					v[k][q] = s * vkp + c * vkq;
				}
			}
		}
	}

	const float eig[3] = { a[0][0], a[1][1], a[2][2] };
	const float largest = std::max(std::abs(eig[0]), std::max(std::abs(eig[1]), std::abs(eig[2])));
	V3 x;
	if (largest < 1e-12f) {
		return x;
	}
	for (int i = 0; i < 3; ++i) {
		if (std::abs(eig[i]) < threshold * largest) {
			continue;
		}
		const V3 axis = { v[0][i], v[1][i], v[2][i] };
		x = x + axis * (dot(axis, b) / eig[i]);
	}
	return x;
}

// Cell corner offsets, corner i at (i & 1, (i >> 1) & 1, (i >> 2) & 1), and the 12 edges as corner pairs.
constexpr int CORNER_OFFSETS[8][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 0, 0, 1 }, { 1, 0, 1 },
	{ 0, 1, 1 }, { 1, 1, 1 } };
constexpr int EDGES[12][2] = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 }, { 0, 4 },
	{ 1, 5 }, { 2, 6 }, { 3, 7 } };

struct WeightedIndex {
	uint8_t index;
	uint8_t count;
};

// Picks the 4 most common material indices among a cell's corners, the same way Transvoxel's Single S4 mode does, and
// maps each corner to one of those 4 slots.
void select_cell_materials(
		const std::array<uint8_t, 8> &corner_indices,
		std::array<uint8_t, 4> &selected,
		std::array<uint8_t, 8> &corner_slots
) {
	std::array<WeightedIndex, 8> items{};
	unsigned int count = 0;
	for (const uint8_t mi : corner_indices) {
		bool found = false;
		for (unsigned int i = 0; i < count; ++i) {
			if (items[i].index == mi) {
				++items[i].count;
				found = true;
				break;
			}
		}
		if (!found) {
			items[count++] = { mi, 1 };
		}
	}
	// Most common first, then lowest index; at most 8 items, so a plain insertion sort.
	for (unsigned int i = 1; i < count; ++i) {
		const WeightedIndex item = items[i];
		unsigned int j = i;
		while (j > 0 && (items[j - 1].count < item.count || (items[j - 1].count == item.count && items[j - 1].index > item.index))) {
			items[j] = items[j - 1];
			--j;
		}
		items[j] = item;
	}
	const unsigned int used = std::min(count, 4u);
	for (unsigned int i = 0; i < 4; ++i) {
		// Unused slots get placeholder indices, which carry no weight.
		selected[i] = i < used ? items[i].index : static_cast<uint8_t>(items[used - 1].index + i);
	}
	for (unsigned int c = 0; c < 8; ++c) {
		corner_slots[c] = 0;
		for (unsigned int i = 0; i < used; ++i) {
			if (selected[i] == corner_indices[c]) {
				corner_slots[c] = i;
				break;
			}
		}
	}
}

// How far past its own cell a vertex may go.
constexpr float CELL_MARGIN = 0.25f;

// A stored edge normal is accepted when its crossing is within this fraction of an edge of the current one.
constexpr float EDGE_NORMAL_T_TOLERANCE = 0.25f;

// Edge normal layout: bits 0-7 the crossing's position along the edge (1-255, 0 means none), bits 8-19 and 20-31 the
// normal in octahedral encoding.
V3 decode_edge_normal(uint32_t data) {
	float u = static_cast<float>((data >> 8) & 4095) / 4095.f * 2.f - 1.f;
	float v = static_cast<float>((data >> 20) & 4095) / 4095.f * 2.f - 1.f;
	const float z = 1.f - std::abs(u) - std::abs(v);
	if (z < 0.f) {
		const float fu = (1.f - std::abs(v)) * (u >= 0.f ? 1.f : -1.f);
		const float fv = (1.f - std::abs(u)) * (v >= 0.f ? 1.f : -1.f);
		u = fu;
		v = fv;
	}
	return normalized_or_up(V3{ u, v, z });
}

inline uint32_t pack_bytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	return static_cast<uint32_t>(a) | (static_cast<uint32_t>(b) << 8) | (static_cast<uint32_t>(c) << 16) |
			(static_cast<uint32_t>(d) << 24);
}

struct Arrays {
	StdVector<Vector3f> vertices;
	StdVector<Vector3f> normals;
	StdVector<float> texturing; // 2 x 32-bit per vertex: packed indices, packed weights
	StdVector<int32_t> indices;
	StdVector<int32_t> cell_vertex;
	StdVector<float> sdf;
	StdVector<uint8_t> material;
	StdVector<StdVector<uint32_t>> edge_normals;

	void clear() {
		vertices.clear();
		normals.clear();
		texturing.clear();
		indices.clear();
	}
};

Arrays &get_tls_arrays() {
	thread_local Arrays tls_arrays;
	return tls_arrays;
}

} // namespace

VoxelMesherDualContouring::VoxelMesherDualContouring() {
	set_padding(PADDING_MIN, PADDING_MAX);
}

void VoxelMesherDualContouring::build(VoxelMesher::Output &output, const VoxelMesher::Input &input) {
	ZN_PROFILE_SCOPE();

	const VoxelBuffer &voxels = input.voxels;
	const unsigned int sdf_channel = VoxelBuffer::CHANNEL_SDF;
	if (voxels.is_uniform(sdf_channel)) {
		return;
	}

	const Vector3i size = voxels.get_size();
	const Vector3i block_size = size - Vector3iUtil::create(PADDING_MIN + PADDING_MAX);
	if (block_size.x <= 0 || block_size.y <= 0 || block_size.z <= 0) {
		return;
	}

	Arrays &arrays = get_tls_arrays();
	arrays.clear();

	// Copy the SDF into floats once; x varies fastest. Negative is matter, as everywhere in the module.
	const int sx = size.x;
	const int sxy = size.x * size.y;
	const int voxel_count = sxy * size.z;
	arrays.sdf.resize(voxel_count);
	for (int z = 0; z < size.z; ++z) {
		for (int y = 0; y < size.y; ++y) {
			for (int x = 0; x < size.x; ++x) {
				arrays.sdf[x + sx * y + sxy * z] = voxels.get_voxel_f(x, y, z, sdf_channel);
			}
		}
	}

	const bool textured = _texturing_mode == TEXTURES_SINGLE_S4 &&
			voxels.get_channel_depth(VoxelBuffer::CHANNEL_INDICES) == VoxelBuffer::DEPTH_8_BIT;
	if (textured) {
		arrays.material.resize(voxel_count);
		if (voxels.is_uniform(VoxelBuffer::CHANNEL_INDICES)) {
			const uint8_t value = voxels.get_voxel(Vector3i(), VoxelBuffer::CHANNEL_INDICES);
			std::fill(arrays.material.begin(), arrays.material.end(), value);
		} else {
			for (int z = 0; z < size.z; ++z) {
				for (int y = 0; y < size.y; ++y) {
					for (int x = 0; x < size.x; ++x) {
						arrays.material[x + sx * y + sxy * z] = voxels.get_voxel(x, y, z, VoxelBuffer::CHANNEL_INDICES);
					}
				}
			}
		}
	}

	const float *sdf = arrays.sdf.data();
	auto solid = [sdf](int i) { return sdf[i] < 0.f; };
	// Outward (toward air) gradient at a padded voxel, by central differences. Within a voxel or so of an edge or
	// corner this blends the faces meeting there, which is why exact edge normals are read when they're provided.
	auto gradient = [sdf, sx, sxy](int i) {
		return V3{ sdf[i + 1] - sdf[i - 1], sdf[i + sx] - sdf[i - sx], sdf[i + sxy] - sdf[i - sxy] };
	};

	// Exact normals for voxel edges, one channel per axis, written by the generator where it knows the surface.
	std::array<const uint32_t *, 3> edge_normals = { nullptr, nullptr, nullptr };
	if (_edge_normals_channel >= 0 && _edge_normals_channel + 2 < VoxelBuffer::MAX_CHANNELS) {
		arrays.edge_normals.resize(3);
		for (int axis = 0; axis < 3; ++axis) {
			const unsigned int channel = _edge_normals_channel + axis;
			if (voxels.get_channel_depth(channel) != VoxelBuffer::DEPTH_32_BIT) {
				continue;
			}
			if (voxels.is_uniform(channel) && voxels.get_voxel(Vector3i(), channel) == 0) {
				continue;
			}
			StdVector<uint32_t> &values = arrays.edge_normals[axis];
			values.resize(voxel_count);
			for (int z = 0; z < size.z; ++z) {
				for (int y = 0; y < size.y; ++y) {
					for (int x = 0; x < size.x; ++x) {
						values[x + sx * y + sxy * z] = static_cast<uint32_t>(voxels.get_voxel(x, y, z, channel));
					}
				}
			}
			edge_normals[axis] = values.data();
		}
	}

	// Cells from -1 to block_size - 1 on each axis (in block coordinates); padded voxel = block coordinate + PADDING_MIN.
	const int cells_x = block_size.x + 1;
	const int cells_y = block_size.y + 1;
	const int cells_z = block_size.z + 1;
	arrays.cell_vertex.assign(cells_x * cells_y * cells_z, -2);
	auto cell_slot = [cells_x, cells_y](int cx, int cy, int cz) { return (cx + 1) + cells_x * ((cy + 1) + cells_y * (cz + 1)); };

	const float sharpness = _sharpness;
	const float bias = _mass_point_bias;

	// Places a cell's vertex (once) and returns its index, or -1 if the surface doesn't cross the cell.
	auto get_cell_vertex = [&](int cx, int cy, int cz) -> int {
		int &slot = arrays.cell_vertex[cell_slot(cx, cy, cz)];
		if (slot != -2) {
			return slot;
		}
		slot = -1;

		std::array<int, 8> corner;
		std::array<float, 8> value;
		unsigned int mask = 0;
		for (int c = 0; c < 8; ++c) {
			const int px = cx + CORNER_OFFSETS[c][0] + PADDING_MIN;
			const int py = cy + CORNER_OFFSETS[c][1] + PADDING_MIN;
			const int pz = cz + CORNER_OFFSETS[c][2] + PADDING_MIN;
			corner[c] = px + sx * py + sxy * pz;
			value[c] = sdf[corner[c]];
			if (solid(corner[c])) {
				mask |= 1u << c;
			}
		}
		if (mask == 0 || mask == 255) {
			return slot;
		}

		std::array<uint8_t, 4> selected = { 0, 0, 0, 0 };
		std::array<uint8_t, 8> corner_slots = { 0, 0, 0, 0, 0, 0, 0, 0 };
		if (textured) {
			std::array<uint8_t, 8> corner_materials;
			for (int c = 0; c < 8; ++c) {
				corner_materials[c] = arrays.material[corner[c]];
			}
			select_cell_materials(corner_materials, selected, corner_slots);
		}

		Sym3 ata;
		V3 atb;
		V3 mass;
		V3 normal_sum;
		float weights[4] = { 0.f, 0.f, 0.f, 0.f };
		int crossings = 0;
		const V3 cell_origin = { static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz) };
		for (const auto &edge : EDGES) {
			const int c0 = edge[0];
			const int c1 = edge[1];
			if (((mask >> c0) & 1) == ((mask >> c1) & 1)) {
				continue;
			}
			const float d0 = value[c0];
			const float d1 = value[c1];
			float t = (d0 == d1) ? 0.5f : std::clamp(d0 / (d0 - d1), 0.f, 1.f);
			V3 n;
			bool exact = false;
			// Edges are stored on their lower voxel; a stored normal only counts if the crossing is still where it
			// was when the normal was written, so a spot dug since falls back to the gradient.
			const int axis = (c0 ^ c1) == 1 ? 0 : ((c0 ^ c1) == 2 ? 1 : 2);
			if (edge_normals[axis] != nullptr) {
				const uint32_t data = edge_normals[axis][corner[c0]];
				const float stored_t = static_cast<float>(data & 255) / 255.f;
				if (data != 0 && std::abs(stored_t - t) < EDGE_NORMAL_T_TOLERANCE) {
					// The stored crossing is the true one; the linear estimate is off near features.
					n = decode_edge_normal(data);
					t = stored_t;
					exact = true;
				}
			}
			if (!exact) {
				n = normalized_or_up(gradient(corner[c0]) * (1.f - t) + gradient(corner[c1]) * t);
			}
			const V3 p0 = cell_origin +
					V3{ float(CORNER_OFFSETS[c0][0]), float(CORNER_OFFSETS[c0][1]), float(CORNER_OFFSETS[c0][2]) };
			const V3 p1 = cell_origin +
					V3{ float(CORNER_OFFSETS[c1][0]), float(CORNER_OFFSETS[c1][1]), float(CORNER_OFFSETS[c1][2]) };
			const V3 p = p0 + (p1 - p0) * t;
			ata.add_outer(n);
			atb = atb + n * dot(n, p);
			mass = mass + p;
			normal_sum = normal_sum + n;
			weights[corner_slots[c0]] += 1.f - t;
			weights[corner_slots[c1]] += t;
			++crossings;
		}
		mass = mass * (1.f / crossings);

		// Minimize the distance to every crossing's tangent plane, measured from the mass point, plus a little pull
		// toward the mass point itself.
		ata.m[0] += bias;
		ata.m[3] += bias;
		ata.m[5] += bias;
		const V3 rhs = atb - ata.mul(mass) + mass * bias;
		V3 position = mass + solve_pseudo_inverse(ata, rhs, sharpness);
		// Keep the vertex near its own cell, or features from far planes could fold the surface. A feature can sit just
		// outside the cell whose crossings define it, hence the margin.
		for (int axis = 0; axis < 3; ++axis) {
			const float lo = get_axis(cell_origin, axis);
			set_axis(position, axis, std::clamp(get_axis(position, axis), lo - CELL_MARGIN, lo + 1.f + CELL_MARGIN));
		}

		slot = static_cast<int>(arrays.vertices.size());
		arrays.vertices.push_back(Vector3f(position.x, position.y, position.z));
		const V3 n = normalized_or_up(normal_sum);
		arrays.normals.push_back(Vector3f(n.x, n.y, n.z));
		if (textured) {
			const float scale = 255.f / crossings;
			const uint32_t packed_weights = pack_bytes(
					static_cast<uint8_t>(std::clamp(weights[0] * scale, 0.f, 255.f)),
					static_cast<uint8_t>(std::clamp(weights[1] * scale, 0.f, 255.f)),
					static_cast<uint8_t>(std::clamp(weights[2] * scale, 0.f, 255.f)),
					static_cast<uint8_t>(std::clamp(weights[3] * scale, 0.f, 255.f))
			);
			const uint32_t packed_indices = pack_bytes(selected[0], selected[1], selected[2], selected[3]);
			float packed[2];
			memcpy(&packed[0], &packed_indices, sizeof(float));
			memcpy(&packed[1], &packed_weights, sizeof(float));
			arrays.texturing.push_back(packed[0]);
			arrays.texturing.push_back(packed[1]);
		}
		return slot;
	};

	// Each sign change along a voxel edge gets a quad joining the vertices of the 4 cells around that edge. This block
	// owns the edges starting inside it, so a quad on its border is made once, by one block.
	for (int axis = 0; axis < 3; ++axis) {
		const int b_axis = (axis + 1) % 3;
		const int c_axis = (axis + 2) % 3;
		const int step = axis == 0 ? 1 : (axis == 1 ? sx : sxy);
		for (int z = 0; z < block_size.z; ++z) {
			for (int y = 0; y < block_size.y; ++y) {
				for (int x = 0; x < block_size.x; ++x) {
					const int i0 = (x + PADDING_MIN) + sx * (y + PADDING_MIN) + sxy * (z + PADDING_MIN);
					const bool s0 = solid(i0);
					if (s0 == solid(i0 + step)) {
						continue;
					}
					int v[4];
					bool complete = true;
					// Around the edge, in the order that winds counter-clockwise about +axis.
					constexpr int AROUND[4][2] = { { -1, -1 }, { 0, -1 }, { 0, 0 }, { -1, 0 } };
					for (int k = 0; k < 4; ++k) {
						int c[3] = { x, y, z };
						c[b_axis] += AROUND[k][0];
						c[c_axis] += AROUND[k][1];
						v[k] = get_cell_vertex(c[0], c[1], c[2]);
						if (v[k] < 0) {
							complete = false;
						}
					}
					if (!complete) {
						continue;
					}
					// Godot draws clockwise faces, whose right-hand normal points into the matter. Matter at the edge's
					// start means the surface faces +axis, so the counter-clockwise ring is reversed.
					if (s0) {
						std::swap(v[1], v[3]);
					}
					// Split along the diagonal that keeps both triangles closest to the surface normals.
					auto pos = [&](int vi) {
						const Vector3f &p = arrays.vertices[vi];
						return V3{ p.x, p.y, p.z };
					};
					auto nrm = [&](int vi) {
						const Vector3f &n = arrays.normals[vi];
						return V3{ n.x, n.y, n.z };
					};
					auto score = [&](int a, int b, int c) {
						const V3 face = normalized_or_up(cross(pos(b) - pos(a), pos(c) - pos(a)) * -1.f);
						return dot(face, nrm(a)) + dot(face, nrm(b)) + dot(face, nrm(c));
					};
					const float split02 = score(v[0], v[1], v[2]) + score(v[0], v[2], v[3]);
					const float split13 = score(v[0], v[1], v[3]) + score(v[1], v[2], v[3]);
					if (split02 >= split13) {
						arrays.indices.insert(arrays.indices.end(), { v[0], v[1], v[2], v[0], v[2], v[3] });
					} else {
						arrays.indices.insert(arrays.indices.end(), { v[0], v[1], v[3], v[1], v[2], v[3] });
					}
				}
			}
		}
	}

	if (arrays.indices.empty()) {
		return;
	}

	PackedVector3Array gd_vertices;
	PackedVector3Array gd_normals;
	PackedInt32Array gd_indices;
	copy_to(gd_vertices, to_span_const(arrays.vertices));
	copy_to(gd_normals, to_span_const(arrays.normals));
	copy_to(gd_indices, to_span_const(arrays.indices));
	// Transvoxel's LOD attribute is all zeros here (no transitions), kept so the same materials work.
	PackedFloat32Array lod_data;
	lod_data.resize(arrays.vertices.size() * 4);
	memset(lod_data.ptrw(), 0, lod_data.size() * sizeof(float));

	Array gd_arrays;
	gd_arrays.resize(Mesh::ARRAY_MAX);
	gd_arrays[Mesh::ARRAY_VERTEX] = gd_vertices;
	gd_arrays[Mesh::ARRAY_NORMAL] = gd_normals;
	gd_arrays[Mesh::ARRAY_CUSTOM0] = lod_data;
	if (textured) {
		PackedFloat32Array texturing;
		texturing.resize(arrays.texturing.size());
		memcpy(texturing.ptrw(), arrays.texturing.data(), arrays.texturing.size() * sizeof(float));
		gd_arrays[Mesh::ARRAY_CUSTOM1] = texturing;
	}
	gd_arrays[Mesh::ARRAY_INDEX] = gd_indices;
	output.surfaces.push_back({ gd_arrays, 0 });
	output.primitive_type = Mesh::PRIMITIVE_TRIANGLES;
	output.mesh_flags = (RenderingServerEnums::ARRAY_CUSTOM_RGBA_FLOAT << Mesh::ARRAY_FORMAT_CUSTOM0_SHIFT);
	if (textured) {
		output.mesh_flags |= (RenderingServerEnums::ARRAY_CUSTOM_RG_FLOAT << Mesh::ARRAY_FORMAT_CUSTOM1_SHIFT);
	}
}

int VoxelMesherDualContouring::get_used_channels_mask() const {
	int mask = 1 << VoxelBuffer::CHANNEL_SDF;
	if (_texturing_mode == TEXTURES_SINGLE_S4) {
		mask |= 1 << VoxelBuffer::CHANNEL_INDICES;
	}
	if (_edge_normals_channel >= 0) {
		mask |= 7 << _edge_normals_channel;
	}
	return mask;
}

void VoxelMesherDualContouring::set_texturing_mode(TexturingMode mode) {
	_texturing_mode = mode;
	emit_changed();
}

VoxelMesherDualContouring::TexturingMode VoxelMesherDualContouring::get_texturing_mode() const {
	return _texturing_mode;
}

void VoxelMesherDualContouring::set_sharpness(float sharpness) {
	_sharpness = std::clamp(sharpness, 0.f, 1.f);
	emit_changed();
}

float VoxelMesherDualContouring::get_sharpness() const {
	return _sharpness;
}

void VoxelMesherDualContouring::set_mass_point_bias(float bias) {
	_mass_point_bias = std::max(bias, 0.f);
	emit_changed();
}

float VoxelMesherDualContouring::get_mass_point_bias() const {
	return _mass_point_bias;
}

void VoxelMesherDualContouring::set_edge_normals_channel(int channel) {
	_edge_normals_channel = std::clamp(channel, -1, static_cast<int>(VoxelBuffer::MAX_CHANNELS) - 3);
	emit_changed();
}

int VoxelMesherDualContouring::get_edge_normals_channel() const {
	return _edge_normals_channel;
}

void VoxelMesherDualContouring::_bind_methods() {
	using Self = VoxelMesherDualContouring;

	ClassDB::bind_method(D_METHOD("set_texturing_mode", "mode"), &Self::set_texturing_mode);
	ClassDB::bind_method(D_METHOD("get_texturing_mode"), &Self::get_texturing_mode);

	ClassDB::bind_method(D_METHOD("set_sharpness", "sharpness"), &Self::set_sharpness);
	ClassDB::bind_method(D_METHOD("get_sharpness"), &Self::get_sharpness);

	ClassDB::bind_method(D_METHOD("set_mass_point_bias", "bias"), &Self::set_mass_point_bias);
	ClassDB::bind_method(D_METHOD("get_mass_point_bias"), &Self::get_mass_point_bias);

	ClassDB::bind_method(D_METHOD("set_edge_normals_channel", "channel"), &Self::set_edge_normals_channel);
	ClassDB::bind_method(D_METHOD("get_edge_normals_channel"), &Self::get_edge_normals_channel);

	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "texturing_mode", PROPERTY_HINT_ENUM, "None:0,Single:2"),
			"set_texturing_mode",
			"get_texturing_mode"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "sharpness", PROPERTY_HINT_RANGE, "0.0,1.0,0.001"),
			"set_sharpness",
			"get_sharpness"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "mass_point_bias", PROPERTY_HINT_RANGE, "0.0,1.0,0.001"),
			"set_mass_point_bias",
			"get_mass_point_bias"
	);

	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "edge_normals_channel", PROPERTY_HINT_RANGE, "-1,5,1"),
			"set_edge_normals_channel",
			"get_edge_normals_channel"
	);

	BIND_ENUM_CONSTANT(TEXTURES_NONE);
	BIND_ENUM_CONSTANT(TEXTURES_SINGLE_S4);
}

} // namespace zylann::voxel
