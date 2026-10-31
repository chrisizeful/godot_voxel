#ifndef VOXEL_SCULPT_DATA_H
#define VOXEL_SCULPT_DATA_H

#include "../util/containers/std_unordered_map.h"
#include "../util/containers/std_vector.h"
#include "../util/godot/classes/resource.h"
#include "../util/godot/core/packed_arrays.h"
#include "../util/math/box3i.h"
#include "../util/math/vector3i.h"
#include "../util/thread/rw_lock.h"

namespace zylann::voxel {

// Hand-sculpted changes over a volume's voxels, used by VoxelModifierSculpt: in chunks of CHUNK_SIZE cubed voxels of
// the volume's own grid, a shape added (union) and one cut away (subtraction) from whatever lies below the modifier,
// then a small offset (smoothing), and a material that replaces it (0 leaves it). As shapes rather than offsets, they
// don't depend on the values below far from its surface. Saved with the resource, one byte array per chunk,
// uncompressed.
class VoxelSculptData : public Resource {
	GDCLASS(VoxelSculptData, Resource)
public:
	static constexpr int CHUNK_SIZE_PO2 = 4;
	static constexpr int CHUNK_SIZE = 1 << CHUNK_SIZE_PO2;
	static constexpr int CHUNK_VOLUME = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;
	// Distances are stored in 1/DISTANCE_SCALE of a voxel.
	static constexpr float DISTANCE_SCALE = 1024.f;
	// A shape distance this far out is no shape.
	static constexpr int16_t NO_SHAPE = 32767;
	static constexpr int BYTES_PER_VOXEL = 7;

	struct Chunk {
		// Signed distances of the added and the cut shapes.
		StdVector<int16_t> add;
		StdVector<int16_t> cut;
		// Added to the distance after the shapes.
		StdVector<int16_t> offset;
		// Material index + 1, or 0 to leave it as it is.
		StdVector<uint8_t> material;

		bool is_empty() const;
	};

	// The distance below (`below`) with a chunk's voxel at `index` applied.
	static inline float apply(const Chunk &chunk, unsigned int index, float below) {
		const float inv_scale = 1.f / DISTANCE_SCALE;
		float sd = below;
		const int16_t add = chunk.add[index];
		if (add != NO_SHAPE) {
			sd = sd < add * inv_scale ? sd : add * inv_scale;
		}
		const int16_t cut = chunk.cut[index];
		if (cut != NO_SHAPE) {
			sd = sd > -cut * inv_scale ? sd : -cut * inv_scale;
		}
		return sd + chunk.offset[index] * inv_scale;
	}

	static inline unsigned int get_index(const Vector3i local) {
		return local.x + CHUNK_SIZE * (local.y + CHUNK_SIZE * local.z);
	}

	// Lock for reading while using get_chunk(), for writing while changing chunks.
	inline RWLock &get_lock() const {
		return _lock;
	}

	const Chunk *get_chunk(const Vector3i chunk_pos) const;
	Chunk &get_or_create_chunk(const Vector3i chunk_pos);
	void erase_chunk(const Vector3i chunk_pos);

	// Call after changing chunks (holding the write lock), so the bounds and flags follow.
	void update_bounds();

	// Voxels covered by chunks; empty when there are none.
	inline Box3i get_bounds() const {
		return _bounds;
	}

	inline bool has_material() const {
		return _has_material;
	}

	// Chunk positions to byte arrays (distances, then materials), for undo and saving; a missing chunk is an empty
	// array.
	Dictionary get_chunks_in_box(const Box3i voxel_box) const;
	void set_chunks(const Dictionary &chunks);

private:
	Dictionary _b_get_chunks_in_box(Vector3i min_voxel, Vector3i max_voxel) const;
	Dictionary _b_get_data() const;
	void _b_set_data(Dictionary data);
	int _b_get_chunk_count() const;

	static PackedByteArray encode_chunk(const Chunk &chunk);
	static bool decode_chunk(const PackedByteArray &bytes, Chunk &chunk);

	static void _bind_methods();

	StdUnorderedMap<Vector3i, Chunk> _chunks;
	Box3i _bounds;
	bool _has_material = false;
	mutable RWLock _lock;
};

} // namespace zylann::voxel

#endif // VOXEL_SCULPT_DATA_H
