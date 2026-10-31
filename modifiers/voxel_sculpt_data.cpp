#include "voxel_sculpt_data.h"
#include "../util/godot/core/array.h"
#include "../util/godot/core/dictionary.h"
#include <cstring>

namespace zylann::voxel {

bool VoxelSculptData::Chunk::is_empty() const {
	for (unsigned int i = 0; i < add.size(); ++i) {
		if (add[i] != NO_SHAPE || cut[i] != NO_SHAPE || offset[i] != 0) {
			return false;
		}
	}
	for (const uint8_t m : material) {
		if (m != 0) {
			return false;
		}
	}
	return true;
}

const VoxelSculptData::Chunk *VoxelSculptData::get_chunk(const Vector3i chunk_pos) const {
	auto it = _chunks.find(chunk_pos);
	return it == _chunks.end() ? nullptr : &it->second;
}

VoxelSculptData::Chunk &VoxelSculptData::get_or_create_chunk(const Vector3i chunk_pos) {
	Chunk &chunk = _chunks[chunk_pos];
	if (chunk.add.size() != CHUNK_VOLUME) {
		chunk.add.resize(CHUNK_VOLUME, NO_SHAPE);
		chunk.cut.resize(CHUNK_VOLUME, NO_SHAPE);
		chunk.offset.resize(CHUNK_VOLUME, 0);
		chunk.material.resize(CHUNK_VOLUME, 0);
	}
	return chunk;
}

void VoxelSculptData::erase_chunk(const Vector3i chunk_pos) {
	_chunks.erase(chunk_pos);
}

void VoxelSculptData::update_bounds() {
	_has_material = false;
	if (_chunks.empty()) {
		_bounds = Box3i();
		return;
	}
	Vector3i lo = _chunks.begin()->first;
	Vector3i hi = lo;
	for (const auto &it : _chunks) {
		lo = math::min(lo, it.first);
		hi = math::max(hi, it.first);
		if (!_has_material) {
			for (const uint8_t m : it.second.material) {
				if (m != 0) {
					_has_material = true;
					break;
				}
			}
		}
	}
	_bounds = Box3i(lo * CHUNK_SIZE, (hi - lo + Vector3i(1, 1, 1)) * CHUNK_SIZE);
}

namespace {

void write_i16(uint8_t *w, const StdVector<int16_t> &values) {
	for (unsigned int i = 0; i < values.size(); ++i) {
		const uint16_t v = static_cast<uint16_t>(values[i]);
		w[i * 2] = v & 0xff;
		w[i * 2 + 1] = v >> 8;
	}
}

void read_i16(const uint8_t *r, StdVector<int16_t> &values, unsigned int count) {
	values.resize(count);
	for (unsigned int i = 0; i < count; ++i) {
		values[i] = static_cast<int16_t>(r[i * 2] | (r[i * 2 + 1] << 8));
	}
}

} // namespace

// Added shape, cut shape and offset (16-bit little-endian each), then materials.
PackedByteArray VoxelSculptData::encode_chunk(const Chunk &chunk) {
	PackedByteArray bytes;
	bytes.resize(CHUNK_VOLUME * BYTES_PER_VOXEL);
	uint8_t *w = bytes.ptrw();
	write_i16(w, chunk.add);
	write_i16(w + CHUNK_VOLUME * 2, chunk.cut);
	write_i16(w + CHUNK_VOLUME * 4, chunk.offset);
	memcpy(w + CHUNK_VOLUME * 6, chunk.material.data(), CHUNK_VOLUME);
	return bytes;
}

bool VoxelSculptData::decode_chunk(const PackedByteArray &bytes, Chunk &chunk) {
	if (bytes.size() != CHUNK_VOLUME * BYTES_PER_VOXEL) {
		return false;
	}
	const uint8_t *r = bytes.ptr();
	read_i16(r, chunk.add, CHUNK_VOLUME);
	read_i16(r + CHUNK_VOLUME * 2, chunk.cut, CHUNK_VOLUME);
	read_i16(r + CHUNK_VOLUME * 4, chunk.offset, CHUNK_VOLUME);
	chunk.material.resize(CHUNK_VOLUME);
	memcpy(chunk.material.data(), r + CHUNK_VOLUME * 6, CHUNK_VOLUME);
	return true;
}

Dictionary VoxelSculptData::get_chunks_in_box(const Box3i voxel_box) const {
	Dictionary out;
	RWLockRead rlock(_lock);
	voxel_box.downscaled(CHUNK_SIZE).for_each_cell([this, &out](const Vector3i cpos) {
		const Chunk *chunk = get_chunk(cpos);
		out[cpos] = chunk == nullptr ? PackedByteArray() : encode_chunk(*chunk);
	});
	return out;
}

void VoxelSculptData::set_chunks(const Dictionary &chunks) {
	{
		RWLockWrite wlock(_lock);
		const Array keys = chunks.keys();
		for (int i = 0; i < keys.size(); ++i) {
			const Vector3i cpos = keys[i];
			const PackedByteArray bytes = chunks[keys[i]];
			Chunk chunk;
			if (!decode_chunk(bytes, chunk) || chunk.is_empty()) {
				_chunks.erase(cpos);
			} else {
				_chunks[cpos] = std::move(chunk);
			}
		}
		update_bounds();
	}
	emit_changed();
}

Dictionary VoxelSculptData::_b_get_chunks_in_box(Vector3i min_voxel, Vector3i max_voxel) const {
	return get_chunks_in_box(Box3i::from_min_max(min_voxel, max_voxel));
}

Dictionary VoxelSculptData::_b_get_data() const {
	Dictionary out;
	RWLockRead rlock(_lock);
	for (const auto &it : _chunks) {
		out[it.first] = encode_chunk(it.second);
	}
	return out;
}

void VoxelSculptData::_b_set_data(Dictionary data) {
	{
		RWLockWrite wlock(_lock);
		_chunks.clear();
	}
	set_chunks(data);
}

int VoxelSculptData::_b_get_chunk_count() const {
	RWLockRead rlock(_lock);
	return _chunks.size();
}

void VoxelSculptData::_bind_methods() {
	ClassDB::bind_method(
			D_METHOD("get_chunks_in_box", "min_voxel", "max_voxel"), &VoxelSculptData::_b_get_chunks_in_box
	);
	ClassDB::bind_method(D_METHOD("set_chunks", "chunks"), &VoxelSculptData::set_chunks);
	ClassDB::bind_method(D_METHOD("get_chunk_count"), &VoxelSculptData::_b_get_chunk_count);
	ClassDB::bind_method(D_METHOD("_get_data"), &VoxelSculptData::_b_get_data);
	ClassDB::bind_method(D_METHOD("_set_data", "data"), &VoxelSculptData::_b_set_data);

	ADD_PROPERTY(
			PropertyInfo(Variant::DICTIONARY, "_data", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR),
			"_set_data",
			"_get_data"
	);
}

} // namespace zylann::voxel
