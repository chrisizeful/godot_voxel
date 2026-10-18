#ifndef VOXEL_MESHER_DUAL_CONTOURING_H
#define VOXEL_MESHER_DUAL_CONTOURING_H

#include "../voxel_mesher.h"

namespace zylann::voxel {

// Smooth voxel mesher placing one vertex per cell where the surface's tangent planes meet (dual contouring with a
// quadratic error function). Given exact normals for the voxel edges the surface crosses (written by a generator that
// knows the shape), flat faces meeting at an angle keep a sharp edge or corner. Without them it falls back to the
// SDF's gradient, which rounds features off over about a voxel, like Transvoxel.
// It produces the same vertex attributes as VoxelMesherTransvoxel (CUSTOM0 lod data, CUSTOM1 texture data in the
// Single S4 layout), so materials written for Transvoxel work with it. It does not support LOD.
class VoxelMesherDualContouring : public VoxelMesher {
	GDCLASS(VoxelMesherDualContouring, VoxelMesher)

public:
	enum TexturingMode {
		TEXTURES_NONE = 0,
		// Each voxel has one 8-bit material index, and up to 4 blend in shader (same layout as Transvoxel's).
		TEXTURES_SINGLE_S4 = 2,
	};

	VoxelMesherDualContouring();

	void build(VoxelMesher::Output &output, const VoxelMesher::Input &input) override;

	int get_used_channels_mask() const override;
	bool supports_lod() const override {
		return false;
	}

	void set_texturing_mode(TexturingMode mode);
	TexturingMode get_texturing_mode() const;

	void set_sharpness(float sharpness);
	float get_sharpness() const;

	void set_mass_point_bias(float bias);
	float get_mass_point_bias() const;

	// Merges triangles wherever that moves the surface less than the threshold, in voxels; see the class documentation.
	void set_mesh_optimization_enabled(bool enabled);
	bool is_mesh_optimization_enabled() const;

	void set_mesh_optimization_error_threshold(float threshold);
	float get_mesh_optimization_error_threshold() const;

	// First of 3 consecutive 32-bit channels holding exact normals for the voxel edges along X, Y and Z, stored on each
	// edge's lower voxel, or -1 to always use the SDF's gradient. See the class documentation for the layout.
	void set_edge_normals_channel(int channel);
	int get_edge_normals_channel() const;

protected:
	static void _bind_methods();

private:
	TexturingMode _texturing_mode = TEXTURES_NONE;
	// Eigenvalues of the error function below this fraction of the largest one are ignored, so near-parallel planes
	// don't throw the vertex far away. Lower keeps more features but is less stable.
	float _sharpness = 0.1f;
	// Pulls the vertex slightly toward the average of the surface crossings, which keeps flat areas tidy.
	float _mass_point_bias = 0.02f;
	int _edge_normals_channel = -1;
	bool _mesh_optimization_enabled = false;
	float _mesh_optimization_error_threshold = 0.05f;
};

} // namespace zylann::voxel

VARIANT_ENUM_CAST(zylann::voxel::VoxelMesherDualContouring::TexturingMode);

#endif // VOXEL_MESHER_DUAL_CONTOURING_H
