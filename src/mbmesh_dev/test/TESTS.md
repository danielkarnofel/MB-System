## mbmesh_dev test commands

All commands below assume they are run from the repository root.

### Full Pipeline Screened Poisson Test

Reads `src/mbmesh_dev/test_data/adjustedPointcloud.xyz`, decimates the input point cloud,
estimates normals, runs the screened Poisson mesh generation pipeline, and writes X3DOM previews.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_main.cpp \
  src/mbmesh_dev/src/algorithms/decimation.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/grid_2D.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/grid_3D.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/screened_poisson.cpp \
  src/mbmesh_dev/src/algorithms/normal_estimation.cpp \
  src/mbmesh_dev/src/io/x3dom_writer.cpp \
  src/mbmesh_dev/src/io/xyz_reader.cpp \
  -o /tmp/mbmesh_test_main && /tmp/mbmesh_test_main
```

Writes:

- `src/mbmesh_dev/output/adjusted_poisson_decimated_points.html`
- `src/mbmesh_dev/output/adjusted_poisson_oriented_points.html`
- `src/mbmesh_dev/output/adjusted_poisson_mesh.html`

### Grid Mesh Demo

Generates a perturbed terrain surface with a half-torus arch, writes point-cloud previews, compares Grid2D meshing with decimated Grid3D marching cubes, and writes X3DOM HTML outputs.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_grid_mesh.cpp \
  src/mbmesh_dev/src/algorithms/decimation.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/grid_2D.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/grid_3D.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/screened_poisson.cpp \
  src/mbmesh_dev/src/algorithms/normal_estimation.cpp \
  src/mbmesh_dev/src/synthetic_data/datasets.cpp \
  src/mbmesh_dev/src/synthetic_data/noise.cpp \
  src/mbmesh_dev/src/io/x3dom_writer.cpp \
  -o /tmp/mbmesh_test_grid_mesh && /tmp/mbmesh_test_grid_mesh
```

Writes:

- `src/mbmesh_dev/output/synthetic_terrain_arch_pointcloud.html`
- `src/mbmesh_dev/output/synthetic_terrain_arch_grid_2D_mesh.html`
- `src/mbmesh_dev/output/synthetic_terrain_arch_decimated_pointcloud.html`
- `src/mbmesh_dev/output/synthetic_terrain_arch_grid_3D_mesh.html`

### Marching Cubes Sphere SDF Test

Builds a pure sphere signed-distance field, extracts the zero surface with marching cubes, validates mesh structure/radius/normals, and writes an X3DOM mesh preview.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/marching_cubes.cpp \
  src/mbmesh_dev/src/io/x3dom_writer.cpp \
  -o /tmp/mbmesh_test_marching_cubes && /tmp/mbmesh_test_marching_cubes
```

Writes:

- `src/mbmesh_dev/output/sphere_sdf_marching_cubes.html`

### Screened Poisson Scalar-Field Test

Runs `screened_poisson(...)` directly on a small oriented plane and validates scalar-field properties without extracting a mesh.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_screened_poisson.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/screened_poisson.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/normal_estimation.cpp \
  -o /tmp/mbmesh_test_screened_poisson && /tmp/mbmesh_test_screened_poisson
```

### Poisson Mesh Hemisphere Test

Generates an oriented hemisphere with known positive-Z normals, runs the full Poisson-to-marching-cubes mesh path, and writes X3DOM previews for the oriented points and reconstructed mesh.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_poisson_mesh.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/screened_poisson.cpp \
  src/mbmesh_dev/src/algorithms/normal_estimation.cpp \
  src/mbmesh_dev/src/io/x3dom_writer.cpp \
  -o /tmp/mbmesh_test_poisson_mesh && /tmp/mbmesh_test_poisson_mesh
```

Writes:

- `src/mbmesh_dev/output/poisson_hemisphere_oriented_points.html`
- `src/mbmesh_dev/output/poisson_hemisphere_mesh.html`

### Synthetic Terrain Arch Poisson Normal Comparison

Generates the perturbed terrain surface with a half-torus arch, runs screened Poisson once with exact synthetic normals and once from pure points using PCA normals flipped to positive Z. This highlights where positive-Z normal orientation breaks down without sensor-origin data, especially around overhang and underside geometry.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_synthetic_poisson_normals.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/screened_poisson.cpp \
  src/mbmesh_dev/src/algorithms/normal_estimation.cpp \
  src/mbmesh_dev/src/synthetic_data/noise.cpp \
  src/mbmesh_dev/src/io/x3dom_writer.cpp \
  -o /tmp/mbmesh_test_synthetic_poisson_normals && /tmp/mbmesh_test_synthetic_poisson_normals
```

Writes:

- `src/mbmesh_dev/output/synthetic_terrain_arch_poisson_exact_normals.html`
- `src/mbmesh_dev/output/synthetic_terrain_arch_poisson_pca_positive_z.html`

### XYZ Reader Test

Checks XYZ parsing for whitespace-separated input, comma-separated input, comments, convenience loading, and malformed-line errors.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_xyz_reader.cpp \
  src/mbmesh_dev/src/io/xyz_reader.cpp \
  -o /tmp/mbmesh_test_xyz_reader && /tmp/mbmesh_test_xyz_reader
```

### XYZ Writer Round-Trip Test

Writes a small point cloud to XYZ, reads it back, compares coordinates, and verifies invalid output is rejected with useful errors.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_xyz_writer.cpp \
  src/mbmesh_dev/src/io/xyz_writer.cpp \
  src/mbmesh_dev/src/io/xyz_reader.cpp \
  -o /tmp/mbmesh_test_xyz_writer && /tmp/mbmesh_test_xyz_writer
```

### GLB Mesh Writer Test

Writes a simple triangle mesh to binary glTF (`.glb`), validates the GLB header/chunks, checks the JSON describes mesh attributes, and verifies invalid mesh indices are rejected.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include -I src/mbgrd2gltf \
  src/mbmesh_dev/test/test_glb_writer.cpp \
  src/mbmesh_dev/src/io/glb_writer.cpp \
  -o /tmp/mbmesh_test_glb_writer && /tmp/mbmesh_test_glb_writer
```

### Swath Reader Conversion Test

Reads a tiny swath-style text fixture, converts soundings into `CollectedPointCloud` records with sensor origins preserved, filters flagged/out-of-bounds beams, and verifies malformed input is rejected.

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test/test_swath_reader.cpp \
  src/mbmesh_dev/src/io/swath_reader.cpp \
  -o /tmp/mbmesh_test_swath_reader && /tmp/mbmesh_test_swath_reader
```
