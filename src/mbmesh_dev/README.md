
Build command to run the mbmesh_dev XYZ Grid2D demo:

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/grid_2D.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/grid_3D.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/mesh_generation/screened_poisson.cpp \
  src/mbmesh_dev/src/io/x3dom_writer.cpp \
  src/mbmesh_dev/src/io/xyz_reader.cpp \
  -o /tmp/mbmesh_test && /tmp/mbmesh_test
```

The demo reads `src/mbmesh_dev/test_data/adjustedPointcloud.xyz` and writes
`src/mbmesh_dev/output/adjusted_pointcloud.html` and
`src/mbmesh_dev/output/adjusted_grid_2D_mesh.html`.

Build command to run the XYZ writer round-trip test:

```bash
g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test_xyz_writer.cpp \
  src/mbmesh_dev/src/io/xyz_writer.cpp \
  src/mbmesh_dev/src/io/xyz_reader.cpp \
  -o /tmp/mbmesh_xyz_writer_test && /tmp/mbmesh_xyz_writer_test
```

General command to run `mbmesh` with a selected datalist:

```bash
mbmesh -I <datalist.mb-1> -O mbmesh_dev/output -html -V
```

To run mbmesh with test data: mbmesh -I testdata/multibeam/ZTopo.mb-1 -html -V
