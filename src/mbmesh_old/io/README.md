# mbmesh IO

This directory is for adapters between mbmesh core representations and files or viewer artifacts. IO code should translate data at the boundary; it should not own mesh reconstruction, normal estimation, decimation, or MB-System swath reading policy.

## Planned Inputs

- `xyz_pointcloud_reader.h`: load an existing `adjustedPointcloud.xyz` style file into `PointCloud`.
  XYZ point clouds are assumed to already be in a local metric coordinate frame.
- Future MB-System-backed sources should adapt swath records into the same core types without changing downstream algorithms.

## Planned Outputs

- `x3dom_writer.h` / `x3dom_writer.cpp`: write browser-debuggable X3DOM HTML for intermediate and final representations.
- `mesh_glb_writer.h`: write triangle meshes as GLB once `Mesh` generation is available.

## Current Experimental Driver

The localized experimental pipeline driver is `../test.cpp`. It intentionally
stays outside the MB-System build files while the new pipeline is still in
flux.

Build from the repository root:

```bash
g++ -std=c++11 -I src/mbmesh \
  src/mbmesh/test.cpp \
  src/mbmesh/io/xyz_pointcloud_reader.cpp \
  src/mbmesh/io/x3dom_writer.cpp \
  -o /tmp/mbmesh_pipeline_test
```

Run with synthetic data:

```bash
/tmp/mbmesh_pipeline_test --synthetic mbmesh_pipeline_output 20 250000
```

Run with an existing local XYZ point cloud:

```bash
/tmp/mbmesh_pipeline_test ../../../tileset/adjustedPointcloud.xyz mbmesh_pipeline_output 20 250000
```

Current pipeline stages:

1. Read `adjustedPointcloud.xyz` into `PointCloud`.
2. Estimate normals into `OrientedPointCloud`.
3. Write `pointcloud.html` and `oriented_pointcloud.html` X3DOM debug views.

After that works, mesh and field visualization can use the same IO boundary.
