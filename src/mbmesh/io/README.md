# mbmesh IO

This directory is for adapters between mbmesh core representations and files or viewer artifacts. IO code should translate data at the boundary; it should not own mesh reconstruction, normal estimation, decimation, or MB-System swath reading policy.

## Planned Inputs

- `xyz_pointcloud_reader.h`: load an existing `adjustedPointcloud.xyz` style file into `PointCloud`.
  XYZ point clouds are assumed to already be in a local metric coordinate frame.
- Future MB-System-backed sources should adapt swath records into the same core types without changing downstream algorithms.

## Planned Outputs

- `x3dom_writer.h`: write browser-debuggable X3DOM HTML for intermediate and final representations.
- `mesh_glb_writer.h`: write triangle meshes as GLB once `Mesh` generation is available.

## First Review Target

The initial alternate pipeline should be:

1. Read `adjustedPointcloud.xyz` into `PointCloud`.
2. Estimate normals into `OrientedPointCloud`.
3. Write an X3DOM vector-field HTML debug view.

After that works, mesh and field visualization can use the same IO boundary.
