# mbmesh core

This directory holds reusable data representations and math helpers for the experimental mbmesh pipeline. Code here should not depend on MB-System file readers, rendering/export code, or any specific reconstruction algorithm.

## Current Contents

- `vec3.h`: small 3D vector type and helpers.
- `mat3.h`: small 3x3 matrix helper used by PCA normal estimation.
- `kd_tree.h`: spatial neighbor search helper.
- `geometry_types.h`: shared pipeline representations:
  - `PointCloud`
  - `OrientedPointCloud`
  - `ScalarField`
  - `Mesh`
  - `CoordinateFrame`

## Intended Boundaries

Core types are the data passed between pipeline stages. Algorithm headers should consume and produce these types, while IO and visualization code should adapt these types to files such as XYZ, GLB, or X3DOM HTML.

## Next Steps

1. Add `io/xyz_pointcloud_reader.h` to load `adjustedPointcloud.xyz` into `PointCloud`.
2. Add an X3DOM writer for `PointCloud` and `OrientedPointCloud`.
3. Move normal estimation into an `algorithms/` directory once the first alternate pipeline driver exists.
4. Add mesh generation implementations behind the `MeshGenerationMethod` choices in `mesh_generation.h`.
