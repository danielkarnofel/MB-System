# mbmesh

`mbmesh` reads MB-System datalists, converts accepted swath bathymetry
soundings into sensor-origin-aware point samples, reconstructs a mesh, and
writes multiple output formats.

## Pipeline

1. Read an MB-System datalist with `read_datalist(...)`.
2. Convert each valid beam into a `CollectedPoint`.
3. Preserve the sonar sensor origin for normal orientation.
4. Optionally decimate the collected points.
5. Estimate oriented normals.
6. Reconstruct a scalar field with screened Poisson.
7. Extract a mesh with marching cubes.
8. Trim unsupported mesh regions.
9. Write `mesh.glb` and an X3DOM `mesh.html` preview.

## Build

From the repository root:

```bash
cmake --build build --target mbmesh
```

The target is configured in `src/mbmesh/CMakeLists.txt` and uses the normal
MB-System `mbio` target instead of requiring a manual `g++` command.

## Run

```bash
build/src/mbmesh/mbmesh -I <datalist.mb-1> -O <output_dir> -html -V
```

Example:

```bash
build/src/mbmesh/mbmesh -I multibeam/ZTopo.mb-1 -O src/mbmesh/output -html -V
```

## Options

- `-I`, `--input`: input MB-System datalist path.
- `-O`, `--output`: output directory. Defaults to `mbmesh_output`.
- `-R`, `--bounds`: geographic bounds as `west/east/south/north`.
- `-V`, `--verbose`: increase progress output.
- `-H`, `-h`, `--help`: print usage.

## Outputs

- `mesh.glb`: binary glTF mesh output.
- `mesh.html`: optional X3DOM viewer that references `mesh.glb`.

## Next Steps

- 
