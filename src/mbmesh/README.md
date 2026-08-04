# mbmesh

## Overview

**mbmesh** reads swath sonar data files through MB-System's datalist processing framework and reconstructs a triangle mesh from accepted multibeam echosounder (MBES) bathymetry soundings. The current implementation is a full point-cloud-to-surface pipeline: it ingests soundings, estimates dataset scale, computes oriented normals, reconstructs a signed scalar field with screened Poisson, extracts a surface with marching cubes, trims unsupported regions, and writes a binary glTF mesh.

The default output is intentionally small and predictable:

- **Final mesh GLB** - `mesh.glb`, the support-trimmed triangle mesh in binary glTF format

Optional command-line flags can also write:

- **Raw mesh GLB** - `raw_mesh.glb`, the untrimmed marching-cubes mesh
- **Local XYZ point cloud** - `pointcloud-local.xyz`, coordinates in the local Cartesian reconstruction frame
- **ECEF XYZ point cloud** - `pointcloud-ecef.xyz`, coordinates in Earth-Centered Earth-Fixed space
- **Oriented PLY point cloud** - `oriented-pointcloud-ecef.ply`, oriented samples for inspection
- **Diagnostic GLB files** - point, normal-vector, and sensor-origin visualizations
- **HTML viewer** - `mesh.html`, an X3DOM page for `mesh.glb` with an optional local server launch

## Development History

The first version of **mbmesh** was created as a California State University, Monterey Bay Capstone project in Spring 2026. That implementation focused on extracting MB-System soundings into point clouds and simple GLB/HTML visualizations.

The current implementation keeps the MB-System datalist input path and web-friendly GLB output style, but replaces the point-cloud-only workflow with a mesh reconstruction pipeline. The code is now split into smaller algorithm, I/O, math, and settings modules rather than keeping the whole tool in a single source file.

## Current Implementation

**mbmesh** implements an end-to-end bathymetry mesh reconstruction workflow:

1. **Data Ingestion**: Read an MB-System datalist and all referenced swath files using MB-System-supported formats.
2. **Sounding Filtering**: Accept valid bathymetry soundings and optionally restrict them to a geographic bounding box.
3. **Local Coordinate Framing**: Convert accepted geodetic soundings into a local Cartesian coordinate frame for stable numerical processing.
4. **Spacing Estimation**: Sample accepted points and estimate nearest-neighbor spacing statistics.
5. **Automatic Settings**: Derive reconstruction defaults from estimated spacing and requested level of detail.
6. **Point Decimation**: Optionally reduce samples according to the current decimation settings.
7. **Normal Estimation**: Estimate oriented normals using local neighborhoods around each sample.
8. **Screened Poisson Reconstruction**: Accumulate oriented samples into a signed scalar field.
9. **Marching Cubes**: Extract a raw triangle mesh from the reconstructed scalar field.
10. **Support Trimming**: Remove mesh regions that lack nearby input-sample support.
11. **Output Generation**: Write the default mesh and any requested diagnostics, point clouds, or viewer files.

### Current Features

- Support for MB-System datalist processing and swath formats supported by MB-System
- Automatic filtering of accepted bathymetry soundings
- Optional geographic bounds filtering with `-R west/east/south/north`
- Local Cartesian reconstruction frame for improved numerical stability
- Dataset spacing estimation using sampled nearest-neighbor distances
- Data-driven defaults for level of detail, normal estimation, Poisson grid scale, and support trimming
- Screened-Poisson scalar-field reconstruction
- Marching-cubes surface extraction
- Support-based trimming to reject unsupported mesh regions
- Default `mesh.glb` output with optional raw mesh, point cloud, diagnostic, and HTML outputs
- Optional X3DOM viewer generation and local Python HTTP server launch
- Metadata-only mode for inspecting input bounds and spacing before running reconstruction
- Verbose progress logging for each processing stage

## Algorithm Stages

### 1. Datalist Reading

The pipeline starts in `src/mbmesh/src/io/datalist_reader.cpp`. The reader opens an MB-System datalist, reads the referenced swath files, and collects accepted bathymetry soundings. The `-R` option applies a geographic subset filter in degrees using the form:

```text
west/east/south/north
```

The collected points include bathymetry samples and the frame information needed to write local and ECEF outputs later.

### 2. Metadata and Spacing Estimation

Before reconstruction begins, `src/mbmesh/settings.cpp` computes metadata from the accepted points:

- Number of files read
- Number of soundings read
- Number of accepted soundings
- Geodetic longitude/latitude bounds
- Local 3D bounds
- Nearest-neighbor spacing statistics

Spacing is estimated from up to 8192 sampled positions using the local KD-tree implementation. The estimated minimum, maximum, average, and median spacing are used to scale later algorithm defaults.

### 3. Level of Detail and Runtime Defaults

The `-L`, `--lod`, or `--level-of-detail` option sets the requested smallest feature size in meters. The implementation clamps this value to the supported range and combines it with the estimated point spacing to set defaults for:

- Decimation cell size
- Normal-estimation search radius and neighbor count
- Screened-Poisson grid cell size, padding, splat radius, screening weight, and iteration count
- Marching-cubes iso-value
- Support-trimming radius and normal-offset threshold

This is why most runs do not require manual tuning beyond choosing a sensible `-L` value for the dataset scale.

### 4. Point Decimation

Point decimation is handled by `src/mbmesh/src/algorithms/point_decimation.cpp`. The current spacing-driven defaults disable decimation by default, but the stage is still part of the pipeline and can retain one representative point per local cell when enabled in code.

The output of this stage is the point set used for normals, reconstruction, and optional point-cloud outputs.

### 5. Normal Estimation

Normal estimation is handled by `src/mbmesh/src/algorithms/normal_estimation.cpp`. The algorithm uses local neighborhoods to estimate oriented normals for the decimated point set. When a search radius is active, it uses radius-based neighborhoods and falls back to k-nearest neighbors when too few neighbors are found.

The resulting oriented samples are the input to screened Poisson reconstruction and can also be written to `oriented-pointcloud-ecef.ply`.

### 6. Screened Poisson Reconstruction

Screened Poisson reconstruction is handled by `src/mbmesh/src/algorithms/screened_poisson.cpp`. This stage builds a padded 3D scalar grid over the point cloud, splats oriented normals into the grid, and solves for a signed scalar field. Screening pulls the solution toward the input samples so the reconstructed surface better follows the measured bathymetry.

The grid dimensions are automatically constrained by a cell-count budget in `settings.cpp`; if the requested feature size would create too many cells, the Poisson cell size is increased until the grid fits the budget.

### 7. Marching Cubes

Marching cubes is handled by `src/mbmesh/src/algorithms/marching_cubes.cpp`. This stage extracts a triangle mesh from the scalar field using the configured iso-value. The raw result can be written with:

```bash
mbmesh -Idatalist.mb-1 --raw-mesh-glb
```

The raw mesh is useful for comparing the reconstructed surface before support trimming.

### 8. Support Trimming

Support trimming is handled by `src/mbmesh/src/algorithms/support_trimming.cpp`. It rejects mesh regions that do not have enough nearby oriented input samples or fail the configured normal-offset and alignment checks. This reduces unsupported surfaces introduced by interpolation across sparse regions.

If trimming removes the entire mesh, **mbmesh** exits with an error rather than silently writing an empty final mesh. In that case, try a larger `-L` value or inspect the raw mesh and diagnostics.

### 9. Output Writing

Output writers live in `src/mbmesh/src/io/`. The final mesh writer always writes `mesh.glb`. Other files are controlled by CLI flags. When `--html` is used, **mbmesh** writes `mesh.html`, starts a local Python server on `127.0.0.1:8000`, and opens the viewer in the default browser. Server logs are written to `/tmp/mbmesh_http.log`.

## Future Development

Useful next steps for **mbmesh** include:

- Exposing more reconstruction and trimming controls as CLI options
- Adding a preprocessing mode that writes reusable intermediate point clouds
- Improving diagnostics for datasets where support trimming rejects too much of the mesh
- Adding automated regression tests for full CLI runs on small swath datasets
- Evaluating 3D Tiles and compression outputs for large web visualization workflows
- Improving viewer controls and metadata display in the generated X3DOM page

## Source Files

### Main Application

- **mbmesh.cpp** - Main CLI entry point, pipeline orchestration, output selection, help text, and local HTML viewer launch
- **settings.cpp** - Runtime defaults, metadata computation, spacing estimation, bounds handling, and metadata output
- **settings.h** - CLI option state and metadata structures

### Algorithm Interfaces

- **include/algorithms/point_decimation.h** - Point decimation options and interface
- **include/algorithms/normal_estimation.h** - Normal estimation options and interface
- **include/algorithms/screened_poisson.h** - Screened-Poisson options and scalar-grid interface
- **include/algorithms/marching_cubes.h** - Marching-cubes options and mesh extraction interface
- **include/algorithms/support_trimming.h** - Support-trimming options, diagnostics, and interface

### Algorithm Implementations

- **src/algorithms/point_decimation.cpp** - Cell-based point decimation
- **src/algorithms/normal_estimation.cpp** - Local neighborhood normal estimation
- **src/algorithms/screened_poisson.cpp** - Scalar-field reconstruction
- **src/algorithms/marching_cubes.cpp** - Triangle extraction
- **src/algorithms/support_trimming.cpp** - Support-based mesh filtering

### Data, Math, and I/O

- **include/data_types/geometry.h** - Point, mesh, bounds, and coordinate-frame data types
- **include/data_types/swathfile.h** - Swath-file point collection types
- **include/math/*.h** - Vector, matrix, Eigen-like helper, and KD-tree utilities
- **include/io/datalist_reader.h** and **src/io/datalist_reader.cpp** - MB-System datalist and swath reading
- **include/io/glb_writer.h** and **src/io/glb_writer.cpp** - Mesh, point, normal, and origin-ray GLB writers
- **include/io/xyz_writer.h** and **src/io/xyz_writer.cpp** - Local XYZ, ECEF XYZ, and oriented PLY writers
- **include/io/x3dom_writer.h** and **src/io/x3dom_writer.cpp** - HTML/X3DOM viewer writer

### Tests

- **tests/test_marching_cubes.cpp** - Marching-cubes test program
- **tests/test_normal_estimation.cpp** - Normal-estimation test program
- **tests/test_support_trimming.cpp** - Support-trimming test program
- **tests/synthetic_data.h** and **tests/perlin_noise.h** - Synthetic test data helpers

### Build Configuration

- **CMakeLists.txt** - CMake build configuration for the `mbmesh` executable and tests
- **Makefile.am** - Autotools build configuration

## Building

### CMake

From the MB-System repository root:

```bash
cmake --build build --target mbmesh
```

If the build directory has not been configured yet:

```bash
cmake -S . -B build
cmake --build build --target mbmesh
```

The resulting executable is:

```bash
./build/src/mbmesh/mbmesh
```

### Autotools

From the MB-System repository root:

```bash
./configure
make mbmesh
```

## Usage Examples

### Basic Mesh Reconstruction

```bash
./build/src/mbmesh/mbmesh -Idatalist.mb-1
```

This reads `datalist.mb-1`, reconstructs a mesh, and writes the default output directory:

```text
output/mesh.glb
```

### Custom Output Directory

```bash
./build/src/mbmesh/mbmesh \
  -I /path/to/data.mb-1 \
  -O output/mbmesh_run \
  -V
```

This writes the final mesh to:

```text
output/mbmesh_run/mesh.glb
```

The `-V` flag enables progress logging for input reading, decimation, normal estimation, screened Poisson, marching cubes, support trimming, and output writing.

### Geographic Filtering

```bash
./build/src/mbmesh/mbmesh \
  -I multibeam/ZTopo.mb-1 \
  -R-122.305/-122.303/36.392/36.394 \
  -L 0.5 \
  -O output/ztopo_subset \
  -V
```

The `-R` format is `west/east/south/north` in degrees. The `-L 0.5` option requests a smallest feature size of 0.5 meters.

### Metadata-Only Inspection

```bash
./build/src/mbmesh/mbmesh -I multibeam/ZTopo.mb-1 --metadata
```

This reads the datalist, computes metadata and spacing estimates, prints them, and exits before normal estimation or mesh reconstruction.

### Mesh With HTML Viewer

```bash
./build/src/mbmesh/mbmesh \
  -I datalist.mb-1 \
  -O output/viewer_run \
  --html \
  -V
```

This writes `mesh.glb` and `mesh.html`, starts a local server, and opens:

```text
http://127.0.0.1:8000/mesh.html
```

The legacy single-dash form is also accepted:

```bash
./build/src/mbmesh/mbmesh -Idatalist.mb-1 -html
```

### Write Point Clouds

```bash
./build/src/mbmesh/mbmesh \
  -I datalist.mb-1 \
  -O output/pointcloud_run \
  --xyz \
  --oriented-ply
```

This writes:

```text
output/pointcloud_run/mesh.glb
output/pointcloud_run/pointcloud-local.xyz
output/pointcloud_run/pointcloud-ecef.xyz
output/pointcloud_run/oriented-pointcloud-ecef.ply
```

### Write Diagnostics and Raw Mesh

```bash
./build/src/mbmesh/mbmesh \
  -I datalist.mb-1 \
  -O output/debug_run \
  --raw-mesh-glb \
  --diagnostics \
  -V
```

This writes the final mesh plus the untrimmed mesh, diagnostic point cloud, normal-vector, and sensor-origin GLB files.

### Write Everything

```bash
./build/src/mbmesh/mbmesh \
  -I datalist.mb-1 \
  -O output/full_run \
  --all-outputs \
  -V
```

This writes every optional output currently supported.

## Command-Line Options

### Required

- `-I<datalist>`, `-I <datalist>`, `--input <datalist>` - Input MB-System datalist path

The input datalist can also be supplied as the first non-option argument if `-I` is omitted.

### Optional Processing Options

- `-O<outputdir>`, `-O <outputdir>`, `--output <outputdir>` - Output directory, default `output`
- `-R<w/e/s/n>`, `-R <w/e/s/n>`, `--bounds <w/e/s/n>` - Geographic bounds in degrees
- `-L<meters>`, `-L <meters>`, `--lod <meters>`, `--level-of-detail <meters>` - Requested smallest feature size in meters
- `--metadata`, `--info` - Print dataset metadata and exit before reconstruction
- `-V`, `--verbose` - Enable progress and diagnostic logging
- `-H`, `-h`, `--help` - Display help text and exit

### Optional Output Options

- `--html`, `-html` - Write `mesh.html`, start a local server, and open the X3DOM viewer
- `--xyz`, `-xyz` - Write both local and ECEF XYZ point clouds
- `--local-xyz`, `-local-xyz` - Write `pointcloud-local.xyz`
- `--ecef-xyz`, `-ecef-xyz` - Write `pointcloud-ecef.xyz`
- `--oriented-ply`, `-oriented-ply` - Write `oriented-pointcloud-ecef.ply`
- `--pointcloud-glb`, `-pointcloud-glb` - Write `pointcloud.glb`
- `--normal-glb`, `-normal-glb` - Write `normals.glb`
- `--origin-glb`, `-origin-glb` - Write `origins.glb`
- `--raw-mesh-glb`, `-raw-mesh-glb` - Write `raw_mesh.glb`
- `--diagnostics`, `-diagnostics` - Write `pointcloud.glb`, `normals.glb`, and `origins.glb`
- `--all-outputs`, `-all-outputs` - Write all optional outputs

## Output Files

Upon successful execution, **mbmesh** always writes:

1. **mesh.glb** - Final triangle mesh in binary glTF format
   - Support-trimmed output from the reconstructed scalar field
   - Default output for all non-metadata runs
   - Suitable for web and desktop glTF/GLB viewers

Optional outputs are written only when requested:

2. **raw_mesh.glb** - Untrimmed mesh from marching cubes
   - Written with `--raw-mesh-glb` or `--all-outputs`
   - Useful for comparing support trimming effects

3. **mesh.html** - X3DOM viewer page
   - Written with `--html` or `--all-outputs`
   - References `mesh.glb`
   - Served locally and opened in the default browser when generated

4. **pointcloud-local.xyz** - Local Cartesian point cloud
   - Written with `--local-xyz`, `--xyz`, or `--all-outputs`
   - Uses the local reconstruction frame

5. **pointcloud-ecef.xyz** - ECEF point cloud
   - Written with `--ecef-xyz`, `--xyz`, or `--all-outputs`
   - Uses Earth-Centered Earth-Fixed coordinates

6. **oriented-pointcloud-ecef.ply** - Oriented point cloud
   - Written with `--oriented-ply` or `--all-outputs`
   - Useful for inspecting estimated normals

7. **pointcloud.glb** - Diagnostic point cloud GLB
   - Written with `--pointcloud-glb`, `--diagnostics`, or `--all-outputs`

8. **normals.glb** - Diagnostic normal-vector GLB
   - Written with `--normal-glb`, `--diagnostics`, or `--all-outputs`

9. **origins.glb** - Diagnostic sensor-origin ray GLB
   - Written with `--origin-glb`, `--diagnostics`, or `--all-outputs`

## Metadata and Verbose Output

### Metadata Mode

The `--metadata` or `--info` option prints dataset information and exits before reconstruction. Current metadata output includes:

```text
mbmesh dataset metadata
input:
  files_read: N
  soundings_read: N
  accepted_soundings: N
geodetic_bounds_degrees:
  min_lon: W
  max_lon: E
  min_lat: S
  max_lat: N
  extent_lon: D
  extent_lat: D
  point_count: N
estimated_spacing_meters:
  sample_count: N
  minimum: X
  maximum: X
  average: X
  median: X
```

### Verbose Mode

The `-V` option prints progress messages for:

- Input reading
- Point decimation
- Normal estimation
- Screened Poisson reconstruction
- Marching cubes
- Support trimming
- Output writing
- Optional viewer launch warnings

Verbose output also reports important runtime settings, including Poisson cell size, padding, splat radius, solver iterations, screening weight, normal-estimation neighborhood settings, and support-trimming thresholds.

## Notes

- The mesh reconstruction pipeline is currently a single end-to-end program. It does not yet expose a separate preprocessing stage that writes reusable intermediate data.
- The default settings are data-driven. They are derived from estimated point spacing and requested level of detail, not fixed constants.
- The final `mesh.glb` is always written for successful reconstruction runs. Optional outputs must be requested explicitly.
- If support trimming rejects too much of the mesh, use `--raw-mesh-glb` and diagnostic outputs to inspect the reconstruction before trimming.
- The HTML viewer launch requires `python3` and a desktop/browser environment. If launch fails, the generated `mesh.html` and `mesh.glb` remain in the output directory.

## Copyright and License

These source files are copyright by David W. Caress, Dale N. Chayes, and the California State University Monterey Bay Capstone team. They are licensed using GPL3 as part of MB-System.

Initial implementation: Spring 2026 CSUMB Capstone Project
