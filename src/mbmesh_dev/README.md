# mbmesh development repo

## Next steps:

### Finish Core IO
- glb_writer
- swath_reader

## Sensor-Origin data
- Add sensor-origin to swath file input
- This is critical for proper normal orientation

## Build pipeline functions
- End-to-end wrapper functions
- Generate meshes from swath data with user-friendly parameters

## Integrate into src/mbmesh
- Update build and run macros to include new features
- Update documentation 

## 3D Tiling
- Implement OGC 3D tile support

## Testing and documentation
- Prepare for final deliverable and presentation

## mbmesh command

General command to run `mbmesh` with a selected datalist:

```bash
mbmesh -I <datalist.mb-1> -O mbmesh_dev/output -html -V
```

Example:

```bash
mbmesh -I multibeam/ZTopo.mb-1 -html -V
```
