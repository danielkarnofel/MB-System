# mbmesh development repo

## Next steps:

Implement: 
- point_decimation.cpp
- support_trimming.cpp
- synthetic_data.h

Write small, independent unit tests for:
- marching_cubes.cpp
- normal_estimation.cpp
- screened_poisson.cpp
- datalist_reader.cpp
- glb_writer.cpp

Write end-to-end test using: 
- Real data: multibeam/ZTopo.mb-1
- Synthetic data

Add CLI compatibility

Update README.md with full documentation


## Run command for mbmesh_dev pipeline:

From MB-System/ directory:

g++ -std=c++17 -O2 -DCMAKE_BUILD_SYSTEM \
  -I src/mbmesh_dev/include -I src/mbio -I src/mbgrd2gltf -I /usr/include/tirpc \
  src/mbmesh_dev/mbmesh.cpp \
  src/mbmesh_dev/src/io/datalist_reader.cpp \
  src/mbmesh_dev/src/algorithms/normal_estimation.cpp \
  src/mbmesh_dev/src/algorithms/screened_poisson.cpp \
  src/mbmesh_dev/src/algorithms/marching_cubes.cpp \
  src/mbmesh_dev/src/algorithms/support_trimming.cpp \
  src/mbmesh_dev/src/io/glb_writer.cpp \
  src/mbmesh_dev/src/io/x3dom_writer.cpp \
  -L build/src/mbio -Wl,-rpath,/home/dan/dev/MB-System/build/src/mbio \
  -lmbio -lproj -lnetcdf -ltirpc -lpthread \
  -o /tmp/mbmesh_dev_bin && \
/tmp/mbmesh_dev_bin multibeam/ZTopo.mb-1