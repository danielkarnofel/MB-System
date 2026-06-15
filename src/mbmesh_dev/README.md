
Build command to run test.cpp:

g++ -std=c++17 -I src/mbmesh/include \
  src/mbmesh/test.cpp \
  src/mbmesh/src/synthetic_data/heightfield.cpp \
  src/mbmesh/src/synthetic_data/pointcloud_generator.cpp \
  src/mbmesh/src/synthetic_data/sdf.cpp \
  src/mbmesh/src/synthetic_data/noise.cpp \
  src/mbmesh/src/algorithms/normal_estimation.cpp \
  src/mbmesh/src/algorithms/decimation.cpp \
  src/mbmesh/src/io/write/x3dom_writer.cpp \
  -o /tmp/mbmesh_test && /tmp/mbmesh_test