
# mbmesh_dev

Build command to run the synthetic point-cloud normal/decimation test:

g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test.cpp \
  src/mbmesh_dev/src/synthetic_data/heightfield.cpp \
  src/mbmesh_dev/src/synthetic_data/pointcloud_generator.cpp \
  src/mbmesh_dev/src/synthetic_data/sdf.cpp \
  src/mbmesh_dev/src/synthetic_data/noise.cpp \
  src/mbmesh_dev/src/algorithms/normal_estimation.cpp \
  src/mbmesh_dev/src/algorithms/decimation.cpp \
  src/mbmesh_dev/src/io/write/x3dom_writer.cpp \
  -o /tmp/mbmesh_test && /tmp/mbmesh_test

Build command to run the XYZ reader test:

g++ -std=c++17 -I src/mbmesh_dev/include \
  src/mbmesh_dev/test_xyz_file_reader.cpp \
  src/mbmesh_dev/src/io/xyz_file_reader.cpp \
  -o /tmp/mbmesh_xyz_reader_test && /tmp/mbmesh_xyz_reader_test
