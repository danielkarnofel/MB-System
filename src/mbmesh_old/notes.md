## mbmesh Test Instructions

mbm_makedatalist -I testdata/multibeam -O datalist.mb-1

cat datalist.mb-1 head-20 > datalist.mb-2
head -n 20 datalist.mb-1 > datalist.mb-2
mbmesh -I datalist.mb-2 -R -122.304246/-122.303812/36.392409/36.392784 -html -V

Full bounds:
mbmesh -I datalist.mb-1 -R -122.304680/-122.303812/36.392409/36.393160 -html -V
mbmesh -I datalist.mb-1 -R -122.304246/-122.303812/36.392409/36.392784 -html -V
-122.304246/-122.303812/36.392409/36.3927845
0.000434
-0.0003755

Data bounds:
  Longitude: -122.304680 to -122.303812
  Latitude:    36.392409 to   36.393160
  Depth:          833.64 to      850.68 meters

## Proposed Pipeline

1. parse input
- we can largely rely on existing implementation for this, unless we want to recover origin rayss

2. generate oriented point cloud (xyz)[] => (px, py, pz; nx, ny, nz)[]
- this requires a normal estimation step
- we could also attempt to recover point -> scanner rays to use as a rejection test for invalid normals

3. generate mesh (px, py, pz; nx, ny, nz)[] => (vertices, normals, indices)
- this step requires implementation of at least one mesh generation algorithm
generate output
- ideally we will be able to implement multiple strategies or make it easier to add more

a. Alpha Shapes

b. Ball Rolling

c. Poisson + Marching Cubes
- First step is convert point cloud to a scalar field
- Then convert scalar field to mesh with marching cubes

4. Output mesh to glb, X3DOM viewer, etc.

---



---

Capstone goals:
Create mesh from point cloud
Replace yy3dtiles
Implement DRACO compression
Make X3DOm the default visualization format
Apply GeoOrigin to ECEF conversion

---

mbmesh build and run instructions
Navigate to your local development directory (e.g. user/dev)
1. git clone https://github.com/danielkarnofel/MB-System.git
2. git checkout mbmesh
3. mkdir build
4. cd build
5. cmake ..
6. make mbmesh

Test files can be found in MB-System/test/utilities/testdata
Data files must be compiled into a *datalist* for use with mbmesh. This can be done with the macro *mbm_makedatalist*.
mbm_makedatalist -I test/utilities/testdata/*
mbmesh -I src/mbmesh/datalist/datalist.mb-1

mbm_makedatalist -I test/utilities/testdata/
mbmesh -I datalists/datalist.mb-11 -O tileset/mb11

---

cesium ion
mggrid w mbmesh comparison testing