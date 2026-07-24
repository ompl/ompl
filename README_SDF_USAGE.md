# Workspace Signed Distance Field (`ompl::sdf`)

A small, environment-agnostic **signed distance field (SDF) cache**. You give it
a *source* of signed distances for your environment — a function you write, a
**point-cloud file**, or a **mesh file** — and `GridSDF` bakes it: it samples the
source once at every grid node, stores **value + gradient**, and thereafter
serves O(1) trilinear-interpolation queries without touching the source again.
The source is evaluated *during* the bake (e.g. the point-cloud nearest-point
query or the mesh distance query runs per node), not supplied precomputed — so
`GridSDF` is a precomputed lookup table (a cache), and the adapters below are the
sources that fill it. Intended as the workspace primitive for CBF-based steering
(barrier `h(q) = d(FK_i(q)) - r_i`), but it has no dependency on planners,
robots, or any specific collision library.

- **Header:** `#include <ompl/sdf/GridSDF.h>` (header-only)
- **Namespace:** `ompl::sdf`
- **Depends on:** Eigen only.

## The interface

The only thing the SDF needs from an environment is one callback:

```cpp
namespace ompl::sdf {
    // signed distance from a workspace point to the nearest obstacle surface
    //   < 0 inside an obstacle, = 0 on the surface, > 0 in free space
    using DistanceFn = std::function<double(const Eigen::Vector3d &)>;

    struct ValueGradient {
        double          value;      // signed distance d(p)
        Eigen::Vector3d gradient;   // ∇d(p): unit vector pointing AWAY from the nearest obstacle
    };
}
```

Requirements on your `DistanceFn`:
- Input point is in the **same frame and units** (metres) as the grid `bounds`.
- Return a **true signed distance** (or a close approximation). It should be
  roughly 1-Lipschitz (`‖∇d‖ ≈ 1`), which any real distance metric satisfies —
  this is what makes the returned gradient come out unit-norm.
- It is called once per grid node at construction, then never again, so it may
  be moderately expensive.

There is deliberately **no `Environment` base class**: any obstacle source plugs
in as a lambda.

```cpp
// analytic scene
auto env = [](const Eigen::Vector3d& p){ return std::min(sdSphere(p,...), sdBox(p,...)); };
// a mesh / FCL signed-distance query
auto env = [&mesh](const Eigen::Vector3d& p){ return mesh.signedDistance(p); };
// a PyBullet / physics scene distance query, another distance field, etc.
```

> Note: the source must report a real **distance**. A checker that only reports
> collision *booleans* (e.g. VAMP) does not fit directly.

## Quick start: from a point-cloud file

If your environment is a point cloud, plug in the file and get the SDF back —
no lambda to write:

```cpp
#include <ompl/sdf/PointCloud.h>
using namespace ompl::sdf;

// one call: load a .ply / .xyz, bake over the cloud's (padded) bounding box
GridSDF grid = sdfFromFile("scene.ply", /*voxel=*/0.025, /*radius=*/0.03);

// or keep the cloud around (inspect it, choose bounds, re-bake):
PointCloud cloud = PointCloud::load("scene.ply", /*radius=*/0.03);
Eigen::AlignedBox3d box = cloud.aabb();
GridSDF grid2 = cloud.bake(box, 0.025);
```

- **Formats:** `.ply` (ascii or `binary_little_endian`; reads the `x/y/z` vertex
  properties — works for point clouds *and* mesh files) and whitespace
  `.xyz/.txt/.pts` (first three columns).
- **`radius`** inflates every point into a small sphere: distance = (distance to
  nearest cloud point) − radius. Set it to your sensor spacing / desired
  obstacle thickness.
- **Sign caveat:** a point cloud only samples obstacle *surfaces*, so this field
  is negative only within `radius` of a measured point — a thin shell — not
  throughout a solid interior. That's the correct behavior for keeping a robot
  `radius` clear of observed points (CBF clearance); it is **not** a watertight
  inside/outside field (points alone can't give that without normals/meshing).
- Nearest-point queries use an internal uniform spatial hash, so baking a full
  grid is fast even for large clouds.

## From a mesh file (FCL)

For triangle meshes (OBJ/STL/PLY/DAE/…) use the FCL adapter. Unlike a point
cloud, a watertight mesh yields a *filled* signed field (negative throughout the
solid interior), because the sign comes from an inside/outside test:

```cpp
#include <ompl/sdf/FclSDF.h>
using namespace ompl::sdf;

// one call: load a mesh, bake over its (padded) bounding box
GridSDF grid = sdfFromMeshFile("scene.stl", /*voxel=*/0.03);

// or accumulate several meshes (each with an optional placement) and bake:
MeshField env;
env.addMesh("table.obj");
env.addMesh("robot_cell.stl", someIsometry3d);
GridSDF grid2 = env.bake(env.aabb(), 0.03);
```

- **Distance magnitude** comes from FCL's exact mesh-to-mesh distance query;
  **sign** from a ray-parity (odd/even crossings) point-in-mesh test over the
  loaded triangles. So closed meshes give a true filled SDF.
- **Requires FCL (>= 0.6) and Assimp**; compile/link with
  `` `pkg-config --cflags --libs fcl assimp` `` and build as C++17. This header
  is opt-in — the rest of the module has no FCL/Assimp dependency.
- **Cost:** the inside test is currently O(triangles) per sample, so baking a
  fine grid over a large mesh is the slow part (seconds). Fine for a one-time
  bake; ask if you want it accelerated (triangle bucketing / multi-ray voting).
- **Notes:** the mesh should be watertight for the interior sign to be
  meaningful; non-closed meshes give distance-to-surface with an unreliable sign.
  A tiny tetrahedron probe is used because FCL's primitive-vs-mesh distance is
  unreliable in some builds, whereas mesh-vs-mesh is solid.

## Building and querying

```cpp
#include <ompl/sdf/GridSDF.h>
using namespace ompl::sdf;

Eigen::AlignedBox3d bounds(Eigen::Vector3d(-1,-1,-1), Eigen::Vector3d(1,1,1));
GridSDF grid(env, bounds, /*voxel=*/0.025);   // bake once

double          d  = grid.distance(p);          // signed distance
Eigen::Vector3d g  = grid.gradient(p);          // ∇d
ValueGradient   vg = grid.valueAndGradient(p);  // both together (use this in a loop)
```

After construction the grid holds **no reference** to `env` — it is a
self-contained cache, safe to keep after the environment object is gone.

### Accessors

```cpp
bool                        grid.inBounds(p);      // is p inside the baked box?
const Eigen::AlignedBox3d&  grid.bounds();
Eigen::Vector3i             grid.dimensions();     // node counts per axis
const Eigen::Vector3d&      grid.spacing();        // realized voxel size per axis
```

## Things to know

- **Interior distances** are whatever your `DistanceFn` returns. A real SDF gives
  true negative depth inside obstacles; the cache preserves it.
- **Gradient** is stored per node (central differences over the baked values) and
  interpolated, so the field is C⁰ continuous — good for a CBF-QP. It is *not*
  the exact analytic derivative of the interpolated value (value and gradient are
  interpolated independently); in practice both track the true field.
- **Out of bounds:** queries outside `bounds` clamp to the nearest boundary node
  (so clearance is *over*-reported outside). Size the box to enclose the whole
  reachable workspace, or guard with `inBounds()`.
- **Discretization error** ≈ ⅓ of a voxel worst case (halve the voxel → ~4× less).
  For safety-critical use (CBF), inflate obstacle/robot radii by this margin so a
  cached value can never falsely report "safe".
- **Static bake:** the field is fixed at construction; a moving obstacle needs a
  re-bake.
- **Singularities:** exactly at an obstacle center or medial axis the true
  gradient is undefined; expect a near-zero / direction-ambiguous gradient there.

## In this repo

- Cache:       `src/ompl/sdf/GridSDF.h`     (`GridSDF`, `DistanceFn`, `ValueGradient`)
- Point cloud: `src/ompl/sdf/PointCloud.h`  (`PointCloud`, `sdfFromFile`)
- Mesh / FCL:  `src/ompl/sdf/FclSDF.h`      (`MeshField`, `sdfFromMeshFile`; needs FCL + Assimp)
- Test:        `tests/sdf/test_sdf.cpp`      → target `test_sdf`
- Demo:        `demos/SDFDemo.cpp`           → target `demo_SDF`

```sh
cmake -S . -B build -DOMPL_BUILD_DEMOS=ON
cmake --build build --target demo_SDF
./build/demos/demo_SDF
```
