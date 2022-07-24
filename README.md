# open3d-tutorial

## How to use

Data collection

```bash
cd data
python bullet_collector.py
```

Dataprocessing
  
```bash
cd pipelines
python multiway_registration.py
```

## Reconstruct objects

1. make fragments (alignment)
2. register fragments (loop closure)
3. refine fragments (rough alignment)
4. integrate scene (fine alignment)

## Pipeline

|initial state|after global registration|after local registration|after graph optimization|
|-----------|-----------------------|-----------------------|-----------------------|
|![](https://tva1.sinaimg.cn/large/e6c9d24egy1h4is90p9w8j213j0u079b.jpg)|fragments|fragments|fragments|fragments|

* Initial alignment: global registration
 * Algorithm: RANSAC 
   * Process:
     * Down sample: Get 33-dimantional FPFH features
     * Prune impossible matches (based on point, edge, normal etc.)
   * Hyperparamters:
     * RANSACConvergenceCriteria
 * Algorithm: Fast global registration
* Fine alignment: local registration
 * Algorithm: ICP
   * categories: point-to-point, point-to-plane
   * difference:
     * point-to-point: directly matching points
     * point-to-plane: making sure close points are on the same plane (need to provide point cloud normal)
   * deal with outlier (noisy points): using Robust Kernel Smoothing (RKS)
* Integrate scene: pose graph optimization
 * Algorithm: GO
 * Process:
   * Rough optimization
   * Fine optimization after prune matches out of distribution

## SLAM

* odometry worker
  * cloud -> motion compensation -> ICP
* mapping worker
* loop closure worker
* dense mapping worker

### Q&A
* When doing ICP, how many previous frames are used to compare?

## Camera setup

Method1: Calulate directly

```
[
  [f_x 0 x]
  [0 f_y y]
  [0 0 1]
]
x = width/2
y = height/2
f_x = width/2/tan(fov/2)
f_y = height/2/tan(fov/2)
```

Method2: Generate from function
```
import open3d as o3d
x = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
o3d.io.write_pinhole_camera_intrinsic("test.json", x)
```

## TODO List

- [ ] Reconstruct object from camera
  - [x] Reconstruct from RGBD image (2022.07.20)
  - [ ] Reconstruct from Depth image
    - [x] Camera distortion compensations (2022.07.22)
    - [x] Direct reconstruction (2022.07.22)
    - [ ] Reconstruct with esitimation of camera position
    - [ ] Camera pose estimation 
    - [ ] Reconstruct with limited observing angle
- [ ] Reconstruct object from moving camera
- [ ] Reconstruct object from tactile sensor