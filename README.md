# open3d-tutorial

## Reconstruct objects

1. make fragments (alignment)
2. register fragments (loop closure)
3. refine fragments (rough alignment)
4. integrate scene (fine alignment)

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
    - [ ] Direct reconstruction
    - [ ] Reconstruct with esitimation of camera position
- [ ] Reconstruct object from tactile sensor