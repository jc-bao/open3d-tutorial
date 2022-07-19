from re import L
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from PIL import Image


def get_point_cloud(width, height, view_matrix, proj_matrix):
  # get a depth image
  # "infinite" depths will have a value close to 1
  image_arr = p.getCameraImage(
    width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
  depth = image_arr[3]

  # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
  proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
  view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
  tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

  # create a grid with pixel coordinates and depth values
  y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
  y *= -1.
  x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
  h = np.ones_like(z)

  pixels = np.stack([x, y, z, h], axis=1)
  # filter out "infinite" depths
  pixels = pixels[z < 0.99]
  pixels[:, 2] = 2 * pixels[:, 2] - 1

  # turn pixels to world coordinates
  points = np.matmul(tran_pix_world, pixels.T).T
  points /= points[:, 3: 4]
  points = points[:, :3]

  return points


def main():
  client = p.connect(p.GUI)
  p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
  p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90,
                               cameraPitch=-30,
                               cameraTargetPosition=[0, 0, 0.01])
  p.setGravity(0, 0, 0)

  visualShapeId = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    rgbaColor=[1, 1, 0],
    halfExtents=[0.1, 0.1, 0.1])
  multiBodyId = p.createMultiBody(
    baseVisualShapeIndex=visualShapeId,
    basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
  
  for i in range(60):
    theta = i * np.pi / 30  
    viewMatrix = p.computeViewMatrix(
      cameraEyePosition=[0.6*np.cos(theta), 0.6*np.sin(theta), 0],
      cameraTargetPosition=[0, 0, 0],
      cameraUpVector=[0, 0, 1])
    projectionMatrix = p.computeProjectionMatrixFOV(
      fov=45.0,
      aspect=1.0,
      nearVal=0.2,
      farVal=3.0)
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
      width=224,
      height=224,
      viewMatrix=viewMatrix,
      projectionMatrix=projectionMatrix)
    depth = Image.fromarray(np.uint8(depthImg*255), mode='L').convert('RGBA')
    depth.save(f'data/cube/depth/depth{i}.png')
    rgb = Image.fromarray(rgbImg, mode='RGBA')
    rgb.save(f'data/cube/image/rgb{i}.png')
    # plt.imshow(rgbImg)


if __name__ == '__main__':
  main()
