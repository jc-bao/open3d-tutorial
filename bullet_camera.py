from re import L
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from PIL import Image

def main():
  client = p.connect(p.DIRECT)
  # p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
  # p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90,
  #                              cameraPitch=-30,
  #                              cameraTargetPosition=[0, 0, 0.01])
  p.setGravity(0, 0, 0)

  visualShapeId = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName='assets/curved_cube.obj',
    rgbaColor=[1,0,0,1],
    meshScale=[0.2, 0.2, 0.2])
  multiBodyId = p.createMultiBody(
    baseVisualShapeIndex=visualShapeId,
    basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

  for i in range(60):
    theta = i * np.pi / 30
    viewMatrix = p.computeViewMatrix(
      cameraEyePosition=[0.6*np.cos(theta), 0.6*np.sin(theta), 0.6],
      cameraTargetPosition=[0, 0, 0],
      cameraUpVector=[0, 0, 1])
    projectionMatrix = p.computeProjectionMatrixFOV(
      fov=45.0,
      aspect=1.0,
      nearVal=0.2,
      farVal=3.0)
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
      width=1024,
      height=1024,
      viewMatrix=viewMatrix,
      projectionMatrix=projectionMatrix)
    depth = Image.fromarray(np.uint8(depthImg*255), mode='L').convert('RGBA')
    depth.save(f'data/cube/depth/depth{i+1}.png')
    rgb = Image.fromarray(rgbImg, mode='RGBA').convert('RGB')
    rgb.save(f'data/cube/image/rgb{i+1}.jpg')


if __name__ == '__main__':
  main()
