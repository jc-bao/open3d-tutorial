import numpy as np
import pandas as pd
import pybullet as p
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
from PIL import Image, ImageFilter
from tqdm import tqdm
import copy
import cv2
import open3d as o3d

def main():
  client = p.connect(p.DIRECT)
  # p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
  # p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90,
  #                              cameraPitch=-30,
  #                              cameraTargetPosition=[0, 0, 0.01])
  p.setGravity(0, 0, 0)

  visualShapeId = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName='../assets/curved_cube.obj',
    rgbaColor=[1,0,0,1],
    meshScale=[0.2, 0.2, 0.2])
  multiBodyId = p.createMultiBody(
    baseVisualShapeIndex=visualShapeId,
    basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

  dist = 0.5
  width = 64
  height = 1024
  pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, height/2, height/2, width/2, height/2) 
  for i in tqdm(range(60)):
    theta = i * np.pi / 30
    viewMatrix = p.computeViewMatrix(
      cameraEyePosition=[dist*0.707*np.cos(theta), dist*0.707*np.sin(theta), dist*0.707],
      cameraTargetPosition=[0, 0, 0],
      cameraUpVector=[0, 0, 1])
    cam2worldTrans = np.array([
      [-np.sin(theta), 0.707*np.cos(theta), -0.707*np.cos(theta), dist*0.707*np.cos(theta)],
      [np.cos(theta),0.707*np.sin(theta) , -0.707*np.sin(theta), dist*0.707*np.sin(theta)],
      [0, -0.707, -0.707, dist*0.707],
      [0, 0, 0, 1]])
    far, near = 1.0, 0.2
    projectionMatrix = p.computeProjectionMatrixFOV(
      fov=2*np.arctan(0.5/dist)*180/np.pi,
      aspect=width/height,
      nearVal=near+dist-0.5,
      farVal=far+dist-0.5)
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
      width=width, #1024,
      height=height,
      viewMatrix=viewMatrix,
      projectionMatrix=projectionMatrix)
    # process image with o3d
    rgbImg = Image.fromarray(rgbImg, mode='RGBA').convert('RGB')
    color = o3d.geometry.Image(np.array(rgbImg))
    depth = far * near / (far - (far - near) * depthImg)
    depthImg = depth/far
    depthImg[depthImg>0.98] = 0
    depth = o3d.geometry.Image((depthImg*255).astype(np.uint8))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
      color, depth, depth_scale=far, depth_trunc=1000, convert_rgb_to_intensity = False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    # save the files
    o3d.io.write_point_cloud(f'cube_narrow/cloud/cloud{i}.ply', pcd.transform(cam2worldTrans))
    o3d.io.write_image(f'cube_narrow/image/rgb{i}.jpg', color)
    o3d.io.write_image(f'cube_narrow/depth/depth{i}.png', depth)

if __name__ == '__main__':
  main()