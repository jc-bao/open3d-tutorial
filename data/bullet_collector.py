import numpy as np
import pandas as pd
import pybullet as p
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
from PIL import Image, ImageFilter
from tqdm import tqdm
import cv2

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
  for i in tqdm(range(60)):
    theta = i * np.pi / 30
    viewMatrix = p.computeViewMatrix(
      cameraEyePosition=[dist*0.707*np.cos(theta), dist*0.707*np.sin(theta), dist*0.707],
      cameraTargetPosition=[0, 0, 0],
      cameraUpVector=[0, 0, 1])
    projectionMatrix = p.computeProjectionMatrixFOV(
      fov=2*np.arctan(0.5/dist)*180/np.pi,
      aspect=1.0,
      nearVal=0.2+dist-0.5,
      farVal=3.0+dist-0.5)
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
      width=1024,
      height=1024,
      viewMatrix=viewMatrix,
      projectionMatrix=projectionMatrix)
    # depth image
    depthImg_filter = np.uint16(depthImg*65535)
    depthImg_filter[depthImg>=65534] = 0
    depth = Image.fromarray(depthImg_filter, mode='I;16')
    depth.save(f'cube/depth/depth{i+1}.png')
    # rgb image
    rgb = Image.fromarray(rgbImg, mode='RGBA').convert('RGB')
    rgb.save(f'cube/image/rgb{i+1}.jpg')
    # point cloud
    local_points = depthImg * 2.8 + 0.2 # convert to real depth/
    img_size = local_points.shape
    xx = np.tile(np.arange(img_size[0]) / img_size[0] - 0.5, (img_size[1], 1)).transpose() * 2  # generate x for -1 to 1 (focal length=1)
    yy = np.tile(np.arange(img_size[1]) / img_size[0] - 0.5 * img_size[1] / img_size[0], (img_size[0], 1)) * 2  # generate y for -1 to 1 (focal length=1) 
    local_points /= np.sqrt(xx**2 + yy**2 + 1) # normalize
    local_points = np.stack((xx*dist*local_points, yy*dist*local_points, local_points), axis=-1) # scale x, y according to depth
    local_points = local_points[local_points[..., -1] < 3.1]
    cloud = PyntCloud(pd.DataFrame(
      data=local_points.reshape(-1, 3),
      columns=["x", "y", "z"]))
    cloud.to_file(f"cube/cloud/cloud{i+1}.ply")
    '''
    # get outline
    rgb = np.uint8(depthImg*256)
    rgb = 255 - cv2.Canny(rgb, 8, 8)
    rgb = Image.fromarray(rgb, mode='L').convert('RGB')
    rgb.save(f'data/cube/image/rgb{i+1}.jpg')
    '''


if __name__ == '__main__':
  main()