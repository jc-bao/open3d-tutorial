# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------
"""Align multiple pieces of geometry in a global space"""

import open3d as o3d
import numpy as np
import copy
import PIL
import skvideo.io
from tqdm import tqdm


def load_point_clouds(voxel_size=0.0):
  pcds = []
  for i in tqdm(range(60)):
    path = f'../data/cube/cloud/cloud{i}.ply'
    pcd = o3d.io.read_point_cloud(path)
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    pcd_down.estimate_normals(
      search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcds.append(pcd_down)
  return pcds


def pairwise_registration(source, target, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
  print("Apply point-to-plane ICP")
  icp_coarse = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance_coarse, np.identity(4),
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
  icp_fine = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance_fine,
    icp_coarse.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
  transformation_icp = icp_fine.transformation
  information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
    source, target, max_correspondence_distance_fine,
    icp_fine.transformation)
  return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
  pose_graph = o3d.pipelines.registration.PoseGraph()
  odometry = np.identity(4)
  pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
  n_pcds = len(pcds)
  matching_num = 5 # matching number for each graph
  # create render window
  vis = o3d.visualization.Visualizer()
  vis.create_window(width=256,height=256,visible=False)
  colors = []
  for source_id in tqdm(range(n_pcds)):
    for target_id in range((source_id - matching_num), source_id):
      target_id %= n_pcds
      transformation_icp, information_icp = pairwise_registration(
        pcds[source_id], pcds[target_id],
        max_correspondence_distance_coarse,
        max_correspondence_distance_fine)
      print("Build o3d.pipelines.registration.PoseGraph")
      if target_id == source_id - 1:  # odometry case
        odometry = np.dot(transformation_icp, odometry)
        pose_graph.nodes.append(
          o3d.pipelines.registration.PoseGraphNode(
            np.linalg.inv(odometry)))
        pose_graph.edges.append(
          o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=False))
      else:  # loop closure case
        pose_graph.edges.append(
          o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=True))
    vis.add_geometry(copy.deepcopy(pcds[source_id]).transform(pose_graph.nodes[source_id].pose))
    vis.get_view_control().set_lookat(np.array([0,0,0]))
    theta = 2*np.pi/n_pcds*source_id + np.pi/4
    vis.get_view_control().set_front(np.array([1.4*np.cos(theta),1.4*np.sin(theta),1]))
    vis.get_view_control().set_up(np.array([0,0,1]))
    vis.get_view_control().set_zoom(1.5)
    color = vis.capture_screen_float_buffer(do_render=True)
    color = (np.asarray(color)*255).astype(np.uint8) 
    colors.append(color)
    # PIL.Image.fromarray(color, mode='RGB').save(f'image/rgb{source_id}.jpg')
    # o3d.io.write_image(f'image/rgb{source_id}.jpg', color)
  imgs = [PIL.Image.fromarray(img) for img in colors]
  imgs[0].save("image/array.gif", save_all=True, append_images=imgs[1:], duration=50, loop=0)
  skvideo.io.vwrite('image/render.mp4', np.array(colors))
  vis.destroy_window()
  vis.close()
  return pose_graph


if __name__ == "__main__":
  print('Load point clouds ...')
  voxel_size = 0.003
  pcds_down = load_point_clouds(voxel_size)
  # o3d.visualization.draw_geometries(pcds_down, point_show_normal=True)

  print("Full registration ...")
  max_correspondence_distance_coarse = voxel_size * 15
  max_correspondence_distance_fine = voxel_size * 1.5
  # with o3d.utility.VerbosityContextManager(
  #     o3d.utility.VerbosityLevel.Debug) as cm:
  pose_graph = full_registration(pcds_down,
                                  max_correspondence_distance_coarse,
                                  max_correspondence_distance_fine)

  print("Optimizing PoseGraph ...")
  option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=max_correspondence_distance_fine,
    edge_prune_threshold=0.25,
    reference_node=0)
  with o3d.utility.VerbosityContextManager(
      o3d.utility.VerbosityLevel.Debug) as cm:
    o3d.pipelines.registration.global_optimization(
      pose_graph,
      o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
      o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
      option)

  print("Transform points and display")
  for point_id in range(len(pcds_down)):
    print(pose_graph.nodes[point_id].pose)
    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
  o3d.visualization.draw_geometries(pcds_down)
