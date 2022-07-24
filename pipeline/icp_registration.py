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
"""ICP (Iterative Closest Point) registration algorithm"""

import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target, transformation):
  source_temp = copy.deepcopy(source)
  target_temp = copy.deepcopy(target)
  source_temp.paint_uniform_color([1, 0.706, 0])
  target_temp.paint_uniform_color([0, 0.651, 0.929])
  source_temp.transform(transformation)
  o3d.visualization.draw_geometries([source_temp, target_temp])


def point_to_point_icp(source, target, threshold, trans_init):
  print("Apply point-to-point ICP")
  reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
  print(reg_p2p)
  print("Transformation is:")
  print(reg_p2p.transformation, "\n")
  draw_registration_result(source, target, reg_p2p.transformation)


def point_to_plane_icp(source, target, threshold, trans_init):
  print("Apply point-to-plane ICP")
  reg_p2l = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
  print(reg_p2l)
  print("Transformation is:")
  print(reg_p2l.transformation, "\n")
  draw_registration_result(source, target, reg_p2l.transformation)


if __name__ == "__main__":
  pcd_data = o3d.data.DemoICPPointClouds()
  source = o3d.io.read_point_cloud('../data/cube/cloud/cloud7.ply')
  source = source.estimate_normals(
      search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
  target = o3d.io.read_point_cloud('../data/cube/cloud/cloud15.ply')
  target = target.estimate_normals(
      search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
  threshold = 0.02
  trans_init = np.asarray([[1,0,0,0],
                           [0,1,0,0],
                           [0,0,1,0], [0.0, 0.0, 0.0, 1.0]])
  draw_registration_result(source, target, trans_init)

  print("Initial alignment")
  evaluation = o3d.pipelines.registration.evaluate_registration(
    source, target, threshold, trans_init)
  print(evaluation, "\n")

  point_to_plane_icp(source, target, threshold, trans_init)