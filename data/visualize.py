import open3d as o3d

def main():
  pcd = o3d.io.read_point_cloud('/juno/u/chaoyi/rl/open3d_tutorial/data/cube/fragments/fragment_000.ply')
  o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
  main()