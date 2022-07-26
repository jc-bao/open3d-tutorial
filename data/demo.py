import numpy as np 
import matplotlib.pyplot as plt 
import open3d as o3d 

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame() 
vis = o3d.visualization.Visualizer() 
vis.create_window(visible=False) 
vis.add_geometry(mesh) 
# vis.poll_events() 
# vis.update_renderer() 

color = vis.capture_screen_float_buffer(True) 
depth = vis.capture_depth_float_buffer(True) 
vis.destroy_window() 
color = np.asarray(color) 
depth = np.asarray(depth) 
plt.imshow(color) 
plt.show() 
plt.imshow(depth) 
plt.show()  