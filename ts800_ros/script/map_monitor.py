#!/usr/bin/env python 
 
import rospy 
from nav_msgs.msg import OccupancyGrid 
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.colors import LinearSegmentedColormap 
 
class OccupancyGridPlotter: 
    def __init__(self): 
        rospy.init_node('occupancy_grid_plotter', anonymous=True) 
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback) 
        self.grid_data = None 
 
        # Initialize figure and axes 
        self.fig, self.ax = plt.subplots(figsize=(15, 15)) 
        self.im = None 
 
    def map_callback(self, data): 
        self.grid_data = data 
 
    def plot_grid_map(self): 
        if self.grid_data is None: 
            rospy.loginfo("Waiting for occupancy grid data...") 
            return 
 
        width = self.grid_data.info.width 
        height = self.grid_data.info.height 
        resolution = self.grid_data.info.resolution 
        origin_x = self.grid_data.info.origin.position.x 
        origin_y = self.grid_data.info.origin.position.y 
 
        data = np.array(self.grid_data.data).reshape((height, width)) 
 
        # Define colormap with 11 segments, from red to blue 
        colors = [(1, 1, 1), (1, 0.5, 0), (1, 1, 0), (0.5, 1, 0), (0, 1, 0), (0, 0.5, 1), (0, 0, 1), 
                  (0.5, 0, 1), (1, 0, 1), (1, 0, 0.5), (0, 0, 0)] 
        cmap = LinearSegmentedColormap.from_list('custom_colormap', colors, N=11) 
 
        if self.im is None: 
            # If the image object doesn't exist yet, create it 
            self.im = self.ax.imshow(data, cmap=cmap, origin='lower', extent=[origin_x, origin_x + width * resolution, 
                                                                               origin_y, origin_y + height * resolution], 
                                      vmin=0, vmax=100)  # Set vmin and vmax to ensure proper scaling 
            cbar = self.fig.colorbar(self.im, ax=self.ax) 
            cbar.set_ticks(np.arange(0, 101, 10))  # Set colorbar ticks for 10-unit intervals 
            self.ax.set_xlabel('X') 
            self.ax.set_ylabel('Y') 
            self.ax.set_title('Occupancy Grid Map') 
            self.ax.grid(True) 
        else: 
            # If the image object exists, update the data 
            self.im.set_data(data) 
            self.im.set_extent([origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution]) 
            self.im.set_clim(vmin=0, vmax=100)  # Update the color limits 
            self.fig.canvas.draw_idle()  # Redraw the figure 
 
        plt.pause(0.01)  # Pause to allow the plot to update 
 
if __name__ == '__main__': 
    plotter = OccupancyGridPlotter() 
    rospy.loginfo("Occupancy Grid Plotter node initialized.") 
 
    rate = rospy.Rate(10)  # Update rate (10 Hz) 
 
    while not rospy.is_shutdown(): 
        plotter.plot_grid_map() 
        rate.sleep() 
    rospy.spin()