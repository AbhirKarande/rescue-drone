import sys
import math
import copy

import numpy as np

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import collections
from matplotlib.patches import Polygon

class GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, quads, env, map_size):
        # Used for the drawing
        self.quads = quads
        self.world = env
        self.fig = plt.figure()
        self.ax = plt.axes()
        x = map_size[0]
        y = map_size[1]
        self.ax.set_xlim([-1 * x, x])
        self.ax.set_xlabel('X')
        self.ax.set_ylim([-1 * y, y])
        self.ax.set_ylabel('Y')
        self.ax.set_title('Quadcopter Simulation')
        # Used to check if redrawing is required
        self.last_draw_path = np.zeros(10)
        self.last_draw_height = -1
        self.crashed = False
        self.line = None
        self.lidar_line = None
        self.polygons = {}
        self.init_plot()

    def rotation_matrix(self,angle):
        ct = math.cos(angle)
        st = math.sin(angle)
        R = np.array([[ct, -st],[st, ct]])
        return R

    def init_plot(self):
        # Draw the quad
        for key in self.quads:
            self.quads[key]['l1'], = self.ax.plot([],[],color='blue',linewidth=3,antialiased=False)
            self.quads[key]['l2'], = self.ax.plot([],[],color='red',linewidth=3,antialiased=False)
            self.quads[key]['hub'], = self.ax.plot([],[],marker='o',color='green', markersize=6,antialiased=False)
        # Draw the path
        path = self.world["path"]
        #self.line, = self.ax.plot(path[:,0], path[:,1], np.full(len(path[:,1]), 5), marker=".", linestyle="--", markersize=10, color='C7')
        self.line, = self.ax.plot(path[:,0], path[:,1], marker=".", linestyle="--", markersize=10, color='C7')

    def draw_quad(self):
        for key in self.quads:
            R = self.rotation_matrix(self.quads[key]['orientation'][0])
            L = self.quads[key]['L']
            points = np.array([ [-L,0], [L,0], [0,-L], [0,L], [0,0], [0,0] ]).T
            points = np.dot(R,points)
            points[0,:] += self.quads[key]['position'][0]
            points[1,:] += self.quads[key]['position'][1]
            self.quads[key]['l1'].set_data(points[0,0:2],points[1,0:2])
            #self.quads[key]['l1'].set_2d_properties(points[2,0:2])
            self.quads[key]['l2'].set_data(points[0,2:4],points[1,2:4])
            #self.quads[key]['l2'].set_2d_properties(points[2,2:4])
            self.quads[key]['hub'].set_data(points[0,5],points[1,5])
            #self.quads[key]['hub'].set_2d_properties(points[2,5])

    def draw_path(self, path):
        self.line.set_xdata(path[:,0])
        self.line.set_ydata(path[:,1])
        #self.line.set_3d_properties(np.full(len(path[:,1]), 5))
        self.last_draw_path = copy.deepcopy(path)

    def draw_polygon(self, loc_x, loc_y, color):
        loc = (loc_x, loc_y)
        if loc in self.polygons:
            p = self.polygons[loc]
            p.set_facecolor(color)
        else:
            x_obs = [loc_x, loc_x + 1, loc_x + 1, loc_x]
            y_obs = [loc_y, loc_y, loc_y + 1, loc_y + 1]
            verts = [list(x) for x in zip(x_obs, y_obs)]
            p = Polygon(verts, facecolor=color)
            self.ax.add_patch(p)
            self.polygons[loc] = p

    def draw_world(self, height):
        for obs in self.world["obstacles"]:
            obs_x = obs[0]
            obs_y = obs[1]
            color = 1 - obs[2] / 100.0  # as given, larger value means filled, so that needs to be black
            color = (color, color, color)  # expand to rgb
            self.draw_polygon(obs_x, obs_y, color)

        for door in self.world["doors"]:
            door_x = door[0]
            door_y = door[1]
            state = door[2]
            color = 'cyan' if state == 'closed' else 'green'
            self.draw_polygon(door_x, door_y, color)

        if 'goal' in self.world:
            goal_x = self.world['goal'][0]
            goal_y = self.world['goal'][1]
            self.draw_polygon(goal_x, goal_y, 'magenta')

        try:
            if 'lidar' in self.world and len(self.world['lidar']) > 0:
                # print(self.world['lidar'])
                if self.lidar_line is None:
                    self.lidar_line, = self.ax.plot(self.world['lidar'][:, 0], self.world['lidar'][:, 1],
                                                    marker=".",
                                                    markersize=2,
                                                    linewidth=0,
                                                    linestyle=None,
                                                    markerfacecolor='red',
                                                    markeredgecolor='red',
                                                    )
                else:
                    self.lidar_line.set_xdata(self.world['lidar'][:, 0])
                    self.lidar_line.set_ydata(self.world['lidar'][:, 1])
        except:
         pass

    def draw_crash(self):
        plt.text(0.1, 0.1, "CRASH!!!", size=50,
         ha="center", va="center",
         bbox=dict(facecolor = 'red')
         )

    def update(self):
        # Draw the quad
        self.draw_quad()

        # Draw the path if it hasnt updated
        path = self.world['path']
        if not np.array_equal(path, self.last_draw_path):
            self.draw_path(path)

        # Draw the obstacles
        current_height = self.quads[self.quads.keys()[0]]['position'][2]
        if abs(current_height - self.last_draw_height) > 0.5:
            self.draw_world(current_height)

        if self.crashed:
            self.draw_crash()

        # Update the plot
        plt.pause(0.000000000000001)
