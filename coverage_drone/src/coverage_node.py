#!/usr/bin/env python3
"""
Grid based sweep planner

author: Atsushi Sakai
"""
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math
from enum import IntEnum
import numpy as np
from angle import rot_mat_2d
from grid_map_lib import GridMap
import subprocess



class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self,
                 moving_direction, sweep_direction, x_inds_goal_y, goal_y):
        self.moving_direction = moving_direction
        self.sweep_direction = sweep_direction
        self.turing_window = []
        self.update_turning_window()
        self.x_indexes_goal_y = x_inds_goal_y
        self.goal_y = goal_y

    def move_target_grid(self, c_x_index, c_y_index, grid_map):
        n_x_index = self.moving_direction + c_x_index
        n_y_index = c_y_index

        # found safe grid
        if not grid_map.check_occupied_from_xy_index(n_x_index, n_y_index,occupied_val=0.5):
            return n_x_index, n_y_index
        else:  # occupied
            next_c_x_index, next_c_y_index = self.find_safe_turning_grid(
                c_x_index, c_y_index, grid_map)
            if (next_c_x_index is None) and (next_c_y_index is None):
                # moving backward
                next_c_x_index = -self.moving_direction + c_x_index
                next_c_y_index = c_y_index
                if grid_map.check_occupied_from_xy_index(next_c_x_index,next_c_y_index):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:
                # keep moving until end
                while not grid_map.check_occupied_from_xy_index(next_c_x_index + self.moving_direction,next_c_y_index, occupied_val=0.5):
                    next_c_x_index += self.moving_direction
                self.swap_moving_direction()
            return next_c_x_index, next_c_y_index

    def find_safe_turning_grid(self, c_x_index, c_y_index, grid_map):

        for (d_x_ind, d_y_ind) in self.turing_window:

            next_x_ind = d_x_ind + c_x_index
            next_y_ind = d_y_ind + c_y_index

            # found safe grid
            if not grid_map.check_occupied_from_xy_index(next_x_ind,next_y_ind,occupied_val=0.5):
                return next_x_ind, next_y_ind

        return None, None

    def is_search_done(self, grid_map):
        for ix in self.x_indexes_goal_y:
            if not grid_map.check_occupied_from_xy_index(ix, self.goal_y,occupied_val=0.5):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self):
        self.turing_window = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()

    def search_start_grid(self, grid_map):
        x_inds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            x_inds, y_ind = search_free_grid_index_at_edge_y(                grid_map, from_upper=True)
        elif self.sweep_direction == self.SweepDirection.UP:
            x_inds, y_ind = search_free_grid_index_at_edge_y(                grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(x_inds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(x_inds), y_ind

        raise ValueError("self.moving direction is invalid ")


def find_sweep_direction_and_start_position(ox, oy):
    # find sweep_direction
    max_dist = 0.0
    vec = [0.0, 0.0]
    sweep_start_pos = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i]
        dy = oy[i + 1] - oy[i]
        d = np.hypot(dx, dy)

        if d > max_dist:
            max_dist = d
            vec = [dx, dy]
            sweep_start_pos = [ox[i], oy[i]]

    return vec, sweep_start_pos


def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_position):
    tx = [ix - sweep_start_position[0] for ix in ox]
    ty = [iy - sweep_start_position[1] for iy in oy]
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    converted_xy = np.stack([tx, ty]).T @ rot_mat_2d(th)

    return converted_xy[:, 0], converted_xy[:, 1]


def convert_global_coordinate(x, y, sweep_vec, sweep_start_position):
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    converted_xy = np.stack([x, y]).T @ rot_mat_2d(-th)
    rx = [ix + sweep_start_position[0] for ix in converted_xy[:, 0]]
    ry = [iy + sweep_start_position[1] for iy in converted_xy[:, 1]]
    return rx, ry


def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
    y_index = None
    x_indexes = []

    if from_upper:
        x_range = range(grid_map.height)[::-1]
        y_range = range(grid_map.width)[::-1]
    else:
        x_range = range(grid_map.height)
        y_range = range(grid_map.width)

    for iy in x_range:
        for ix in y_range:
            if not grid_map.check_occupied_from_xy_index(ix, iy):
                y_index = iy
                x_indexes.append(ix)
        if y_index:
            break

    return x_indexes, y_index


def setup_grid_map(ox, oy, resolution, sweep_direction, offset_grid=10):
    width = math.ceil((max(ox) - min(ox)) / resolution) + offset_grid
    height = math.ceil((max(oy) - min(oy)) / resolution) + offset_grid
    center_x = (np.max(ox) + np.min(ox)) / 2.0
    center_y = (np.max(oy) + np.min(oy)) / 2.0

    grid_map = GridMap(width, height, resolution, center_x, center_y)
    grid_map.print_grid_map_info()
    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)
    grid_map.expand_grid()

    x_inds_goal_y = []
    goal_y = 0
    if sweep_direction == SweepSearcher.SweepDirection.UP:
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(            grid_map, from_upper=True)
    elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(            grid_map, from_upper=False)

    return grid_map, x_inds_goal_y, goal_y


def sweep_path_search(sweep_searcher, grid_map):
    # search start grid
    c_x_index, c_y_index = sweep_searcher.search_start_grid(grid_map)
    if not grid_map.set_value_from_xy_index(c_x_index, c_y_index, 0.5):
        print("Cannot find start grid")
        return [], []

    x, y = grid_map.calc_grid_central_xy_position_from_xy_index(c_x_index,c_y_index)
    px, py = [x], [y]

    while True:
        c_x_index, c_y_index = sweep_searcher.move_target_grid(c_x_index,c_y_index,grid_map)

        if sweep_searcher.is_search_done(grid_map) or (c_x_index is None or c_y_index is None):
            print("Done")
            break

        x, y = grid_map.calc_grid_central_xy_position_from_xy_index(c_x_index, c_y_index)

        px.append(x)
        py.append(y)

        grid_map.set_value_from_xy_index(c_x_index, c_y_index, 0.5)

    return px, py


def planning(ox, oy, resolution,moving_direction=SweepSearcher.MovingDirection.RIGHT,sweeping_direction=SweepSearcher.SweepDirection.UP,):
    sweep_vec, sweep_start_position = find_sweep_direction_and_start_position(ox, oy)

    rox, roy = convert_grid_coordinate(ox, oy, sweep_vec,sweep_start_position)

    grid_map, x_inds_goal_y, goal_y = setup_grid_map(rox, roy, resolution, sweeping_direction)

    sweep_searcher = SweepSearcher(moving_direction, sweeping_direction,x_inds_goal_y, goal_y)

    px, py = sweep_path_search(sweep_searcher, grid_map)

    rx, ry = convert_global_coordinate(px, py, sweep_vec,sweep_start_position)
    return rx, ry
def publish_marker(x,y,i,color):
    marker = Marker();
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    # marker.ns = "path_visualization"
    marker.id = i
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1.0
    if(color=="r"):
        marker.color.r = 1.0
        marker.color.b = 0.0
    else :
        marker.color.b = 1.0
        marker.color.r = 0.0
    path_pub.publish( marker )

def waypoint_motion():
    drone_coordinates_x = [4.4   , 10.1 , 10.1 , 4.4 , 4.4]
    drone_coordinates_y = [-3.02 , -3.02, 1.9, 1.9 , -3.02]

    resolution = 0.9
    px, py = planning(drone_coordinates_x, drone_coordinates_y, resolution)
    print("x: ",len(px) ,"/ y: ", len(py))

    client= actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    # current_pose_x =
    for i in range(len(px)-1):
        print("Moving to point :" , px[i],"/",py[i])
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = px[i]
        goal.target_pose.pose.position.y = py[i]
        goal.target_pose.pose.orientation.w = 0.99

        publish_marker(px[i],py[i],i,rospy.get_param("/marker_color"))


        # alpha = Math.atan((yb - yp) / (xb - xp));
        if(i==5):
            print("Spawning Second Drone")

            # Launching second drone
            subprocess.Popen(["roslaunch", "coverage_drone", "spawn_drone_second.launch", f"x_pos:={px[i]}", f"y_pos:={py[i]}"])
            subprocess.Popen(["rosrun", "coverage_drone", "resetting.bash"])

        client.send_goal(goal)
        wait=client.wait_for_result()
        if not wait:
            rospy.logerr("Action server Down !! ")
        else:
            print("Goal is Completed")



if __name__ == '__main__':
    rospy.init_node("Interactive_poninter_goals")
    path_pub = rospy.Publisher('/path_visualization', Marker, queue_size=10)
    move_robot = waypoint_motion()

