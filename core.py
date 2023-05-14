import os
from math import sqrt, sin, cos, atan2, pi
import argparse
from time import sleep
import signal

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from obstacle_detector.msg import Obstacles
import rospy
import tf
import matplotlib.pyplot as plt

from map import Map
from global_planner import GlobalPlanner
from local_planner import LocalPlanner


class Navigation:
    def __init__(
        self,
        map_file,
        source_loc=(-1.8, -1.5),
        source_dir=0,
        target=(0.7, 2.0),
        arc_length=0.2,
        velocity=0.3,
        dt=0.05,
        steps=21,
    ):
        self.bot_pose = (0, 0, 0)  # theta in degrees
        self.pose_list = []
        self.decision_points = []
        self.waypoints = [(0, 0, 0)]
        self.control = Twist()
        self.control.linear.x = 0
        self.control.linear.y = 0
        self.control.linear.z = 0
        self.control.angular.x = 0
        self.control.angular.y = 0
        self.control.angular.z = 0
        self.done = False

        self.map = Map(map_file)
        self.map.downsample()
        self.map.show()

        # os.environ["ROBOT_INITIAL_POSE"] = "-x 2 -y 2 -Y 0"
        self.global_planner = GlobalPlanner(
            self.map, source_loc, source_dir, target
        )
        self.local_planner = LocalPlanner(arc_length, velocity, dt, steps)

        rospy.init_node("Navigation")
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber(
            "/odom", Odometry, self.odometry_callback
        )
        self.dyn_obs_sub = rospy.Subscriber(
            "/obstacles", Obstacles, self.local_planner.dyn_obs_callback
        )

        signal.signal(signal.SIGINT, self.signal_handler)

    def generate_global_path(self):
        # self.waypoints = [(10, 0, 0), (0, 0, 0)]
        print("Generating Global Path...")
        self.global_planner.solve()
        print("States Explored:", self.global_planner.num_explored)
        self.waypoints = self.global_planner.waypoints
        self.waypoints.reverse()
        print("No. of waypoints", len(self.waypoints))
        print(self.waypoints)
        self.global_planner.show_path()
        self.local_planner.target = self.waypoints.pop()

    def odometry_callback(self, data):
        """Called everytime /odom topic is updated."""
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_list = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        ]
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        theta = euler[2]
        # print("Current Bot Orientation:", theta)
        self.bot_pose = (x, y, theta, 1)
        self.pose_list.append(self.bot_pose)

        self.local_planner.pose = self.bot_pose
        gx, gy, _ = self.local_planner.target
        if sqrt((gx - x) ** 2 + (gy - y) ** 2) < 0.2:
            # print(sqrt((gx - x) ** 2 + (gy - y) ** 2))
            if not self.waypoints:
                # self.control.linear.x = 0
                # self.control.angular.z = 0
                # self.velocity_pub.publish(self.control)
                self.done = True
                return
                # rospy.signal_shutdown("Reached Destination.")
            self.local_planner.prev = self.local_planner.target
            self.local_planner.target = self.waypoints.pop()
            print(self.local_planner.prev)
            print(self.local_planner.target)

    def align_to_first_waypoint(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            tx, ty, _ = self.local_planner.target
            rx, ry, r_theta, _ = self.bot_pose
            desired_angle = atan2((ty - ry), (tx - rx)) * 180 / pi
            # print(rx, ry, r_theta)
            r_theta *= 180 / pi
            theta_error = desired_angle - r_theta
            if theta_error < 0.1:
                return
                # rospy.signal_shutdown("Aligned")
            self.control.angular.z = 0.03 * theta_error
            self.velocity_pub.publish(self.control)
            rate.sleep()

    def signal_handler(self, sig, frame):
        print("\nCtrl-C pressed!\nAborting!")
        self.done = True

    def start(self):
        # hz = self.local_planner.velocity / self.local_planner.arc_length
        self.local_planner.generate_motion_primitives()
        self.local_planner.show_motion_prims()
        # self.local_planner.show_motion_prims_tree()
        hz = 1 / (self.local_planner.dt * self.local_planner.steps)
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and not self.done:

            vel, omega = self.local_planner.optimize()
            print("Setting velocity:", vel, omega)
            self.control.linear.x = vel
            self.control.angular.z = omega
            self.velocity_pub.publish(self.control)

            x, y, _, _ = self.local_planner.pose
            self.decision_points.append((x, y))

            rate.sleep()

        self.control.linear.x = 0
        self.control.angular.z = 0
        self.velocity_pub.publish(self.control)
        print("Reached Destination")

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map.cropped, extent=(-1.75, 42.2, -9.9, 28.05))
        plt.plot(
            [x for x, y, _, _ in self.pose_list],
            [y for x, y, _, _ in self.pose_list],
            c="blue",
        )
        plt.scatter(
            [x for x, y in self.decision_points],
            [y for x, y in self.decision_points],
            c="green",
        )
        plt.show()

        # Takes too long!
        # plt.figure(figsize=((10, 10)))
        # plt.ion()
        # for path in self.local_planner.collisions:
        #     plt.clf()
        #     plt.imshow(self.map.grid, extent=(-1.75, 42.2, -9.9, 28.05))
        #     plt.plot(path[0, :], path[1, :])
        #     plt.pause(0.01)

        plt.figure(figsize=(10, 10))
        plt.ion()
        for paths_list in self.local_planner.paths_lists:
            plt.clf()
            plt.imshow(self.map.grid, extent=(-1.75, 42.2, -9.9, 28.05))
            for path in paths_list:
                plt.plot(path[0, :], path[1, :])
            plt.pause(0.01)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mapfile", help="file containing map in .pgm format")
    args = parser.parse_args()

    navigation = Navigation(args.mapfile, (1.192, 1.347), 0, (33, 24.3))
    # navigation = Navigation(args.mapfile, (-1.8, 0.6), 1, (0.7, 2.0))
    # navigation = Navigation(args.mapfile, (-1.8, -1.5), 0, (0.7, 2.0))
    # navigation = Navigation(args.mapfile, (-1.8, -1.5), 0, (0.7, -1.5))
    # navigation = Navigation(args.mapfile, (-1.8, -1.5), 0, (-1.8, 1.0))
    navigation.generate_global_path()
    sleep(1)
    navigation.align_to_first_waypoint()
    navigation.start()
