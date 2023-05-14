from math import sin, cos, sqrt, pi, atan2

import numpy as np
import matplotlib.pyplot as plt

from global_planner import PriorityQueueFrontier


class MotionPrimitiveNode:
    """..."""

    def __init__(
        self, path, time, layer, index, parent, heuristic, accum_cost
    ):
        self.path = path
        self.time = time
        self.layer = layer
        self.index = index
        self.parent = parent
        self.heuristic = heuristic
        self.accum_cost = accum_cost
        self.ctg = heuristic + accum_cost

    def __lt__(self, other):
        return self.ctg < other.ctg

    def __gt__(self, other):
        return self.ctg > other.ctg


class LocalPlanner:
    def __init__(
        self,
        arc_length=0.3,
        velocity=0.3,
        dt=0.05,
        steps=21,
        motion_prim_conf=[-0.2, -0.4, "inf", 0.4, 0.2],
        # [-0.16, -0.3, "inf", 0.3, 0.16],
    ):
        self.pose = (0, 0, 0, 1)
        self.target = (0, 0, 0)
        self.prev = (0, 0, 0)
        self.arc_length = arc_length
        self.velocity = velocity
        self.dt = dt
        self.steps = steps
        self.motion_prim_conf = motion_prim_conf
        self.motion_prims = []
        self.dyn_obstacles = []
        self.dyn_obstacles_fixed = []
        self.paths_lists = []
        self.collisions = []

    def generate_motion_primitives(self):
        for r in self.motion_prim_conf:
            mp = []
            for i in range(1, self.steps + 1):
                s = self.velocity * self.dt * i
                if r == "inf":
                    phi = 0
                    x = s
                    y = 0
                else:
                    phi = s / r
                    x = r * sin(phi)
                    y = r * (1 - cos(phi))
                mp.append([x, y, phi, 1])
            self.motion_prims.append(np.array(mp).T)

    def show_motion_prims(self):
        for mp in self.motion_prims:
            plt.scatter(mp[0, :], mp[1, :], c="green")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.show()

    def show_ego_tree(self):
        end_poses = [(1, [0, 0, 0, 1])]
        while end_poses:
            l, pose = end_poses.pop()
            R = self.rot_trans_matrix(pose)
            for mp in self.motion_prims:
                rot_mp = R @ mp
                plt.scatter(rot_mp[0, :], rot_mp[1, :], s=2, c="green")
                if l < 4:
                    end_poses.append((l + 1, rot_mp[:, -1]))
        plt.xlabel("x [m]")
        plt.xlim(-1, 1.6)
        plt.ylabel("y [m]")
        plt.ylim(1.5, -1.5)
        # plt.plot([-0.5, 1.5], [1, 0], linewidth=2, color="r")
        # plt.plot([-0.5], [1], color="b", marker="o", markersize=20)
        plt.show()

    def dyn_obs_callback(self, obstacles):
        # self.c = obstacles.circles
        self.dyn_obstacles = obstacles.circles

    def is_inside_circle(self, xo, yo, r, x, y):
        inside = False
        if (x - xo) ** 2 + (y - yo) ** 2 <= r ** 2:
            inside = True
        return inside

    def is_in_collision(self, path, t):
        for obstacle in self.dyn_obstacles_fixed:
            xo = obstacle.center.x
            yo = obstacle.center.y
            vx = obstacle.velocity.x
            vy = obstacle.velocity.y
            r = obstacle.radius
            if -0.1 < vx < 0.1 and -0.1 < vy < 0.1:
                continue
            print(vx, vy)
            for i in range(1 + t, self.steps + 1 + t):
                xi = xo + vx * self.dt * i
                yi = yo + vy * self.dt * i
                x, y, _, _ = path[:, i - t - 1]
                if self.is_inside_circle(xi, yi, r, x, y):
                    self.collisions.append(path)
                    return True
        return False

    def rot_trans_matrix(self, pose):
        x, y, th, _ = pose
        R = np.array(
            [
                [cos(th), -sin(th), 0, x],
                [sin(th), cos(th), 0, y],
                [0, 0, 1, th],
                [0, 0, 0, 1],
            ]
        )
        return R

    def neighbors(self, node):
        """Rotated motion prims in the next layer"""
        pose = node.path[:, -1]
        # print(pose)
        R = self.rot_trans_matrix(pose)

        t = node.time + len(node.path)

        neighbors = []
        for i in range(len(self.motion_prims)):
            mp = R @ self.motion_prims[i]
            if not self.is_in_collision(mp, t):
                neighbors.append((i, mp, t))

        return neighbors

    def optimize(self):
        self.dyn_obstacles_fixed = self.dyn_obstacles
        # Optimal cost to goal
        self.G = float("inf")

        # Keep track of no. of states explored
        self.num_explored = 0

        # Initialize the frontier to just the starting mption prims
        frontier = PriorityQueueFrontier()
        print(self.pose)
        R = self.rot_trans_matrix(self.pose)
        for i in range(len(self.motion_prims)):
            path = R @ self.motion_prims[i]
            if self.is_in_collision(path, 0):
                continue
            # print(i)
            cost = self.cross_track_error(path)
            cost += self.curvature_error(i)
            source = MotionPrimitiveNode(
                path=path,
                time=0,
                layer=0,
                index=i,
                parent=None,
                heuristic=self.euclidean_dist(path),
                accum_cost=cost,
            )
            frontier.push(source)

        # Initialise an empty explored set
        self.explored = set()

        # Keep looping until solution found
        while not frontier.empty():
            # print([node.accum_cost for node in frontier.frontier[:5]])
            # Choose a node with the least cost from frontier
            node = frontier.pop()
            self.num_explored += 1
            # print(node.loc, node.dir, node.accum_cost)

            # Mark node as explored
            # self.explored.add(node.index)

            if node.accum_cost >= self.G:
                continue

            # If node is goal, then we have a Solution
            if node.layer == 5:
                print("hi", node.accum_cost)
                self.G = node.accum_cost
                self.G_node = node
                continue

            # Add neighbour to frontier
            for idx, path, t in self.neighbors(node):
                # if ind not in self.explored:
                # print(idx)
                cost = node.accum_cost
                cost += self.cross_track_error(path)
                cost += self.curvature_error(idx)
                # Each of the above costs can be weighted to prioratize one over the other.
                child = MotionPrimitiveNode(
                    path=path,
                    time=t,
                    layer=node.layer + 1,
                    index=idx,
                    parent=node,
                    heuristic=self.euclidean_dist(path),
                    accum_cost=cost,
                )
                frontier.push(child)

        if self.G == float("inf"):
            # Stop and wait
            # raise Exception("No Solution\n")
            print("No Solution")
            return 0, 0
        else:
            opt_idx = self.backtrack(self.G_node)
            print(opt_idx)

        r_opt = self.motion_prim_conf[opt_idx]
        if r_opt == "inf":
            return self.velocity, 0
        else:
            return self.velocity, self.velocity / r_opt

    def euclidean_dist(self, path):
        x, y, _, _ = path[:, -1]
        # print(self.target)
        gx, gy, _ = self.target
        return sqrt((gx - x) ** 2 + (gy - y) ** 2)

    def cross_track_error(self, path):
        x1, y1, _ = self.prev
        x2, y2, _ = self.target
        th = atan2((y2 - y1), (x2 - x1))
        # print(x1, y1, th)
        R1 = self.rot_trans_matrix((-x1, -y1, 0, 1))
        R2 = self.rot_trans_matrix((0, 0, -th, 1))
        # print(path)
        rot_path = R2 @ R1 @ path
        # print(rot_path)
        cte = sum(abs(y) for y in rot_path[1, :])
        if rot_path[0, -1] < 0:
            cte += 10
            # print(cte)
        # input()
        return cte

    def curvature_error(self, idx):  # delta theta could also be used
        cur = self.motion_prim_conf[idx]
        if cur == "inf":
            return 0
        else:
            return 1 / abs(cur)

    def cost_map_error(self, path):
        pass

    def backtrack(self, node):
        """..."""
        paths = []
        while node.parent is not None:
            # print(node.index)
            node = node.parent
            paths.append(node.path)
        self.paths_lists.append(paths)
        return node.index


if __name__ == "__main__":
    planner = LocalPlanner()
    planner.target = (-1, 1, 0)
    planner.generate_motion_primitives()
    planner.show_motion_prims()
    planner.show_ego_tree()
    print("Solution:", planner.optimize())
