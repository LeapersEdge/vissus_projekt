#!/usr/bin/env python3

import rclpy
import json
import numpy as np
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


class TaskAllocation(Node):

    def __init__(self):
        super().__init__('task_allocation')

        self.num_robots = 4
        self.goal_tolerance = 0.2

        base_path = Path(__file__).resolve().parent
        self.tasks_file = base_path.parent / 'config' / 'tasks.json'
        self.schedule_folder = base_path

        self.task_pos = {}              # task_id -> np.array([x,y])
        self.task_required = {}         # task_id -> robots_needed

        self.task_to_robots = {}        # task_id -> set(robot_id)
        self.task_completed = set()

        self.robot_positions = {}       # robot_id -> np.array([x,y])
        self.robot_goals = {}            # robot_id -> [x,y]
        self.active_robots = set()


        self.goal_pub = self.create_publisher(PoseArray, 'robot_goals', 10)
        self.adj_pub = self.create_publisher(Float64MultiArray, 'adjacency_matrix', 10)

        for r in range(1, self.num_robots+1):
            self.create_subscription(
                PoseStamped,
                f'cf_{r}/pose',
                lambda msg, rid=r: self.pose_callback(msg, rid),
                10
            )

        # --------------------------------------------------
        # Initialization
        tasks_data = self.load_tasks()  # load_tasks now returns the tasks list
        self.task_state = {t['id']: 'pending' for t in tasks_data}
        self.task_predecessors = {t['id']: set(t.get('predecessors', [])) for t in tasks_data}
        self.robot_state = {r: 'idle' for r in range(1, self.num_robots+1)}


        self.load_robot_schedules()
        # Check pending tasks twice per second
        self.create_timer(0.5, self.try_activate_tasks)
        #self.try_activate_tasks()

    # ==================================================
    # Initialization
    # ==================================================

    def load_tasks(self):
        with open(self.tasks_file, 'r') as f:
            data = json.load(f)

        for t in data['tasks']:
            tid = t['id']
            self.task_pos[tid] = np.array(t['pos'], dtype=float)
            self.task_required[tid] = t['robots_needed']
            self.task_to_robots[tid] = set()

        self.get_logger().info(f"Loaded {len(self.task_pos)} tasks")
        return data['tasks']


    def load_robot_schedules(self):
        for r in range(1, self.num_robots+1):
            path = self.schedule_folder / f'robot_{r}_schedule.json'
            if not path.exists():
                self.get_logger().warn(f"Schedule not found: {path}")
                continue

            with open(path, 'r') as f:
                sched = json.load(f)

            for node in sched.get('nodes', []):
                tid = node['id']
                if tid in self.task_to_robots:
                    self.task_to_robots[tid].add(r)



        self.get_logger().info("Robot schedules loaded")

    def try_activate_tasks(self):
        for tid, robots in self.task_to_robots.items():

            if self.task_state[tid] != 'pending':
                continue
            if not self.task_predecessors[tid].issubset(self.task_completed):
                continue
            if not all(self.robot_state[r] == 'idle' for r in robots):
                continue

            # 3. Correct coalition size
            if len(robots) != self.task_required[tid]:
                continue
            
            # ACTIVATE
            self.task_state[tid] = 'active'

            for r in robots:
                self.robot_state[r] = 'busy'
                self.robot_goals[r] = self.task_pos[tid].tolist()
                self.active_robots.add(r)

            self.get_logger().info(
                f"Task {tid} activated"
            )

        self.publish_goals()
        self.publish_adjacency_matrix()


    # ==================================================
    # Runtime callbacks
    # ==================================================

    def pose_callback(self, msg: PoseStamped, robot_id: int):
        # Update robot position
        self.robot_positions[robot_id] = np.array(
        [msg.pose.position.x, msg.pose.position.y],
        dtype=float
    )

        # self.get_logger().info(
        #     f"[POSE] Robot {robot_id} @ "
        #     f"x={msg.pose.position.x:.2f}, "
        #     f"y={msg.pose.position.y:.2f}"
        # )
        self.check_task_completion()

    # ==================================================
    # Task completion logic
    # ==================================================

    def check_task_completion(self):
        for tid, robots in self.task_to_robots.items():
            if tid in self.task_completed:
                continue
            
            if self.task_state.get(tid) != 'active':
                continue


            # Only consider fully assigned tasks
            if len(robots) != self.task_required[tid]:
                self.get_logger().debug(
                    f"[SKIP] Task {tid}: "
                    f"coalition size mismatch "
                    f"({len(robots)} != {self.task_required[tid]})"
                )
                continue

            goal = self.task_pos[tid]
            arrived = True

            for r in robots:
                if r not in self.robot_positions:
                    arrived = False
                    self.get_logger().warn(
                        f"[WAIT] Task {tid}: "
                        f"robot {r} has no pose yet"
                    )
                    break

                dist = np.linalg.norm(self.robot_positions[r] - goal)
                if dist > self.goal_tolerance:
                    # self.get_logger().info(
                    #    f"[WAIT] Task {tid}: "
                    #    f"robot {r} distance={dist:.3f} "
                    #    f"(tolerance={self.goal_tolerance})"
                    # )
                    arrived = False
                    break

            if arrived:
                self.complete_task(tid)

    def complete_task(self, task_id: int):
        self.task_completed.add(task_id)
        self.task_state[task_id] = 'completed'

        robots = self.task_to_robots[task_id]
        for r in robots:
            self.robot_goals.pop(r, None)
            self.active_robots.discard(r)
            self.robot_state[r] = 'idle'

        self.get_logger().info(
            f"Task {task_id} completed by robots {sorted(robots)}"
        )

        self.publish_goals()
        self.publish_adjacency_matrix()


    def publish_goals(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for i in range(1, self.num_robots + 1):
            pose = Pose()
            if i in self.robot_goals:
                pose.position.x = float(self.robot_goals[i][0])
                pose.position.y = float(self.robot_goals[i][1])
            else:
                pose.position.x = float('nan')
                pose.position.y = float('nan')
            msg.poses.append(pose)

        self.goal_pub.publish(msg)
        for i, pose in enumerate(msg.poses, start=1):
            self.get_logger().info(
                f"Published goal for robot {i}: x={pose.position.x:.2f}, y={pose.position.y:.2f}"
            )


    def publish_adjacency_matrix(self):
        n = self.num_robots
        A = np.zeros((n, n))

        for i in self.active_robots:
            for j in self.active_robots:
                if i != j:
                    A[i-1, j-1] = 1.0

        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='rows', size=n, stride=n * n),
            MultiArrayDimension(label='cols', size=n, stride=n),
        ]
        msg.data = A.flatten().tolist()

        self.adj_pub.publish(msg)
        self.get_logger().info(f"Published adjacency matrix to 'adjacency_matrix':\n{A}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
