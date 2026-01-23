#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from vissus_projekt.msg import TaskList, Bid


class TaskAllocation(Node):

    def __init__(self):
        super().__init__('task_allocation')

        self.num_robots = 4  # ovako je hardkodirano, vjerojatno necemo citati

        # Varijable u koje se sprema goalovi i aktivni roboti
        self.task_goals = {}            # task_id -> [x, y]
        self.robot_goals = {}           # robot_id -> [x, y]
        self.active_robots = set()      # robots with finalized tasks
        self.task_to_robots = {}        # task_id -> set(robot_id)
        self.task_required = {}         # task_id -> robots_needed

        # Subscribers
        self.create_subscription(
            TaskList,
            'auction_task_list',
            self.task_list_callback,
            10
        )

        self.create_subscription(
            Bid,
            'auction_winner',
            self.bid_callback,
            10
        )

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseArray,
            'robot_goals',
            10
        )

        self.adj_pub = self.create_publisher(
            Float64MultiArray,
            'adjacency_matrix',
            10
        )

    def task_list_callback(self, msg: TaskList):
        """
        Store task goals and required robots.
        """
        for t in msg.tasks:
            self.task_goals[t.id] = t.pos
            self.task_required[t.id] = t.robots_needed
            self.task_to_robots[t.id] = set()

    def bid_callback(self, msg: Bid):
        """
        Assign bid to robot, but only finalize task when all required robots are assigned.
        """
        task_id = msg.task_id
        robot_id = msg.bidder_id

        if task_id not in self.task_goals:
            self.get_logger().warn(f"Task {task_id} goal not known yet")
            return

        self.task_to_robots[task_id].add(robot_id)


        if len(self.task_to_robots[task_id]) == self.task_required[task_id]:
            goal = self.task_goals[task_id]
            for r in self.task_to_robots[task_id]:
                self.robot_goals[r] = goal
                self.active_robots.add(r)

            self.get_logger().info(
                f"Task {task_id} finalized with robots {sorted(self.task_to_robots[task_id])}"
            )
            self.publish_goals()
            self.publish_adjacency_matrix()


    def publish_goals(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for i in range(self.num_robots):
            pose = Pose()
            if i in self.robot_goals:
                pose.position.x = float(self.robot_goals[i][0])
                pose.position.y = float(self.robot_goals[i][1])
            else:
                pose.position.x = float('nan')
                pose.position.y = float('nan')
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.goal_pub.publish(msg)

    def publish_adjacency_matrix(self):
        n = self.num_robots
        A = np.zeros((n, n))
        #samo aktivni roboti su povezani
        for i in self.active_robots:
            for j in self.active_robots:
                if i != j:
                    A[i, j] = 1.0

        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='rows', size=n, stride=n * n),
            MultiArrayDimension(label='cols', size=n, stride=n),
        ]
        msg.data = A.flatten().tolist()

        self.adj_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
