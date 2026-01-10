#!/usr/bin/env python3
import rclpy
import networkx as nx       #onaj library za rad s grafovima
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TaskAllocator(Node):

    def __init__(self):
        super().__init__('AdjacencyMatrixPublisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'adjacency_matrix',
            10
        )
        self.timer = self.create_timer(1.0, self.publish_matrix)
        self.goals = np.array([         #ovo je samo za primjer za sad, goalove cemo citat od nekud, neki topic ili sta vec
            [0.0, 0.0],
            [1.0, 2.0],
            [3.0, 1.0],
            [4.0, 3.0]
        ])

    def publish_matrix(self):
        msg = Float64MultiArray()

        # 3x3 matrix, row-major
        msg.data = self.create_adjacency_matrix(self.goals).flatten().tolist()

        self.publisher_.publish(msg)
        self.get_logger().info('Published 4x4 matrix')

    def create_adjacency_matrix(self, goal, n = 4):
        #tu kreiramo adj matricu, potreban je cilj ili broj ciljeva i broj dronova koji lete, default je 4, potrebno citati iz filea
        AM = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                if i == j:
                    AM[i, j] = 0.0
                else:
                    AM[i, j] = np.linalg.norm(goal[i] - goal[j])

        return AM

def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()