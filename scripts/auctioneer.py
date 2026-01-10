"""The auctioneer should load all the tasks T and their descriptions at initialization. Then, it should organize them
in sets of tasks without predecessors TF = free(T), second layer of tasks whose predecessors are in TF ,
TL = free(T \ TF ), and the set of remaining, hidden tasks TH = T \ {TF ∪ TL}, according to tasks’ precedence
constraints.

After that, the auctioneer starts the task allocation by auctioning the set of free tasks TF .

The auctioneer collects all the bids and selects the best one. It communicates its selection to the bid winner, who
then permanently adds the tasks to its list of assigned tasks. The auctioneer starts the process again with the
remaining tasks until all tasks from TF are assigned. """

import rclpy as ros
import networkx as nx       #onaj library za rad s grafovima
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import json
import matplotlib.pyplot as plt


class Auctioneer(Node):
    def __init__(self):
        super().__init__('Auctioneer')
        tasks = self.load_mission_config()
        self.task_graph = self.build_tasks_graph(tasks)
        self.visualize_task_graph(self.task_graph)

        self.task_publisher = self.create_publisher(
            "TaskList",
            'task_list',
            10
        )

        self.free_tasks = list()
        self.allocated_tasks = list()

        self.publish_free_tasks()


    def build_tasks_graph(self, tasks_data):
        task_graph = nx.DiGraph()
        for t in tasks_data:
            # add task nodes
            task_graph.add_node(t['id'], pos=t['pos'], duration=t['duration'], robots=t['robots_needed'])

            # Add edges from predecessors
            for p_id in t['predecessors']:
                task_graph.add_edge(p_id, t['id'])
        return task_graph

    def load_mission_config(self):
        # TODO: ROS Path
        json_path = "../config/tasks.json"

        # open json file and load the data
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                return data['tasks']
        except FileNotFoundError:
            print(f"Could not find tasks.json at {json_path}")

    def visualize_task_graph(self, G):
        plt.figure(figsize=(8, 6))

        pos = nx.spring_layout(G)
        nx.draw_networkx_nodes(G, pos, node_size=700, node_color='skyblue')
        nx.draw_networkx_edges(G, pos, arrowstyle='->', arrowsize=20, edge_color='gray')
        nx.draw_networkx_labels(G, pos, font_size=12, font_family='sans-serif')

        plt.title("Task Precedence Graph (DAG)")
        plt.axis('off')
        plt.show()

    def publish_free_tasks(self):
        """publishes all tasks which have no predecessor (ready to allocate)"""
        self.free_tasks = [node for node, degree in self.task_graph.in_degree() if degree == 0]
        msg = TaskList()

        for t_data in self.free_tasks:
            t = Task()
            t.id = t_data['id']
            t.pos = [float(t_data['pos'][0]), float(t_data['pos'][1])]
            t.duration = t_data['duration']
            t.robots_needed = t_data['robots_needed']
            t.predecessors = t_data['predecessors']

            # Append the individual task to the list
            msg.tasks.append(t)

        self.task_publisher.publish(msg)



def main(args=None):
    auctioneer = Auctioneer()
    # ros.init(args=args)
    # node = Auctioneer()
    # ros.spin(node)
    # node.destroy_node()
    # ros.shutdown()

if __name__ == '__main__':
    main()