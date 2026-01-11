#!/usr/bin/env python3
"""The auctioneer should load all the tasks T and their descriptions at initialization. Then, it should organize them
in sets of tasks without predecessors TF = free(T), second layer of tasks whose predecessors are in TF ,
TL = free(T \ TF ), and the set of remaining, hidden tasks TH = T \ {TF ∪ TL}, according to tasks’ precedence
constraints.

After that, the auctioneer starts the task allocation by auctioning the set of free tasks TF .

The auctioneer collects all the bids and selects the best one. It communicates its selection to the bid winner, who
then permanently adds the tasks to its list of assigned tasks. The auctioneer starts the process again with the
remaining tasks until all tasks from TF are assigned. """

import rclpy as ros
import networkx as nx  # onaj library za rad s grafovima
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import json
import matplotlib.pyplot as plt
from vissus_projekt.msg import TaskList, Task, Bid


class Auctioneer(Node):
    def __init__(self):
        super().__init__('Auctioneer')
        tasks = self.load_mission_config()
        self.task_graph = self.build_tasks_graph(tasks)
        self.free_tasks = list()
        self.free_tasks_bids = dict()
        self.expected_bids = self.get_num_robots()
        self.allocated_tasks = list()
        self.auction_round = 0

        self.task_publisher = self.create_publisher(TaskList, 'auction_task_list', 10)

        self.winner_publisher = self.create_publisher(Bid, 'auction_winner', 10)

        self.bid_listener = self.create_subscription(Bid, 'bids', self.bid_evaluator_callback, 10)

        self.publish_free_tasks()

    def get_num_robots(self):
        return 4  # TODO: forward from launch

    def build_tasks_graph(self, tasks_data):
        task_graph = nx.DiGraph()
        for t in tasks_data:
            # add task nodes with attributes as node data
            task_graph.add_node(t['id'], pos=t['pos'], duration=t['duration'], robots=t['robots_needed'],
                                earliest_start=0)

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

    def visualize_task_graph(self, graph):
        plt.figure(figsize=(8, 6))

        pos = nx.spring_layout(graph)
        nx.draw_networkx_nodes(graph, pos, node_size=700, node_color='skyblue')
        nx.draw_networkx_edges(graph, pos, arrowstyle='->', arrowsize=20, edge_color='gray')
        nx.draw_networkx_labels(graph, pos, font_size=12, font_family='sans-serif')

        plt.title("Task Precedence Graph (DAG)")
        plt.axis('off')
        plt.show()

    def publish_free_tasks(self):
        """publishes"""
        msg = TaskList()
        msg.auction_round = self.auction_round

        for t_id, t_data in self.free_tasks:
            t = Task()
            t.id = t_id
            t.pos = [float(t_data['pos'][0]), float(t_data['pos'][1])]
            t.duration = t_data['duration']
            t.robots_needed = t_data['robots']
            t.earliest_start = t_data['earliest_start']

            # Append the individual task to the list
            msg.tasks.append(t)
            self.free_tasks_bids[t_id] = None
        self.task_publisher.publish(msg)

    def assign_best_bid(self):
        """publishes a winner"""
        best_task_id = min(self.free_tasks_bids,
                           key=lambda k: self.free_tasks_bids[k][0] if self.free_tasks_bids[k] is not None else float(
                               'inf'))

        best_bid = Bid()
        best_bid.task_id = best_task_id
        best_bid.bid = self.free_tasks_bids[best_task_id][0]
        best_bid.bidder_id = self.free_tasks_bids[best_task_id][1]
        best_bid.auction_round = self.free_tasks_bids[best_task_id][2]
        best_bid.task_end = self.free_tasks_bids[best_task_id][3]

        self.winner_publisher.publish(best_bid)
        self.allocated_tasks.append((best_task_id, self.task_graph.nodes[best_task_id]))
        for succ in self.task_graph.successors(best_task_id):
            current_earliest_start = self.task_graph.nodes[succ].get('earliest_start')
            self.task_graph.nodes[succ]['earliest_start'] = max(current_earliest_start, best_bid.task_end)
        self.task_graph.remove_node(best_task_id)

    def bid_evaluator_callback(self, msg):
        """evaluates incoming bids by saving the best ones in free_tasks_bids"""
        bidder_id = msg.bidder_id
        task_id = msg.task_id
        bid_val = msg.bid
        msg_round = msg.auction_round
        task_end = msg.task_end

        if msg_round == self.auction_round:
            # only accepts current round bids
            self.expected_bids -= 1

            if self.free_tasks_bids[task_id] is None or self.free_tasks_bids[task_id][0] > bid:
                self.free_tasks_bids[task_id] = (bid, bidder_id, auction_round, task_end)

            if self.expected_bids == 0:
                self.assign_best_bid()
                self.start_next_auction()

    def start_next_auction(self):
        # odmah se računaju svi slobodni taskovi
        self.free_tasks = [(node, data) for node, data in self.task_graph.nodes(data=True) if
                               self.task_graph.in_degree(node) == 0]
        self.expected_bids = self.get_num_robots()
        self.auction_round += 1
        self.free_tasks_bids.clear()


def main(args=None):
    ros.init(args=args)
    node = Auctioneer()
    ros.spin(node)
    node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
