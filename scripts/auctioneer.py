#!/usr/bin/env python3

"""TASK DESCRIPTION: The auctioneer should load all the tasks T and their descriptions at initialization. Then, it should organize them
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
        self.pending_assignments = dict()

        self.task_publisher = self.create_publisher(TaskList, 'auction_task_list', 10)

        self.winner_publisher = self.create_publisher(Bid, 'auction_winner', 10)

        self.bid_listener = self.create_subscription(Bid, 'bids', self.bid_evaluator_callback, 10)

        self.startup_timer = self.create_timer(1.0, self.wait_for_bidders)

    def wait_for_bidders(self):
        count = self.task_publisher.get_subscription_count()
        self.get_logger().info(f"Waiting for bidders... ({count}/{self.num_robots} connected)")

        if count >= self.num_robots:
            self.get_logger().info("All bidders ready. Starting first auction round.")
            self.startup_timer.cancel()  # Stop checking
            self.start_next_auction()  # Trigger the first round

    def get_num_robots(self):
        return 4  # TODO: forward from launch

    def build_tasks_graph(self, tasks_data):
        task_graph = nx.DiGraph()
        for t in tasks_data:
            # add task nodes with attributes as node data
            task_graph.add_node(t['id'], pos=t['pos'], duration=t['duration'], robots_needed=t['robots_needed'],
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
            t.robots_needed = t_data['robots_needed']
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
        best_bid.task_start = self.free_tasks_bids[best_task_id][3]

        # smanjujem broj robota koji su potrebni
        node_data = self.task_graph.nodes[best_task_id]

        node_data['robots_needed'] -= 1

        # ako je startno vrijeme zadatka nakon onog od bida bit ce odgodeno
        node_data['earliest_start'] = max(node_data['earliest_start'], best_bid.task_start)

        if best_task_id not in self.pending_assignments:
            self.pending_assignments[best_task_id] = []

        self.pending_assignments[best_task_id].append(best_bid)
        if node_data['robots_needed'] == 0:  # dovoljno robota je dobilo zadatak
            final_start_time = node_data['earliest_start']
            for i in self.pending_assignments[best_task_id]:
                final_bid = Bid()
                final_bid.auction_round = i.auction_round
                final_bid.bidder_id = i.bidder_id
                final_bid.task_id = i.task_id
                final_bid.bid = i.bid
                final_bid.task_start = final_start_time

                self.winner_publisher.publish(final_bid)

            # ispunjena je kvota robota pa mičem zadatak iz liste
            self.allocated_tasks.append((best_task_id, self.task_graph.nodes[best_task_id]))
            for succ in self.task_graph.successors(best_task_id):
                current_earliest_start = self.task_graph.nodes[succ].get('earliest_start')
                self.task_graph.nodes[succ]['earliest_start'] = max(current_earliest_start,
                                                                    final_start_time + node_data['duration'])
            del self.pending_assignments[best_task_id]
            self.task_graph.remove_node(best_task_id)
        else:  # ako je potrebno čekati da se pronade jos robota šaljem privremeni bid koji ce se poslje updateat
            self.winner_publisher.publish(best_bid)

    def bid_evaluator_callback(self, msg):
        """evaluates incoming bids by saving the best ones in free_tasks_bids"""
        bidder_id = msg.bidder_id
        task_id = msg.task_id
        bid = msg.bid
        auction_round = msg.auction_round
        task_start = msg.task_start

        if auction_round == self.auction_round:
            # only accepts current auction round bids
            self.expected_bids -= 1

            if self.free_tasks_bids[task_id] is None or self.free_tasks_bids[task_id][0] > bid:
                self.free_tasks_bids[task_id] = (bid, bidder_id, auction_round, task_start)

            if self.expected_bids == 0:
                self.assign_best_bid()
                self.start_next_auction()

    def start_next_auction(self):
        # free tasks are all tasks with no predecessor
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
