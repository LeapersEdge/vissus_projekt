#!/usr/bin/env python3

"""TASK DESCRIPTION: Each bidder should know its initial position and set of capabilities (we assume that all robots are the same, have
the same speed, and can perform all tasks).

Upon receiving the list of tasks in the auction, the bidder tries to insert each task into its current schedule and
computes the corresponding total schedule duration (makespan). It then selects the task that would result in the
minimum makespan and submits it as a bid (task-makespan pair)."""
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy as ros
import copy
import numpy as np
import networkx as nx
from rclpy.node import Node
from vissus_projekt.msg import TaskList, Task, Bid
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
import json

class Bidder(Node):
    def __init__(self):
        super().__init__('Bidder')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)

        # Get the values
        self.robot_id = self.get_parameter('robot_id').value
        self.start_position = [
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value
        ]

        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.speed = 1  # cost of task will be calculated using speed and distance to point and durtion of task
        # all robots of same speed? maybe it could just stay speed=1
        self.my_schedule =  nx.DiGraph() # task_ID as node and node data (start_time, task)
        self.tasks = dict() # dict of al tasks in schedule (key == task ID)
        self.last_processed_round = -1
        self.active_bid = None
        self.first_task = None

        # Init Publishers
        self.bid_publisher = self.create_publisher(Bid, 'bids', qos_profile)
        self.ready_publisher = self.create_publisher(Bool, 'bidder_ready', qos_profile)

        # Init Subscribers
        self.task_sub = self.create_subscription(
            TaskList, 'auction_task_list', self.task_callback, qos_profile)

        self.winner_sub = self.create_subscription(
            Bid, 'auction_winner', self.winner_callback, qos_profile)

        self.closing_sub = self.create_subscription(Bool, 'auction_closing', self.closing_callback, qos_profile)

    def closing_callback(self, msg):
        self.get_logger().info("Market closed... shutting down")
        raise SystemExit

    def save_schedule(self):
        # Convert graph to a dictionary format
        data = nx.node_link_data(self.my_schedule, edges='edges')

        # Replacing task with dictionary of its values
        for node in data['nodes']:
            if 'task' in node:
                t = node['task']
                node['task_details'] = {
                    'id': t.id,
                    'pos': list(t.pos),
                    'duration': t.duration,
                    'robots_needed': t.robots_needed,
                    'predecessors': list(t.predecessors),
                    'earliest_start': t.earliest_start,
                }
                del node['task']

        # Write to JSON file
        filename = f"robot_{self.robot_id + 1}_schedule.json"
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)

        self.get_logger().info(f"Schedule saved to {filename}")

    def distance(self, A, B):
        return np.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)

    def calculate_bid_value(self, task_to_bid):
        """Logic to determine the cost for a task"""
        best_makespan = float('inf')
        best_schedule = None
        new_node = task_to_bid.id

        # going through all possible positions in schedule and choosing the one with lowest makespan
        for index in range(len(self.tasks) + 1):
            temp_graph = copy.deepcopy(self.my_schedule)
            temp_graph.add_node(task_to_bid.id, task=task_to_bid)

            if index == 0:
                if self.first_task is not None:
                    dist = self.distance(task_to_bid.pos, temp_graph.nodes[self.first_task]['task'].pos)
                    temp_graph.add_edge(task_to_bid.id, self.first_task, distance=dist)
                temp_first = task_to_bid.id
            else:
                # Middle or End
                pred = self.first_task
                for _ in range(1, index):
                    pred = next(temp_graph.successors(pred))

                succ = next(temp_graph.successors(pred), None)

                if succ is not None: # if not the last
                    temp_graph.remove_edge(pred, succ)
                    dist_new = self.distance(task_to_bid.pos, temp_graph.nodes[succ]['task'].pos)
                    temp_graph.add_edge(task_to_bid.id, succ, distance=dist_new)

                dist_pred = self.distance(temp_graph.nodes[pred]['task'].pos, task_to_bid.pos)
                temp_graph.add_edge(pred, task_to_bid.id, distance=dist_pred)
                temp_first = self.first_task

            makespan, start = self.calculate_makespan(temp_graph, temp_first, new_node)

            if makespan < best_makespan:
                best_makespan = makespan
                best_schedule = temp_graph

        return best_makespan, best_schedule, start

    def calculate_makespan(self, graph, start_node, new_node):
        """Calculates the total duration of given schedule and assigns start_times along the way."""
        curr = start_node

        t_obj = graph.nodes[curr]['task']
        dist_from_robot = self.distance(self.start_position, t_obj.pos)
        arrival = dist_from_robot / self.speed
        earliest = max(arrival, t_obj.earliest_start)
        graph.nodes[curr]['start_time'] = earliest if graph.nodes[curr].get('start_time') is None else max(graph.nodes[curr]['start_time'], earliest)

        while True:
            if curr == new_node:
                start_time = graph.nodes[curr]['start_time']
            successors = list(graph.successors(curr))
            if not successors:
                return graph.nodes[curr]['start_time'] + graph.nodes[curr]['task'].duration, start_time # if last node return makespan

            nxt = successors[0]
            dist = graph.edges[curr, nxt]['distance']

            arrival_next = graph.nodes[curr]['start_time'] + graph.nodes[curr]['task'].duration + (dist / self.speed)
            earliest = max(arrival_next, graph.nodes[nxt]['task'].earliest_start)
            graph.nodes[nxt]['start_time'] = earliest if graph.nodes[nxt].get('start_time') is None else max(earliest, graph.nodes[nxt]['start_time'])

            curr = nxt

    def task_callback(self, msg):
        """Triggered when the Auctioneer publishes tasks"""
        if msg.auction_round <= self.last_processed_round:
            return

        self.last_processed_round = msg.auction_round
        lowest_cost =float(1e10)
        best_bid = None

        # Evaluate all tasks and pick the best one
        for task in msg.tasks:
            if task.id in self.tasks:
                 none_id = task.id
                 continue
            # Get the task with best bid
            makespan, schedule, start = self.calculate_bid_value(task)
            if makespan < lowest_cost:
                lowest_cost = makespan
                best_bid = makespan, schedule, task, start

        if best_bid is None: # Send a huge bid that will not win
            bid = Bid()
            bid.auction_round = msg.auction_round
            bid.bidder_id = self.robot_id
            bid.task_id = none_id
            bid.bid = lowest_cost
            bid.task_start = -1.0
        else:
            bid = Bid()
            bid.auction_round = msg.auction_round
            bid.bidder_id = self.robot_id
            bid.task_id = best_bid[2].id
            bid.bid = best_bid[0]
            bid.task_start = best_bid[3]

        self.active_bid = best_bid
        self.bid_publisher.publish(bid)

    def winner_callback(self, winning_bid):
        """Triggered when the Auctioneer announces a winner."""
        self.get_logger().info(f"Winner callback {winning_bid.task_id} id, round {winning_bid.auction_round}")

        if winning_bid.auction_round != self.last_processed_round: # processes only current round
            return
        # check if you're the winner
        if winning_bid.bidder_id == self.robot_id:
            if winning_bid.task_id not in self.tasks:
                self.get_logger().info(f"got {winning_bid.task_id}, active_bid is {self.active_bid[2].id}")
                makespan, schedule, task, start = self.active_bid
                self.my_schedule = schedule
                self.first_task = next(n for n, d in self.my_schedule.in_degree() if d == 0)
                self.tasks[task.id] = task
                self.active_bid = None

            else: # update start_time and push successors if necessary
                node = winning_bid.task_id
                node_data = self.my_schedule.nodes[node]

                self.get_logger().info(f"Updating task {winning_bid.task_id} start_time from {self.my_schedule.nodes[node].get('start_time')} to {winning_bid.task_start}...")
                node_data['start_time'] = winning_bid.task_start

                next_node = next(self.my_schedule.successors(node), None)
                while next_node is not None:
                    distance = self.my_schedule.edges[node, next_node]['distance']
                    next_node_data = self.my_schedule.nodes[next_node]
                    next_node_data['start_time'] = np.max(next_node_data['start_time'], node_data['start_time'] + node_data['task'].duration + distance/self.speed)
                    node = next_node
                    node_data = next_node_data
                    next_node = next(self.my_schedule.successors(node), None)

            self.save_schedule()
        ready = Bool()
        self.ready_publisher.publish(ready)


def main(args=None):
    ros.init(args=args)
    node = Bidder()
    ros.spin(node)
    node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
