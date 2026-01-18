#!/usr/bin/env python3

"""TASK DESCRIPTION: Each bidder should know its initial position and set of capabilities (we assume that all robots are the same, have
the same speed, and can perform all tasks).

Upon receiving the list of tasks in the auction, the bidder tries to insert each task into its current schedule and
computes the corresponding total schedule duration (makespan). It then selects the task that would result in the
minimum makespan and submits it as a bid (task-makespan pair)."""

import rclpy as ros
import copy
import numpy as np
import networkx as nx
from rclpy.node import Node
from vissus_projekt.msg import TaskList, Task, Bid
import matplotlib as plt


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

        self.speed = 1  # cost of task will be calculated using speed and distance to point and durtion of task
        # mozda moze ovako ostat stin da su svi iste brzine? nije bitno
        self.my_schedule =  nx.DiGraph() # task_ID as node and node data (start_time = start_time)
        self.tasks = dict() # dict of al tasks in schedule (key == task ID)
        self.last_processed_round = -1
        self.active_bid = None
        self.first_task = None

        # 2. Publishers
        self.bid_publisher = self.create_publisher(Bid, 'bids', 10)

        # 3. Subscribers
        self.task_sub = self.create_subscription(
            TaskList, 'auction_task_list', self.task_callback, 10)

        self.winner_sub = self.create_subscription(
            Bid, 'auction_winner', self.winner_callback, 10)

        self.closing_sub = self.create_subscription(Bid, 'auction_closing', self.closing_callback, 10)

    def closing_callback(self, msg):
        if msg.data:
            self.visualize_schedule(self.my_schedule, self.robot_id)
            ros.shutdown()

    def visualize_schedule(self, graph, robot_id):
        plt.figure(figsize=(10, 4))

        # Position nodes by their actual start_time on the X-axis
        # We set Y to 0 since it's a single robot's timeline
        pos = {node: (data['start_time'], 0) for node, data in graph.nodes(data=True)}

        # Draw the components
        nx.draw_networkx_nodes(graph, pos, node_size=800, node_color='lightgreen')
        nx.draw_networkx_edges(graph, pos, arrowstyle='->', arrowsize=15)

        # Add labels showing Task ID and Start Time
        labels = {n: f"T{n}\n@{data['start_time']:.1f}" for n, data in graph.nodes(data=True)}
        nx.draw_networkx_labels(graph, pos, labels=labels, font_size=10)

        plt.title(f"Final Schedule Timeline: Robot {robot_id}")
        plt.xlabel("Time (seconds)")
        plt.yticks([])  # Hide Y axis as it has no meaning here
        plt.grid(axis='x', linestyle='--', alpha=0.7)

        plt.savefig(f"robot_{robot_id}_timeline.png")
        plt.close()  # Close to free up memory

    def save_schedule(self):
        # TODO:
        pass

    def distance(self, A, B):
        return np.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)

    def calculate_bid_value(self, task_to_bid):
        """Logic to determine the cost for a task"""
        best_makespan = float('inf')
        best_schedule = None
        new_node = task_to_bid.id

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

                if succ is not None:
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
        curr = start_node

        # First Task: Travel from robot current position
        t_obj = graph.nodes[curr]['task']
        dist_from_robot = self.distance(self.start_position, t_obj.pos)
        arrival = dist_from_robot / self.speed
        graph.nodes[curr]['start_time'] = max(arrival, t_obj.earliest_start)


        while True:
            if curr == new_node:
                start_time = graph.nodes[curr]['start_time']
            successors = list(graph.successors(curr))
            if not successors:
                return graph.nodes[curr]['start_time'] + graph.nodes[curr]['task'].duration, start_time

            nxt = successors[0]
            dist = graph.edges[curr, nxt]['distance']

            arrival_next = graph.nodes[curr]['start_time'] + graph.nodes[curr]['task'].duration + (dist / self.speed)
            graph.nodes[nxt]['start_time'] = max(arrival_next, graph.nodes[nxt]['task'].earliest_start)

            curr = nxt

    def task_callback(self, msg):
        """Triggered when the Auctioneer publishes tasks"""
        if msg.auction_round <= self.last_processed_round:
            return

        self.last_processed_round = msg.auction_round
        lowest_cost =float(1e10)
        best_bid = None

        # evaluate all tasks and pick the best one
        for task in msg.tasks:
            if task.id in self.tasks:
                 none_id = task.id
                 continue
            # get the task with best bid
            makespan, schedule, start = self.calculate_bid_value(task)
            if makespan < lowest_cost:
                lowest_cost = makespan
                best_bid = makespan, schedule, task, start

        if best_bid is None:
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
        if winning_bid.auction_round != self.last_processed_round:
            return
        # check if you're the winner
        if winning_bid.bidder_id == self.robot_id:
            if self.active_bid is not None:
                makespan, schedule, task, start = self.active_bid
                self.my_schedule = schedule
                self.first_task = next(n for n, d in self.my_schedule.in_degree() if d == 0)
                self.tasks[task.id] = task
                self.active_bid = None
            else:
                # mora se updateat vrijeme i potencijalno pushat successore
                node = winning_bid.task_id
                node_data = self.my_schedule.nodes[node]
                node_data['start_time'] = winning_bid.task_start
                next_node = next(self.my_schedule.successors(node), None)
                while next_node is not None:
                    distance = self.my_schedule.edges[node, next_node]['distance']
                    next_node_data = self.my_schedule.nodes[next_node]
                    next_node_data['start_time'] = np.max(next_node_data['start_time'], node_data['start_time'] + node_data['task'].duration + distance/self.speed)
                    node = next_node
                    next_node = next(self.my_schedule.successors(node), None)



def main(args=None):
    ros.init(args=args)
    node = Bidder()
    ros.spin(node)
    node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
