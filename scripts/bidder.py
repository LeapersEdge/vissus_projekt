#!/usr/bin/env python3

"""TASK DESCRIPTION: Each bidder should know its initial position and set of capabilities (we assume that all robots are the same, have
the same speed, and can perform all tasks).

Upon receiving the list of tasks in the auction, the bidder tries to insert each task into its current schedule and
computes the corresponding total schedule duration (makespan). It then selects the task that would result in the
minimum makespan and submits it as a bid (task-makespan pair)."""

import rclpy as ros
from rclpy.node import Node
from vissus_projekt.msg import TaskList, Task, Bid


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
        self.my_schedule = []
        self.tasks = dict()
        self.last_processed_round = -1
        self.active_bid = None

        # 2. Publishers
        self.bid_publisher = self.create_publisher(Bid, 'bids', 10)

        # 3. Subscribers
        self.task_sub = self.create_subscription(
            TaskList, 'auction_task_list', self.task_callback, 10)

        self.winner_sub = self.create_subscription(
            Bid, 'auction_winner', self.winner_callback, 10)

    def calculate_bid_value(self, task):
        """Logic to determine the cost for a task"""
        # TODO: Implement calculation
        return (0, placement_in_schedule, start)

    def task_callback(self, msg):
        """Triggered when the Auctioneer publishes tasks"""
        if msg.auction_round <= self.last_processed_round:
            return

        self.last_processed_round = msg.auction_round
        lowest_cost = float("inf")
        bid = None

        # evaluate all tasks and pick the best one
        for task in msg.tasks:
            cost, placement_in_schedule, start = self.calculate_bid_value(task)
            if cost < lowest_cost:
                lowest_cost = cost
                bid = (task, placement_in_schedule, start)

        # make a bid
        task, placement_in_schedule, start = bid

        best_bid = Bid()
        best_bid.auction_round = msg.auction_round
        best_bid.bidder_id = self.robot_id
        best_bid.task_id = task.id
        best_bid.bid = lowest_cost
        best_bid.task_start = start

        self.active_bid = (task, placement_in_schedule, best_bid)
        self.bid_publisher.publish(best_bid)

    def winner_callback(self, winning_bid):
        """Triggered when the Auctioneer announces a winner."""
        if winning_bid.auction_round != self.last_processed_round:
            return
        # check if you're the winner
        if winning_bid.bidder_id == self.robot_id:
            task, placement_in_schedule, bid = self.active_bid
            # if winner place task in schedule
            if task.id not in self.tasks:
                self.tasks[task.id] = task
                self.my_schedule.insert(placement_in_schedule, task.id)
            else:
                self.tasks[task.id].earliest_start = task.earliest_start
                index = self.my_schedule.index(task.id)
                # ako updatean task nije zadnji u rasporedu trebaju se pushat ostali ako ih ne stigne obavit na vrijeme
                if index != len(self.my_schedule) - 1:
                    next_task_id = self.my_schedule[index + 1]
                    diff = self.tasks[task.id].earliest_start + self.tasks[task.id].duration - self.tasks[next_task_id].earliest_start
                    if diff > 0:
                        for task_id_to_pushback in self.my_schedule[index + 1:]:
                            self.tasks[task_id_to_pushback].earliest_start += diff

        self.active_bid = None


def main(args=None):
    ros.init(args=args)
    node = Bidder()
    ros.spin(node)
    node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
