#!/usr/bin/env python3
import rclpy as ros
import networkx as nx       #onaj library za rad s grafovima
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from auctioneer import Auctioneer

def main(args=None):
    ros.init(args=args)
    node = Auctioneer()
    ros.spin(node)
    node.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()
