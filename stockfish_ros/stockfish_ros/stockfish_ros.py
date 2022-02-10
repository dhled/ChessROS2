#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from chess_msgs.srv import GetNextMove
import stockfish


dft_cfg = {
    "Write Debug Log": "false",
    "Contempt": 0,
    "Min Split Depth": 0,
    "Threads": 1,
    "Ponder": "false",
    "Hash": 16,
    "MultiPV": 1,
    "Skill Level": 20,
    "Move Overhead": 30,
    "Minimum Thinking Time": 20,
    "Slow Mover": 80,
    "UCI_Chess960": "false",
}


class StockFishROS(Node):
    def __init__(self, node_name="stockfish_node"):
        super().__init__(node_name)
        self._stockfish = stockfish.Stockfish()
        self._next_move_pub = self.create_publisher(String, "next_move", 10)
        self._get_move_played = self.create_subscription(String, "played_move", self._move_played_cb, 10)
        self._get_next_move_srv = self.create_service(GetNextMove, "get_next_move", self._get_next_move_cb)

    def _move_played_cb(self, msg):
        self._stockfish.make_moves_from_current_position([msg.data])

    def _get_next_move_cb(self, _, response):
        response.move = self._stockfish.get_best_move_time(1000)


def main(args=None):
    rclpy.init(args=args)

    stockfish_node = StockFishROS()
    try:
        rclpy.spin(stockfish_node)
    finally:
        stockfish_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
