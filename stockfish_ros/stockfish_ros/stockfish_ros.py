#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from chess_msgs.srv import GetNextMove, SetEloRating
from std_srvs.srv import Empty
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
        self._stockfish = stockfish.Stockfish(depth=18)
        self._get_move_played = self.create_subscription(String, "played_move", self._move_played_cb, 10)
        self._get_next_move_srv = self.create_service(GetNextMove, "get_next_move", self._get_next_move_cb)
        self._set_skill_level_srv = self.create_service(SetEloRating, "set_elo_rating", self._set_elo_rating)
        self._reset_game = self.create_service(Empty, "reset_game", self._reset_game)

    def _move_played_cb(self, msg):
        self.get_logger().info("Received move %s" % msg.data)
        self._stockfish.make_moves_from_current_position([msg.data])

    def _get_next_move_cb(self, _, response):
        move = self._stockfish.get_best_move_time(1000)
        self.get_logger().info("My next move %s" % move)
        response.move = move
        return response

    def _set_elo_rating(self, request, response):
        self.get_logger().info("Elo Rating %s" % request.elo_rating)
        self._stockfish.set_elo_rating(request.elo_rating)
        response.success = True
        return response

    def _reset_game(self, _, response):
        self.get_logger().info("Reset")
        self._stockfish.set_position([""])
        return response


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
