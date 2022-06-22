import rclpy
from rclpy.node import Node
import copy

from chess_msgs.msg import ChessMove
from moveit_msgs.srv import GetMotionPlan, GetCartesianPath
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import threading
from orchesstrateur.utils import create_motion_plan_request
import time
import math


mapping_col = {"a": 0, "b": 1, "c": 2, "d": 3, "e": 4, "f": 5, "g": 6, "h": 7}


class Board(object):
    def __init__(self):
        self._square_size = 0.02
        self._pose_a1 = Pose()
        self._pose_a1.position.x = 0.18730
        self._pose_a1.position.y = 0.81465
        self._pose_a1.position.z = 0.290
        self._pose_a1.orientation.x = -0.7081662592767818
        self._pose_a1.orientation.y = 0.017245045458809432
        self._pose_a1.orientation.z = -0.02146781882332226
        self._pose_a1.orientation.w = 0.7055085331759088
        a1 = (0.187, 0.824)
        h1 = (-0.097, 0.824)
        a8 = (0.187, 0.539)
        self._dir_col = (-1.0, 0)  # ((h1[0] - a1[0]), (h1[1] - a1[1]))
        self._dir_row = (0.0, -1.0)  # ((a8[0] - a1[0]), (a8[1] - a1[1]))

    def get_pose_square(self, square="a1", z_offset=0.1):
        offset_x = (float(mapping_col[square[0]])) * self._dir_col[0] * self._square_size + (float(square[1]) - 1) * self._dir_row[
            0
        ] * self._square_size
        offset_y = (float(mapping_col[square[0]])) * self._dir_col[1] * self._square_size + (float(square[1]) - 1) * self._dir_row[
            1
        ] * self._square_size
        pose_target = Pose()
        pose_target.position.x = self._pose_a1.position.x + offset_x
        pose_target.position.y = self._pose_a1.position.y + offset_y
        pose_target.position.z = self._pose_a1.position.z + z_offset
        pose_target.orientation = self._pose_a1.orientation
        return pose_target


class orchestrateur_node(Node):
    def __init__(self):
        super().__init__("orchestrateur_node")
        self.board = Board()
        self._robot_moved_pub = self.create_publisher(Empty, "robot_moved", 1)
        self._phidget_pub = self.create_publisher(Bool, "/digital_output07", 1)

        self._move_robot_ac = ActionClient(self, FollowJointTrajectory, "/iiwa_arm_controller/follow_joint_trajectory")
        self._cart_planner_rqst = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._motion_planner_rqst = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self._last_js = None
        self._drop_pose = Pose()
        self._drop_pose.position.x = -0.18730
        self._drop_pose.position.y = 0.62465
        self._drop_pose.position.z = 0.350
        self._drop_pose.orientation.x = -0.7081662592767818
        self._drop_pose.orientation.y = 0.017245045458809432
        self._drop_pose.orientation.z = -0.02146781882332226
        self._drop_pose.orientation.w = 0.7055085331759088
        self._is_moving = False

        self._played_move_sub = self.create_subscription(ChessMove, "played_move", self._start_robot, 1)

    def _start_robot(self, move):
        if self._is_moving == False:
            if move.type == "capture" or move.type == "en_passant":
                self._thread = threading.Thread(target=self.capture, args=[move.move])
            elif move.type == "roque":
                self._thread = threading.Thread(target=self.roque, args=[move.move])
            else:
                self._thread = threading.Thread(target=self.pick_and_drop, args=[move.move])
            self._thread.start()
        self._is_moving = True

    def _js_cb(self, msg):
        js = JointState()
        js.header.frame_id = "world"
        js.name = msg.name
        js.position = msg.position
        self._last_js = js

    def get_motion_plan_to_path(self, pose):
        js = copy.deepcopy(self._last_js)
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rqst = create_motion_plan_request(
            movegroup="iiwa_arm", end_effector="tool0", ref_frame="world", target=pose, max_vel_scaling=0.1, max_acc_scaling=0.1, joint_states=js
        )
        self._motion_planner_rqst.wait_for_service()
        rsp = self._motion_planner_rqst.call(rqst)
        print(rsp)
        input()
        return rsp.solution

    def get_plan_to_path(self, pose):
        rqst = GetCartesianPath.Request()
        rqst.header.frame_id = "world"
        rqst.header.stamp = self.get_clock().now().to_msg()
        rqst.link_name = "tool0"
        rqst.group_name = "iiwa_arm"
        rqst.waypoints = [pose]
        rqst.max_step = 0.01
        rqst.jump_threshold = 0.0
        rqst.avoid_collisions = False
        rqst.prismatic_jump_threshold = 0.0
        rqst.revolute_jump_threshold = 0.0
        fraction = 0.0

        while fraction < 0.98:
            self._cart_planner_rqst.wait_for_service()
            rqst.start_state.joint_state = self._last_js
            rqst.start_state.joint_state.velocity = []
            rqst.start_state.joint_state.effort = []
            rsp = self._cart_planner_rqst.call(rqst)
            fraction = rsp.fraction
            if not rclpy.ok():
                return None
        return rsp.solution

    def execute_trajectory(self, plan):
        self._result = None
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = plan.joint_trajectory
        self._move_robot_ac.wait_for_server()
        self._move_robot_ac.send_goal(goal)
        return True

    def move_to(self, square, z_offset=0.05):
        pose = self.board.get_pose_square(square, z_offset)
        plan = self.get_plan_to_path(pose)
        if plan is not None:
            self.execute_trajectory(plan)

    def turn_off_ea(self):
        rqst = Bool()
        rqst.data = False
        self._phidget_pub.publish(rqst)

    def turn_on_ea(self):
        rqst = Bool()
        rqst.data = True
        self._phidget_pub.publish(rqst)

    def pick(self, square):
        self.move_to(square, 0.05)
        self.turn_off_ea()
        self.move_to(square, 0.00)
        self.turn_on_ea()
        self.move_to(square, 0.05)

    def drop(self, square):
        self.move_to(square, 0.05)
        self.move_to(square, 0.01)
        self.turn_off_ea()
        self.move_to(square, 0.05)

    def drop_capture(self):
        plan = self.get_plan_to_path(self._drop_pose)
        if plan is not None:
            self.execute_trajectory(plan)
        self.turn_off_ea()

    def pick_and_drop(self, move):
        print(move)
        pick_sq = move[:2]
        drop_sq = move[2:]
        self.pick(pick_sq)
        self.drop(drop_sq)
        self._is_moving = False
        msg = Empty()
        self._robot_moved_pub.publish(msg)

    def capture(self, move):
        pick_sq = move[:2]
        drop_sq = move[2:]
        self.pick(drop_sq)
        self.drop_capture()
        self.pick(pick_sq)
        self.drop(drop_sq)
        self._is_moving = False
        msg = Empty()
        self._robot_moved_pub.publish(msg)

    def roque(self, move):
        if move == "e1g1":
            self.pick_and_drop("h1f1")
            self.pick_and_drop(move)
        elif move == "e1c1":
            self.pick_and_drop("a1d1")
            self.pick_and_drop(move)
        elif move == "e8g8":
            self.pick_and_drop("h8f8")
            self.pick_and_drop(move)
        elif move == "e8c8":
            self.pick_and_drop("a8d8")
            self.pick_and_drop(move)

        self._is_moving = False
        msg = Empty()
        self._robot_moved_pub(msg)


def main(args=None):
    rclpy.init(args=args)

    orchestrateur = orchestrateur_node()
    rclpy.spin(orchestrateur)
    orchestrateur.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
