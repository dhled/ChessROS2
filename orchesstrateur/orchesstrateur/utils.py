#!/usr/bin/env python3

from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)

from moveit_msgs.srv import (
    GetMotionPlan,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


# Create a joint constraint from target configuration and joints name.
# Tolerance is in radian. Default : 0.001
def create_joint_constraint(target_configuration, joints_name, tolerance=0.001):
    if len(target_configuration) != len(joints_name):
        raise Exception(
            "Target configuration size is not matching joints name list size."
        )

    constraint = Constraints()
    for joint_id, joint_name in enumerate(joints_name):
        js_constraint = JointConstraint()
        js_constraint.joint_name = joint_name
        js_constraint.position = target_configuration[joint_id]
        js_constraint.tolerance_below = tolerance
        js_constraint.tolerance_above = tolerance
        js_constraint.weight = 1.0
        constraint.joint_constraints.append(js_constraint)
    return constraint


# Create a Pose constraint from a geometry_msgs/Pose
# Base frame is the Pose reference frame
# End_effector_frame is the target position frame
# Tolerance is in meter. Default Tolerance 0.0001m


def create_pose_constraint(
    target_pose, base_frame, end_effector_frame, tolerance=0.0001
):
    if not isinstance(target_pose, Pose):
        raise Exception("Target Pose is not a geometry_msgs/Pose.")

    constraint = Constraints()
    # Position Constraint is a sphere of 0.0001m
    position_contraint = PositionConstraint()
    position_contraint.header.frame_id = base_frame
    position_contraint.link_name = end_effector_frame
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions.append(tolerance)
    sphere_pose = target_pose
    position_contraint.constraint_region.primitives.append(sphere)
    position_contraint.constraint_region.primitive_poses.append(sphere_pose)
    position_contraint.weight = 1.0
    constraint.position_constraints.append(position_contraint)

    # Orientation Constraint
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = base_frame
    orientation_constraint.link_name = end_effector_frame
    orientation_constraint.orientation = target_pose.orientation
    orientation_constraint.weight = 1.0
    constraint.orientation_constraints.append(orientation_constraint)

    return constraint

def create_motion_plan_request(
        target,
        movegroup,
        end_effector,
        ref_frame,
        max_vel_scaling=1.0,
        max_acc_scaling=1.0,
        joint_states=None,
    ):
        rqst = GetMotionPlan.Request()
        rqst.motion_plan_request.group_name = movegroup
        rqst.motion_plan_request.num_planning_attempts = 100
        rqst.motion_plan_request.allowed_planning_time = 10.0
        rqst.motion_plan_request.max_velocity_scaling_factor = max_vel_scaling
        rqst.motion_plan_request.max_acceleration_scaling_factor = max_acc_scaling
        rqst.motion_plan_request.start_state.joint_state = joint_states
        if isinstance(target, list):
            rqst.motion_plan_request.goal_constraints.append(create_joint_constraint(target, joint_states.name))
        elif isinstance(target, Pose):
            rqst.motion_plan_request.goal_constraints.append(create_pose_constraint(target, ref_frame, end_effector))
        else:
            raise Exception("Not Recognized Target - Accepted : list (articular motion) - geometry_msgs/Pose(cartesian motion)")
        return rqst