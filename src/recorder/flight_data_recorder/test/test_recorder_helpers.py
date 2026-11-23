import math

from geometry_msgs.msg import PoseStamped, TwistStamped

from flight_data_recorder.recorder_node import finite, pose_dict, twist_dict


def test_finite_rejects_non_finite_json_values():
    assert finite(1.25) == 1.25
    assert finite(math.nan) is None
    assert finite(math.inf) is None


def test_pose_and_twist_use_explicit_units():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.2
    pose.pose.position.y = -0.65
    assert pose_dict(pose)["y_m"] == -0.65

    twist = TwistStamped()
    twist.header.frame_id = "base_link"
    twist.twist.linear.y = -0.2
    twist.twist.angular.z = 0.1
    result = twist_dict(twist)
    assert result["vy_m_s"] == -0.2
    assert result["yaw_rate_rad_s"] == 0.1
