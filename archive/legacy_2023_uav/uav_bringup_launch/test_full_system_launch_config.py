"""Validate the installed full-system launch topology without starting hardware."""

import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "launch" / "full_system.launch.py"


def test_launch_file_is_valid_python_and_declares_gcs_device():
    text = SOURCE.read_text(encoding="utf-8")
    ast.parse(text)
    assert 'LaunchConfiguration("gcs_serial_device")' in text
    assert '"gcs_serial_device"' in text
    assert 'default_value="/dev/gcs_bluetooth"' in text
    assert 'package="gcs_serial_bridge"' in text
    assert 'executable="gcs_serial_bridge_node"' in text


def test_gcs_is_started_but_is_not_a_critical_control_node():
    text = SOURCE.read_text(encoding="utf-8")
    critical = text.split("critical_nodes = [", 1)[1].split("]", 1)[0]
    assert "gcs_bridge" not in critical
    assert "actions=[pid, camera_bridge, car_bridge, gcs_bridge]" in text


def test_flight_recorder_defaults_to_lightweight_mode():
    text = SOURCE.read_text(encoding="utf-8")
    assert (
        'DeclareLaunchArgument(\n                "enable_flight_recorder",\n'
        '                default_value="true",'
    ) in text
    assert (
        'DeclareLaunchArgument(\n                "record_lidar_scan",\n'
        '                default_value="false",'
    ) in text
    assert "(flight_recorder, \"flight_data_recorder\")" not in text


def test_full_system_has_a_process_lifetime_single_instance_lock():
    text = SOURCE.read_text(encoding="utf-8")
    assert "def _acquire_single_instance_lock():" in text
    assert "fcntl.LOCK_EX | fcntl.LOCK_NB" in text
    assert "_acquire_single_instance_lock()" in text
