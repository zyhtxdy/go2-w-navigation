"""Actual observation-only ROS nodes; synthetic points, no robot or live bag."""

import atexit
import hashlib
import math
import os
from pathlib import Path
import shutil
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest


# The test process itself and its child nodes use a private loopback domain.
# No machine-wide network configuration or existing ROS process is changed.
os.environ["ROS_DOMAIN_ID"] = "84"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
for _name in (
    "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
    "ROS_DISCOVERY_SERVER", "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS",
):
    os.environ.pop(_name, None)

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


def _transform(x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    value = np.eye(4)
    value[:3, :3] = ((c, -s, 0), (s, c, 0), (0, 0, 1))
    value[:3, 3] = (x, y, z)
    return value


def _plane(points, origin, first_axis, second_axis, first_length, second_length):
    first_count = math.ceil(first_length / 0.24)
    second_count = math.ceil(second_length / 0.24)
    origin, first_axis, second_axis = map(np.asarray, (origin, first_axis, second_axis))
    for first in range(first_count + 1):
        for second in range(second_count + 1):
            points.append(origin + first_axis * (first_length * first / first_count)
                          + second_axis * (second_length * second / second_count))


def _box(points, xmin, xmax, ymin, ymax, zmin, zmax):
    for x in (xmin, xmax):
        _plane(points, (x, ymin, zmin), (0, 1, 0), (0, 0, 1), ymax - ymin, zmax - zmin)
    for y in (ymin, ymax):
        _plane(points, (xmin, y, zmin), (1, 0, 0), (0, 0, 1), xmax - xmin, zmax - zmin)
    _plane(points, (xmin, ymin, zmax), (1, 0, 0), (0, 1, 0), xmax - xmin, ymax - ymin)


def _scene():
    # Same geometric fixture as test_registration.cpp, not a captured map.
    points = []
    _plane(points, (-3.4, -2.3, -0.5), (1, 0, 0), (0, 1, 0), 7.6, 5.0)
    _plane(points, (-3.4, -2.3, -0.5), (1, 0, 0), (0, 0, 1), 7.6, 2.3)
    _plane(points, (-3.4, 2.7, -0.5), (1, 0, 0), (0, 0, 1), 7.6, 2.3)
    _plane(points, (4.2, -2.3, -0.5), (0, 1, 0), (0, 0, 1), 5.0, 2.3)
    _plane(points, (-3.4, 1.2, -0.5), (0, 1, 0), (0, 0, 1), 1.5, 1.5)
    _box(points, 0.9, 1.5, 1.1, 1.8, -0.5, 2.1)
    _box(points, -2.0, -1.3, 0.2, 0.9, -0.5, 0.8)
    _box(points, 2.4, 3.1, -1.6, -0.7, -0.5, 0.35)
    return np.asarray(points, dtype=np.float32)


def _write_pcd(path, points):
    with path.open("w", encoding="ascii") as output:
        output.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                     "TYPE F F F\nCOUNT 1 1 1\nWIDTH {}\nHEIGHT 1\n"
                     "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA ascii\n".format(
                         len(points), len(points)))
        np.savetxt(output, points, fmt="%.9g")


@pytest.mark.launch_test
def generate_test_description():
    temporary = Path(tempfile.mkdtemp(prefix="go2_relocalization_ros_test_"))
    atexit.register(shutil.rmtree, temporary, ignore_errors=True)
    database = temporary / "known_database"
    (database / "keyframes").mkdir(parents=True)
    points = _scene()
    keyframe_pose = _transform(12.0, -5.0, 0.5, 0.17)
    _write_pcd(database / "keyframes" / "junction.pcd", points)
    half_yaw = 0.17 / 2
    (database / "manifest.csv").write_text(
        "id,x,y,z,qx,qy,qz,qw\n"
        "junction,12,-5,0.5,0,0,{:.17g},{:.17g}\n".format(
            math.sin(half_yaw), math.cos(half_yaw)), encoding="ascii")
    output_directory = temporary / "recorded_database"
    common = {
        "cloud_topic": "/offline_relocalization/cloud",
        "odometry_topic": "/offline_relocalization/odom",
        "body_frame": "body", "odom_frame": "odom",
    }
    localization = launch_ros.actions.Node(
        package="go2_relocalization", executable="relocalization_node",
        name="relocalization_test", namespace="/offline_relocalization",
        output="screen", parameters=[dict(common, **{
            "database_directory": str(database), "map_frame": "map",
            "publish_tf": False, "publish_known_map": True, "processing_interval": 0.1,
            "confirmation_frames": 3, "confirmation_interval": 0.2,
            "max_input_age": 3.0, "tracking_timeout": 1.2,
        })])
    recorder = launch_ros.actions.Node(
        package="go2_relocalization", executable="database_recorder",
        name="recorder_test", namespace="/offline_relocalization",
        output="screen", parameters=[dict(common, output_directory=str(output_directory))])
    return launch.LaunchDescription([
        localization, recorder, launch_testing.actions.ReadyToTest(),
    ]), {
        "localization": localization, "recorder": recorder,
        "temporary_directory": temporary, "output_directory": output_directory,
        "scene": points, "keyframe_pose": keyframe_pose,
    }


class TestObservationOnlyNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("shadow_test_client", namespace="/offline_relocalization")
        self.messages = {"valid": [], "pose": [], "transform": [], "status": [],
                         "cloud": [], "known_map": []}
        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, "/offline_relocalization/cloud", sensor_qos)
        self.odom_pub = self.node.create_publisher(
            Odometry, "/offline_relocalization/odom", sensor_qos)
        prefix = "/offline_relocalization/relocalization_test"
        self.subscriptions = [
            self.node.create_subscription(Bool, prefix + "/valid",
                                          lambda msg: self.messages["valid"].append(msg), state_qos),
            self.node.create_subscription(DiagnosticArray, prefix + "/status",
                                          lambda msg: self.messages["status"].append(msg), state_qos),
            self.node.create_subscription(PoseStamped, prefix + "/pose",
                                          lambda msg: self.messages["pose"].append(msg), 20),
            self.node.create_subscription(TransformStamped, prefix + "/map_to_odom",
                                          lambda msg: self.messages["transform"].append(msg), 20),
            self.node.create_subscription(PointCloud2, prefix + "/aligned_cloud",
                                          lambda msg: self.messages["cloud"].append(msg), sensor_qos),
            self.node.create_subscription(PointCloud2, prefix + "/known_map",
                                          lambda msg: self.messages["known_map"].append(msg), state_qos),
        ]
        self.reset_client = self.node.create_client(Trigger, prefix + "/relocalize")
        self.save_client = self.node.create_client(
            Trigger, "/offline_relocalization/recorder_test/save")

    def tearDown(self):
        self.node.destroy_node()

    def _spin(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def _wait(self, condition, timeout, message):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail(message + "; last status=" + str(self._status()))

    def _status(self):
        if not self.messages["status"]:
            return {}
        status = self.messages["status"][-1].status[0]
        return dict({"state": status.message}, **{item.key: item.value for item in status.values})

    def _valid(self):
        return bool(self.messages["valid"] and self.messages["valid"][-1].data)

    def _pair(self, points, odom_pose, stamp=None, frame="body"):
        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = frame
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=name, offset=4 * index,
                                  datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate(("x", "y", "z"))]
        cloud.is_bigendian = False
        cloud.point_step, cloud.row_step = 12, 12 * len(points)
        cloud.is_dense = True
        cloud.data = np.asarray(points, dtype="<f4").tobytes()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id, odometry.child_frame_id = "odom", "body"
        odometry.pose.pose.position.x = float(odom_pose[0, 3])
        odometry.pose.pose.position.y = float(odom_pose[1, 3])
        odometry.pose.pose.position.z = float(odom_pose[2, 3])
        yaw = math.atan2(odom_pose[1, 0], odom_pose[0, 0])
        odometry.pose.pose.orientation.z = math.sin(yaw / 2)
        odometry.pose.pose.orientation.w = math.cos(yaw / 2)
        return odometry, cloud

    def _publish_pair(self, pair):
        self.odom_pub.publish(pair[0])
        self._spin(0.025)
        self.cloud_pub.publish(pair[1])

    def _confirm(self, points, odom_pose, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not self._valid():
            self._publish_pair(self._pair(points, odom_pose))
            self._spin(0.35)
        self.assertTrue(self._valid(), "Distinct consistent observations did not confirm: "
                        + str(self._status()))
        self._wait(lambda: bool(self.messages["pose"] and self.messages["transform"]),
                   2.0, "Accepted result did not publish its pose and transform")

    def _service(self, client):
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))
        future = client.call_async(Trigger.Request())
        self._wait(future.done, 5.0, "Service response timed out")
        return future.result()

    @staticmethod
    def _matrix(position, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        value = np.eye(4)
        value[:3, :3] = (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )
        value[:3, 3] = (position.x, position.y, position.z)
        return value

    def _assert_pose(self, actual, expected):
        self.assertLess(np.linalg.norm(actual[:3, 3] - expected[:3, 3]), 0.12)
        relative = expected[:3, :3].T @ actual[:3, :3]
        angle = math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
        self.assertLess(angle, 0.06)

    @staticmethod
    def _xyz(cloud):
        offsets = {field.name: field.offset for field in cloud.fields}
        dtype = np.dtype({"names": ("x", "y", "z"), "formats": ("<f4",) * 3,
                          "offsets": tuple(offsets[name] for name in ("x", "y", "z")),
                          "itemsize": cloud.point_step})
        records = np.ndarray(shape=(cloud.height, cloud.width), dtype=dtype,
                             buffer=memoryview(cloud.data),
                             strides=(cloud.row_step, cloud.point_step))
        return np.column_stack([records[name].reshape(-1) for name in ("x", "y", "z")])

    def test_actual_nodes_confirm_timeout_reset_and_never_publish_control(
            self, scene, keyframe_pose, output_directory):
        self._wait(lambda: self.cloud_pub.get_subscription_count() >= 2
                   and self.odom_pub.get_subscription_count() >= 2, 8.0,
                   "Both actual nodes did not subscribe")
        self._wait(lambda: bool(self.messages["valid"]), 3.0, "No startup validity state")
        self.assertFalse(self._valid())
        keyframe_query = _transform(0.22, -0.14, 0, 2 * math.pi / 3)
        query_inverse = np.linalg.inv(keyframe_query)
        query_points = scene @ query_inverse[:3, :3].T + query_inverse[:3, 3]
        odom_pose = _transform(2, -1, 0.4, -0.2)
        expected_map_pose = keyframe_pose @ keyframe_query

        single_frame = self._pair(query_points, odom_pose)
        self._publish_pair(single_frame)
        self._wait(lambda: float(self._status().get("global_registration_ms", "0")) > 0,
                   8.0, "Background global search did not finish")
        self.assertFalse(self._valid(), "A single globally matched scan bypassed multi-frame confirmation")
        self.assertEqual(len(self.messages["pose"]), 0)
        self._wait(lambda: self._status().get("confirmation_count") == "1", 8.0,
                   "First geometric match never entered CONFIRMING")
        for _ in range(5):
            self._publish_pair(single_frame)
            self._spin(0.23)
        self.assertFalse(self._valid(), "Duplicate timestamps incorrectly confirmed a pose")
        self.assertLessEqual(int(self._status().get("confirmation_count", "0")), 1)
        self.assertFalse(any(msg.data for msg in self.messages["valid"]))
        self.assertEqual(len(self.messages["pose"]), 0)

        self._confirm(query_points, odom_pose)
        diagnostics = self._status()
        for key in ("global_result_status", "global_result_outcome", "global_job_stage",
                    "global_result_input_stamp", "global_result_input_age_s",
                    "global_result_job_ms", "global_cycle_ms", "global_result_accepted",
                    "global_result_rmse_m", "global_result_overlap",
                    "global_result_quality_score", "global_result_score_gap",
                    "global_result_normal_information_ratio", "global_verification_age_s"):
            self.assertIn(key, diagnostics, "Missing persistent global-job evidence: " + key)
        self.assertGreater(float(diagnostics["global_result_input_stamp"]), 0)
        self.assertLessEqual(float(diagnostics["global_result_input_age_s"]), 3.0)
        self.assertGreater(float(diagnostics["global_result_job_ms"]), 0)
        self.assertEqual(diagnostics["global_result_accepted"], "true")
        published_pose = self.messages["pose"][-1]
        published_transform = self.messages["transform"][-1]
        self.assertEqual(published_pose.header.frame_id, "map")
        self.assertEqual(published_transform.header.frame_id, "map")
        self.assertEqual(published_transform.child_frame_id, "odom")
        map_pose = self._matrix(published_pose.pose.position, published_pose.pose.orientation)
        map_odom = self._matrix(published_transform.transform.translation,
                                published_transform.transform.rotation)
        self._assert_pose(map_pose, expected_map_pose)
        self._assert_pose(map_odom, expected_map_pose @ np.linalg.inv(odom_pose))
        self._assert_pose(map_odom @ odom_pose, map_pose)
        self._wait(lambda: bool(self.messages["cloud"]), 2.0, "No aligned map cloud")
        aligned = self.messages["cloud"][-1]
        self.assertEqual(aligned.header.frame_id, "map")
        self.assertGreater(aligned.width * aligned.height, 100)
        self.assertLessEqual(aligned.width * aligned.height, 6000)
        self._wait(lambda: bool(self.messages["known_map"]), 2.0, "No persistent known-map cloud")
        known = self.messages["known_map"][-1]
        self.assertEqual(known.header.frame_id, "map")
        self.assertGreater(known.width * known.height, 100)
        aligned_points, known_points = self._xyz(aligned), self._xyz(known)
        self.assertTrue(np.isfinite(aligned_points).all())
        self.assertTrue(np.isfinite(known_points).all())
        samples = aligned_points[np.linspace(0, len(aligned_points) - 1, 25, dtype=int)]
        # Independent visualization oracle: decoded registered points should
        # physically coincide with the decoded persistent map, including PCL's
        # padded PointXYZ layout. Small sample count keeps this bounded.
        distances = np.linalg.norm(known_points[:, None, :] - samples[None, :, :], axis=2)
        nearest = distances.min(axis=0)
        self.assertLess(float(nearest.mean()), 0.12)
        self.assertLess(float(nearest.max()), 0.30)

        # Inspect ownership of endpoints, not whether some unrelated participant
        # happens to publish a similarly named topic in the ROS graph.
        for name in ("relocalization_test", "recorder_test"):
            publishers = dict(self.node.get_publisher_names_and_types_by_node(
                name, "/offline_relocalization"))
            for forbidden in ("/tf", "/tf_static", "/cmd_vel", "/api/sport/request",
                              "/go2/nav/cmd_vel", "/initialpose"):
                self.assertNotIn(forbidden, publishers)
        publishers = dict(self.node.get_publisher_names_and_types_by_node(
            "relocalization_test", "/offline_relocalization"))
        self.assertIn("/offline_relocalization/relocalization_test/map_to_odom", publishers)
        self.assertIn("/offline_relocalization/relocalization_test/known_map", publishers)

        saved = self._service(self.save_client)
        self.assertTrue(saved.success, saved.message)
        manifest = (output_directory / "manifest.csv").read_text(encoding="ascii").splitlines()
        self.assertEqual(len(manifest), 2, "Repeated stationary scans created extra keyframes")
        fields = manifest[1].split(",")
        self.assertEqual(fields[0], "k000000")
        np.testing.assert_allclose(list(map(float, fields[1:4])), odom_pose[:3, 3], atol=1e-8)
        self.assertTrue((output_directory / "keyframes" / "k000000.pcd").is_file())
        map_file = output_directory / "map.pcd"
        before = hashlib.sha256(map_file.read_bytes()).hexdigest()
        second_save = self._service(self.save_client)
        self.assertFalse(second_save.success, "Recorder overwrote an exported database")
        self.assertEqual(hashlib.sha256(map_file.read_bytes()).hexdigest(), before)

        self._wait(lambda: not self._valid() and self._status().get("reason")
                   == "tracking_timeout", 4.0, "Stopped input retained a valid pose")
        self.assertEqual(self._status().get("reason"), "tracking_timeout")
        reset = self._service(self.reset_client)
        self.assertTrue(reset.success, reset.message)
        self._spin(0.1)
        self.assertFalse(self._valid())
        self.assertEqual(self._status().get("confirmation_count"), "0")
        self._confirm(query_points, odom_pose)
        self._spin(0.3)

        # A moved robot sees the same mapped environment from a > 1 m shifted,
        # opposite-facing pose, while its old odometry remains unchanged. The
        # periodic global verification must reject the old hypothesis and find
        # the actual pose. Keep publishing fresh input: absence of input is not
        # a valid test of detecting a moved robot. Do not call the reset service.
        moved_query = _transform(1.2, 0.3, 0, -math.pi / 3)
        moved_inverse = np.linalg.inv(moved_query)
        moved_points = scene @ moved_inverse[:3, :3].T + moved_inverse[:3, 3]
        self.assertGreater(np.linalg.norm(moved_query[:3, 3] - keyframe_query[:3, 3]), 1.0)
        rejection_reasons = {
            "low_quality", "low_overlap", "high_rmse", "degenerate_geometry",
            "registration_failed", "tracking_correction_jump", "ambiguous_match",
        }
        status_start = len(self.messages["status"])
        observed_rejection = False
        recovery_deadline = time.monotonic() + 15.0
        recovered = False
        while time.monotonic() < recovery_deadline:
            self._publish_pair(self._pair(moved_points, odom_pose))
            self._spin(0.35)
            for message in self.messages["status"][status_start:]:
                status = message.status[0]
                values = {item.key: item.value for item in status.values}
                if values.get("valid") == "false" and values.get("reason") in rejection_reasons:
                    observed_rejection = True
            if observed_rejection and self._valid() and self.messages["pose"]:
                latest = self.messages["pose"][-1].pose
                latest_matrix = self._matrix(latest.position, latest.orientation)
                expected_moved = keyframe_pose @ moved_query
                translation_error = np.linalg.norm(latest_matrix[:3, 3] - expected_moved[:3, 3])
                relative = expected_moved[:3, :3].T @ latest_matrix[:3, :3]
                angle_error = math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
                if translation_error < 0.12 and angle_error < 0.06:
                    recovered = True
                    break
        self.assertTrue(observed_rejection,
                        "Fresh moved input never invalidated the old hypothesis: " + str(self._status()))
        self.assertTrue(recovered,
                        "Continuous moved input did not recover its real pose: " + str(self._status()))
        for message in self.messages["status"][status_start:]:
            values = {item.key: item.value for item in message.status[0].values}
            self.assertNotEqual(values.get("reason"), "tracking_timeout",
                                "Input timeout was mistaken for detecting the moved robot")
        moved_pose = self.messages["pose"][-1]
        self._assert_pose(self._matrix(moved_pose.pose.position, moved_pose.pose.orientation),
                          keyframe_pose @ moved_query)
        moved_transform = self.messages["transform"][-1].transform
        self._assert_pose(self._matrix(moved_transform.translation, moved_transform.rotation),
                          keyframe_pose @ moved_query @ np.linalg.inv(odom_pose))
        self._spin(0.3)

        invalid_pair = self._pair(moved_points, odom_pose, frame="hesai_lidar")
        pose_count_before_invalid = len(self.messages["pose"])
        invalid_stamp = (invalid_pair[1].header.stamp.sec * 1000000000
                         + invalid_pair[1].header.stamp.nanosec)
        self._publish_pair(invalid_pair)
        self._wait(lambda: not self._valid() and self._status().get("reason")
                   == "invalid_cloud_frame_stamp_or_layout", 2.0,
                   "Wrong input frame did not invalidate localization")
        self._spin(0.5)
        self.assertFalse(self._valid())
        for message in self.messages["pose"][pose_count_before_invalid:]:
            stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            self.assertLess(stamp, invalid_stamp,
                            "Rejected frame or later input emitted a pose")
        print("SHADOW_TEST_COMPLETE: unknown120deg; distinct-frame confirmation; "
              "pose/map_to_odom truth; aligned-cloud/known-map nearest points; "
              "recorder export; stale timeout; reset; "
              "moved>1m+180deg automatic recovery with unchanged odometry; "
              "bad frame rejected; no owned TF/control publishers; robot_runs=0", flush=True)


@launch_testing.post_shutdown_test()
class TestProcessShutdown(unittest.TestCase):
    def test_nodes_exit_cleanly(self, proc_info, temporary_directory):
        try:
            launch_testing.asserts.assertExitCodes(proc_info)
        finally:
            shutil.rmtree(temporary_directory, ignore_errors=True)
