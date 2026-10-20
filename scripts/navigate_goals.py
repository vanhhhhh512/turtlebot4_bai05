#!/usr/bin/env python3
"""Navigate a configured route; record the actual action results and ROS telemetry."""
import csv
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_dict(pose):
    p, q = pose.position, pose.orientation
    return {'x': p.x, 'y': p.y, 'yaw': math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))}


class RouteNavigator(BasicNavigator):
    def __init__(self):
        super().__init__('bai05_route_navigator')
        self.declare_parameter('output_dir', str(Path.home() / 'bai05_run'))
        self.declare_parameter('goals_file', '')
        self.declare_parameter('initialize', True)
        self.out = Path(self.get_parameter('output_dir').value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.config = yaml.safe_load(Path(self.get_parameter('goals_file').value).read_text())
        self.gt = self.amcl = self.odom = self.dock = None
        self.active_goal = 0
        self.contacts = self.bump_events = self.contact_messages = self.hazard_messages = 0
        self.events = (self.out / 'navigation_events.jsonl').open('a', buffering=1)
        self.telemetry_file = (self.out / 'trajectory.csv').open('w', buffering=1)
        self.writer = csv.writer(self.telemetry_file)
        self.writer.writerow(['wall_monotonic', 'sim_time', 'goal', 'amcl_stamp', 'amcl_x', 'amcl_y',
                              'amcl_yaw', 'gt_stamp', 'gt_x', 'gt_y', 'gt_yaw', 'vx', 'wz',
                              'contact_count', 'bump_events'])
        self.create_subscription(Odometry, '/sim_ground_truth_pose', self.on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(DockStatus, '/dock_status', self.on_dock, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bumper_contact', self.on_contact, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.on_hazard,
                                 qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/bai05/mission_status', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.create_timer(1.0, self.publish_goal_markers)
        self.create_timer(0.2, self.sample)
        self.undock = ActionClient(self, Undock, '/undock')

    def event(self, kind, **kwargs):
        item = {'event': kind, 'sim_time': self.get_clock().now().nanoseconds * 1e-9,
                'wall_monotonic': time.monotonic(), **kwargs}
        self.events.write(json.dumps(item) + '\n')
        self.get_logger().info(json.dumps(item))
        self.status_pub.publish(String(data=json.dumps(item)))

    def on_gt(self, msg): self.gt = msg
    def on_odom(self, msg): self.odom = msg
    def on_amcl(self, msg): self.amcl = msg
    def on_dock(self, msg): self.dock = msg

    def publish_goal_markers(self):
        markers = MarkerArray()
        for index, goal in enumerate(self.config['goals'], 1):
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns, text.id = 'goal_labels', index
            text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y = float(goal['x']), float(goal['y'])
            text.pose.position.z = .35
            text.pose.orientation.w = 1.0
            text.scale.z = .38
            text.color.r, text.color.g, text.color.b, text.color.a = 0.0, .15, .75, 1.0
            text.text = f'Goal {index}'
            markers.markers.append(text)
        self.markers_pub.publish(markers)

    def on_contact(self, msg):
        self.contact_messages += 1
        if msg.contacts:
            self.contacts += len(msg.contacts)
            self.event('BUMPER_CONTACT', goal=self.active_goal, count=len(msg.contacts))

    def on_hazard(self, msg):
        self.hazard_messages += 1
        bumps = sum(d.type == 1 for d in msg.detections)
        self.bump_events += bumps
        if bumps:
            self.event('BUMP_HAZARD', goal=self.active_goal, count=bumps)

    def sample(self):
        a = pose_dict(self.amcl.pose.pose) if self.amcl else dict(x='', y='', yaw='')
        g = pose_dict(self.gt.pose.pose) if self.gt else dict(x='', y='', yaw='')
        self.writer.writerow([time.monotonic(), self.get_clock().now().nanoseconds * 1e-9,
            self.active_goal, stamp_s(self.amcl.header.stamp) if self.amcl else '', *a.values(),
            stamp_s(self.gt.header.stamp) if self.gt else '', *g.values(),
            self.odom.twist.twist.linear.x if self.odom else '',
            self.odom.twist.twist.angular.z if self.odom else '', self.contacts, self.bump_events])

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def localization_error(self):
        """Compare fresh AMCL and Gazebo poses in this simulation's map frame."""
        if not self.amcl or not self.gt:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - stamp_s(self.amcl.header.stamp) > 3 or
                now - stamp_s(self.gt.header.stamp) > 3):
            return None
        a, g = pose_dict(self.amcl.pose.pose), pose_dict(self.gt.pose.pose)
        return math.hypot(a['x'] - g['x'], a['y'] - g['y'])

    def relocalize_from_gazebo(self, goal, error):
        """Simulation-only recovery: stop Nav2 before resetting a lost AMCL estimate."""
        self.event('LOCALIZATION_LOST', goal=goal, position_error_m=error,
                   amcl=pose_dict(self.amcl.pose.pose), gt=pose_dict(self.gt.pose.pose))
        self.cancelTask()
        deadline = time.monotonic() + 15
        while not self.isTaskComplete() and time.monotonic() < deadline:
            pass
        if not self.isTaskComplete():
            raise TimeoutError('Nav2 did not stop after localization was lost')
        initial = PoseStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = self.get_clock().now().to_msg()
        initial.pose = self.gt.pose.pose
        self.setInitialPose(initial)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            new_stamp = stamp_s(self.amcl.header.stamp) if self.amcl else 0
            new_error = self.localization_error()
            if new_stamp >= stamp_s(initial.header.stamp) and new_error is not None and new_error < 0.5:
                self.event('LOCALIZATION_RECOVERED', goal=goal,
                           position_error_m=new_error)
                return
        raise TimeoutError('AMCL did not converge after pose reset')

    def wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('Action response exceeded readiness timeout')
        return future.result()

    def initialize_robot(self):
        deadline = time.monotonic() + 120
        while rclpy.ok() and (self.gt is None or self.dock is None or
                self.get_clock().now().nanoseconds == 0):
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation clock, ground truth or dock status unavailable')
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.dock.is_docked:
            if not self.undock.wait_for_server(timeout_sec=30):
                raise TimeoutError('/undock is not available')
            handle = self.wait_future(self.undock.send_goal_async(Undock.Goal()), 30)
            if not handle.accepted:
                raise RuntimeError('Undock rejected')
            result = self.wait_future(handle.get_result_async(), 60)
            self.event('UNDOCK_RESULT', status=result.status)
            if result.status != 4:
                raise RuntimeError('Undock did not succeed')
        self.spin_for(2)
        # Simulation-only initial estimate. AMCL still uses scan/map to localize.
        initial = PoseStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = self.get_clock().now().to_msg()
        initial.pose = self.gt.pose.pose
        self.setInitialPose(initial)
        self.event('INITIAL_POSE_FROM_GAZEBO', pose=pose_dict(initial.pose))
        self.waitUntilNav2Active()
        self.spin_for(3)
        self.event('LOCALIZATION_READY', amcl=pose_dict(self.amcl.pose.pose),
                   gt=pose_dict(self.gt.pose.pose))

    def run_route(self):
        if self.get_parameter('initialize').value:
            self.initialize_robot()
        else:
            self.waitUntilNav2Active()
        results = []
        for index, goal in enumerate(self.config['goals'], 1):
            self.active_goal = index
            p = PoseStamped()
            p.header.frame_id = self.config.get('frame_id', 'map')
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = float(goal['x']), float(goal['y'])
            p.pose.orientation.z = math.sin(float(goal['yaw']) / 2)
            p.pose.orientation.w = math.cos(float(goal['yaw']) / 2)
            before_contact, before_bump = self.contacts, self.bump_events
            wall_start = time.monotonic()
            sim_start = self.get_clock().now().nanoseconds * 1e-9
            self.event('GOAL_SENT', goal=index, target=goal)
            relocalizations = 0
            while True:
                p.header.stamp = self.get_clock().now().to_msg()
                accepted = self.goToPose(p)
                if not accepted:
                    self.event('GOAL_REJECTED', goal=index)
                    raise RuntimeError(f'Goal {index} rejected')
                retry = False
                while rclpy.ok() and not self.isTaskComplete():
                    error = self.localization_error()
                    if error is not None and error > 1.0:
                        if relocalizations >= 2:
                            self.cancelTask()
                            raise RuntimeError(f'Goal {index}: AMCL lost localization repeatedly')
                        self.relocalize_from_gazebo(index, error)
                        relocalizations += 1
                        retry = True
                        break
                    if time.monotonic() - wall_start > self.config.get('goal_timeout_sec', 240):
                        self.event('GOAL_TIMEOUT', goal=index)
                        self.cancelTask()
                        while not self.isTaskComplete():
                            pass
                        break
                if not retry:
                    break
            result = self.getResult()
            self.spin_for(0.5)
            a, g = pose_dict(self.amcl.pose.pose), pose_dict(self.gt.pose.pose)
            record = {'goal': index, 'target': goal, 'result': result.name,
                'sim_duration_s': self.get_clock().now().nanoseconds * 1e-9 - sim_start,
                'wall_duration_s': time.monotonic() - wall_start, 'final_amcl': a,
                'final_gt': g, 'amcl_stamp': stamp_s(self.amcl.header.stamp),
                'gt_stamp': stamp_s(self.gt.header.stamp),
                'amcl_goal_distance_m': math.hypot(a['x']-goal['x'], a['y']-goal['y']),
                'contact_count': self.contacts-before_contact, 'bump_events': self.bump_events-before_bump,
                'relocalizations': relocalizations,
                'contact_messages_received': self.contact_messages,
                'hazard_messages_received': self.hazard_messages}
            results.append(record)
            (self.out / 'goal_results.json').write_text(json.dumps(results, indent=2))
            self.event('GOAL_RESULT', **record)
            if result != TaskResult.SUCCEEDED or record['contact_count'] or record['bump_events']:
                raise RuntimeError(f'Goal {index} failed acceptance: {record}')
            self.active_goal = 0
            self.spin_for(self.config.get('dwell_sec', 6))
        self.event('MISSION_COMPLETE', goals=len(results), contact_count=self.contacts,
                   bump_events=self.bump_events)


def main():
    rclpy.init()
    node = RouteNavigator()
    try:
        node.run_route()
    except KeyboardInterrupt:
        if node.result_future and not node.result_future.done():
            node.cancelTask()
    except Exception as exc:
        node.event('MISSION_ERROR', error=str(exc))
        raise
    finally:
        node.events.close()
        node.telemetry_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
