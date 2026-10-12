"""Task 4: variant 19 local ENU waypoint flight followed by automatic landing."""
import math
import time
from collections import deque
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from .flight_common import FlightNode, POSITION_QOS, spin_flight_node


class CustomFlightNode(FlightNode):
    # Variant 19 from the method sheet: target = start + (8.2, -0.4) m.
    DELTA_X, DELTA_Y = 8.2, -0.4
    POSITION_TOLERANCE, ALTITUDE_TOLERANCE = 0.20, 0.25
    MAX_HORIZONTAL_SPEED, MAX_VERTICAL_SPEED = 0.45, 0.35
    TAKEOFF_ALTITUDE_TOLERANCE = 0.3
    TAKEOFF_TIMEOUT_SECONDS = 25.0

    def __init__(self):
        super().__init__('custom_flight')
        self.pose = self.home = self.target = self.reached_at = None
        self.position_samples = deque(maxlen=20)
        self.pose_received_at = self.pose_stale_since = None
        self.takeoff_started = None
        self.takeoff_complete = False
        self.last_climb_log = 0.0
        self.pose_sub = self.create_subscription(
            PoseStamped, '/ap/v1/pose/filtered', self._pose_cb, POSITION_QOS)
        self.velocity_pub = self.create_publisher(TwistStamped, '/ap/v1/cmd_vel', 10)
        self.get_logger().info(
            'Variant 19 target offset: dx=+8.2 m, dy=-0.4 m; cruise altitude 2.0 m')

    def _pose_cb(self, msg):
        p = msg.pose.position
        self.pose = (float(p.x), float(p.y), float(p.z))
        if self.phase != 'airborne' and p.z < 0.5:
            self.position_samples.append(self.pose)
        self.pose_received_at = time.monotonic()
        self.pose_stale_since = None

    def _publish_velocity(self, vx=0.0, vy=0.0, vz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # AP_DDS expects ENU when frame_id is map.
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(vx), float(vy), float(vz)
        self.velocity_pub.publish(msg)

    def control_loop(self):
        # Average the final ground samples before the takeoff service is called.
        # The method-sheet target is relative to the actual start position.
        if self.home is None and self.phase == 'takeoff':
            if self.position_samples:
                count = len(self.position_samples)
                self.home = tuple(sum(p[i] for p in self.position_samples) / count for i in range(3))
            elif self.pose is not None:
                self.home = self.pose
        if self.phase == 'airborne':
            if not self.armed_or_starting():
                self._publish_velocity()
                self.fail('Vehicle disarmed unexpectedly during route')
                return
            now = time.monotonic()
            if (self.pose is None or self.pose_received_at is None
                    or now - self.pose_received_at > 0.5):
                self._publish_velocity()
                if self.pose_stale_since is None:
                    self.pose_stale_since = now
                    self.get_logger().warning('Local position is stale; holding velocity at zero')
                elif now - self.pose_stale_since >= 2.0:
                    self.get_logger().error('Local position unavailable for 2 seconds; requesting LAND')
                    self.phase = 'land'
                    super().control_loop()
                return  # Do not move the vehicle using an outdated position estimate.
            if self.home is None:
                # Wait for a fresh position if startup had no local-pose sample.
                return
            if self.target is None:
                self.target = (self.home[0] + self.DELTA_X,
                               self.home[1] + self.DELTA_Y,
                               self.home[2] + self.TAKEOFF_ALTITUDE)
                self.get_logger().info(
                    f'Initial position x={self.home[0]:.2f}, y={self.home[1]:.2f}, z={self.home[2]:.2f} m')
                self.get_logger().info(
                    f'Variant 19 target x={self.target[0]:.2f}, y={self.target[1]:.2f}, z={self.target[2]:.2f} m')
                self.get_logger().info(
                    'Straight route loaded in the standard ArduPilot maze world')
            climb = self.pose[2] - self.home[2]
            if not self.takeoff_complete:
                if self.takeoff_started is None:
                    self.takeoff_started = time.monotonic()
                if climb >= self.TAKEOFF_ALTITUDE - self.TAKEOFF_ALTITUDE_TOLERANCE:
                    self.takeoff_complete = True
                    self.get_logger().info(
                        f'Takeoff altitude reached: {climb:.2f} m '
                        f'(target {self.TAKEOFF_ALTITUDE:.2f} m); starting waypoint flight')
                elif time.monotonic() - self.takeoff_started >= self.TAKEOFF_TIMEOUT_SECONDS:
                    self.get_logger().error(
                        f'Takeoff timed out at {climb:.2f} m; requesting LAND')
                    self.phase = 'land'
                    return
                else:
                    now = time.monotonic()
                    if now - self.last_climb_log >= 2.0:
                        self.last_climb_log = now
                        self.get_logger().info(
                            f'Waiting for takeoff: {climb:.2f} / '
                            f'{self.TAKEOFF_ALTITUDE:.2f} m; waypoint commands held')
                    # Takeoff service owns vertical motion until its target altitude.
                    # Sending /cmd_vel here can cancel or interfere with the climb.
                    return
            waypoint = self.target
            ex, ey = waypoint[0] - self.pose[0], waypoint[1] - self.pose[1]
            ez = waypoint[2] - self.pose[2]
            distance = math.hypot(ex, ey)
            if distance <= self.POSITION_TOLERANCE and abs(ez) <= self.ALTITUDE_TOLERANCE:
                self._publish_velocity()
                if self.reached_at is None:
                    self.reached_at = time.monotonic()
                    self.get_logger().info(
                        f'Target reached x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, z={self.pose[2]:.2f} m; '
                        f'horizontal error={distance:.2f} m')
                elif time.monotonic() - self.reached_at >= 2.0:
                    self.get_logger().info('Position hold complete; beginning landing')
                    self.phase = 'land'
                return
            self.reached_at = None
            # Proportional ENU velocity, capped by total horizontal speed.
            speed = min(self.MAX_HORIZONTAL_SPEED, 0.7 * distance)
            if distance > self.POSITION_TOLERANCE and speed < 0.12:
                speed = 0.12
            vx = speed * ex / max(distance, 1e-6)
            vy = speed * ey / max(distance, 1e-6)
            vz = max(-self.MAX_VERTICAL_SPEED, min(self.MAX_VERTICAL_SPEED, 0.6 * ez))
            self._publish_velocity(vx, vy, vz)
            return
        if self.phase in ('land', 'wait_landing'):
            self._publish_velocity()
        super().control_loop()

    def _land_done(self, label, future):
        super()._land_done(label, future)
        if self.phase == 'wait_landing' and self.pose is not None:
            error = math.hypot(self.pose[0] - self.target[0], self.pose[1] - self.target[1])
            self.get_logger().info(
                f'Final position before landing x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, '
                f'z={self.pose[2]:.2f} m; target error={error:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    spin_flight_node(CustomFlightNode())


if __name__ == '__main__': main()
