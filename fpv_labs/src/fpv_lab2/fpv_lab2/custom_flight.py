"""Task 4: variant 19 local ENU waypoint flight followed by automatic landing."""
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from .flight_common import FlightNode, POSITION_QOS, spin_flight_node


class CustomFlightNode(FlightNode):
    # Variant 19: target = initial position - (8.2, -0.4) metres.
    DELTA_X, DELTA_Y = 8.2, -0.4
    POSITION_TOLERANCE, ALTITUDE_TOLERANCE = 0.25, 0.25
    MAX_HORIZONTAL_SPEED, MAX_VERTICAL_SPEED = 0.8, 0.4
    TAKEOFF_ALTITUDE_TOLERANCE = 0.3
    TAKEOFF_TIMEOUT_SECONDS = 25.0

    def __init__(self):
        super().__init__('custom_flight')
        self.pose = self.home = self.target = self.reached_at = None
        self.takeoff_started = None
        self.takeoff_complete = False
        self.last_climb_log = 0.0
        self.pose_sub = self.create_subscription(
            PoseStamped, '/ap/v1/pose/filtered', self._pose_cb, POSITION_QOS)
        self.velocity_pub = self.create_publisher(TwistStamped, '/ap/v1/cmd_vel', 10)
        self.get_logger().info('Variant 19 offset dx=-8.2 m, dy=+0.4 m in ENU/map')

    def _pose_cb(self, msg):
        p = msg.pose.position
        self.pose = (float(p.x), float(p.y), float(p.z))

    def _publish_velocity(self, vx=0.0, vy=0.0, vz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # AP_DDS expects ENU when frame_id is map.
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(vx), float(vy), float(vz)
        self.velocity_pub.publish(msg)

    def control_loop(self):
        # Capture the actual ground position while the pre-arm/mode requests run.
        # Capturing only after takeoff would incorrectly add another 2 m to the target.
        if self.home is None and self.pose is not None and self.phase != 'airborne':
            self.home = self.pose
        if self.phase == 'airborne':
            if self.status is None or not self.status.armed:
                self._publish_velocity()
                self.fail('Vehicle disarmed unexpectedly during route')
                return
            if self.pose is None:
                return  # Do not start the route until actual local position is available.
            if self.home is None:
                # Wait for a fresh position if startup had no local-pose sample.
                return
            if self.target is None:
                self.target = (self.home[0] - self.DELTA_X,
                               self.home[1] - self.DELTA_Y,
                               self.home[2] + self.TAKEOFF_ALTITUDE)
                self.get_logger().info(
                    f'Initial position x={self.home[0]:.2f}, y={self.home[1]:.2f}, z={self.home[2]:.2f} m')
                self.get_logger().info(
                    f'Variant 19 target x={self.target[0]:.2f}, y={self.target[1]:.2f}, z={self.target[2]:.2f} m')
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
            ex, ey = self.target[0] - self.pose[0], self.target[1] - self.pose[1]
            ez = self.target[2] - self.pose[2]
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
            # Proportional ENU speed with a cap, reduced as the vehicle approaches the target.
            scale = min(1.0, self.MAX_HORIZONTAL_SPEED / max(distance, 1e-6))
            vx = max(-self.MAX_HORIZONTAL_SPEED, min(self.MAX_HORIZONTAL_SPEED, 0.7 * ex)) * scale
            vy = max(-self.MAX_HORIZONTAL_SPEED, min(self.MAX_HORIZONTAL_SPEED, 0.7 * ey)) * scale
            if abs(ex) > self.POSITION_TOLERANCE and abs(vx) < 0.12: vx = math.copysign(0.12, ex)
            if abs(ey) > self.POSITION_TOLERANCE and abs(vy) < 0.12: vy = math.copysign(0.12, ey)
            vz = max(-self.MAX_VERTICAL_SPEED, min(self.MAX_VERTICAL_SPEED, 0.6 * ez))
            self._publish_velocity(vx, vy, vz)
            return
        if self.phase in ('land', 'wait_landing'):
            self._publish_velocity()
        super().control_loop()

    def _land_done(self, label, future):
        super()._land_done(label, future)
        if self.phase == 'wait_landing' and self.pose is not None:
            self.get_logger().info(
                f'Position before landing x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, z={self.pose[2]:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    spin_flight_node(CustomFlightNode())


if __name__ == '__main__': main()
