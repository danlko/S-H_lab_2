"""Task 3: automatic takeoff, 30-second hover, and landing."""
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from .flight_common import FlightNode, POSITION_QOS, spin_flight_node


class FlightTestNode(FlightNode):
    HOVER_SECONDS = 30.0

    def __init__(self):
        super().__init__('flight_test')
        self.hover_started = None
        self.initial_altitude = None
        self.altitude = None
        self.last_climb_log = 0.0
        self.last_pose_wait_log = 0.0
        self.pose_sub = self.create_subscription(
            PoseStamped, '/ap/v1/pose/filtered', self._pose_cb, POSITION_QOS)

    def _pose_cb(self, msg):
        self.altitude = float(msg.pose.position.z)
        if self.initial_altitude is None:
            self.initial_altitude = self.altitude

    def control_loop(self):
        if self.phase == 'airborne':
            if not self.armed_or_starting():
                self.fail('Vehicle disarmed unexpectedly during flight')
            elif self.altitude is None or self.initial_altitude is None:
                now = time.monotonic()
                if now - self.last_pose_wait_log >= 3.0:
                    self.last_pose_wait_log = now
                    self.get_logger().info('Waiting for local altitude estimate')
            elif self.altitude - self.initial_altitude < self.TAKEOFF_ALTITUDE * 0.85:
                now = time.monotonic()
                if now - self.last_climb_log >= 2.0:
                    self.last_climb_log = now
                    self.get_logger().info(
                        f'Climbing: {self.altitude - self.initial_altitude:.2f} / '
                        f'{self.TAKEOFF_ALTITUDE:.2f} m')
            elif self.hover_started is None:
                self.hover_started = time.monotonic()
                self.get_logger().info(
                    f'Takeoff altitude reached: '
                    f'{self.altitude - self.initial_altitude:.2f} m '
                    f'(target {self.TAKEOFF_ALTITUDE:.2f} m)')
                self.get_logger().info(f'Hovering for {self.HOVER_SECONDS:.0f} seconds')
            elif time.monotonic() - self.hover_started >= self.HOVER_SECONDS:
                self.get_logger().info('Hover complete; beginning landing')
                self.phase = 'land'
            return
        super().control_loop()


def main(args=None):
    rclpy.init(args=args)
    spin_flight_node(FlightTestNode())


if __name__ == '__main__': main()
