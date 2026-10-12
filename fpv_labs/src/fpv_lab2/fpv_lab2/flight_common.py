"""Shared ArduPilot DDS service sequence for both flight examples."""
from functools import partial
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

POSITION_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


class FlightNode(Node):
    GUIDED_MODE, LAND_MODE, TAKEOFF_ALTITUDE = 4, 9, 2.0
    PREARM_TIMEOUT_SECONDS, PREARM_RETRY_SECONDS = 30.0, 1.0

    def __init__(self, name):
        super().__init__(name)
        self.status = None
        self.request_in_progress = self.failed = self.finished = False
        self.takeoff_accepted_at = None
        self.armed_seen = False
        self.prearm_started_at = None
        self.next_prearm_check_at = 0.0
        self.phase = 'wait'
        self.status_sub = self.create_subscription(Status, '/ap/v1/status', self._status_cb, 10)
        self.prearm_client = self.create_client(Trigger, '/ap/v1/prearm_check')
        self.mode_client = self.create_client(ModeSwitch, '/ap/v1/mode_switch')
        self.arm_client = self.create_client(ArmMotors, '/ap/v1/arm_motors')
        self.takeoff_client = self.create_client(Takeoff, '/ap/v1/experimental/takeoff')
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('Waiting for ArduPilot DDS status and services')

    def _status_cb(self, msg):
        self.status = msg

    def services_ready(self):
        return all(c.service_is_ready() for c in
                   (self.prearm_client, self.mode_client, self.arm_client, self.takeoff_client))

    def control_loop(self):
        if self.finished or self.failed:
            return
        if self.phase == 'wait':
            if self.status is not None and self.services_ready():
                self.get_logger().info('ArduPilot status and flight services are ready')
                self.phase = 'prearm'
        elif self.phase == 'prearm' and not self.request_in_progress:
            now = time.monotonic()
            if self.prearm_started_at is None:
                self.prearm_started_at = now
            if now - self.prearm_started_at >= self.PREARM_TIMEOUT_SECONDS:
                self.fail('Pre-arm did not become ready within 30 seconds')
            elif now >= self.next_prearm_check_at:
                self._call(self.prearm_client, Trigger.Request(), self._prearm_done, 'pre-arm check')
        elif self.phase == 'guided' and not self.request_in_progress:
            req = ModeSwitch.Request(); req.mode = self.GUIDED_MODE
            self._call(self.mode_client, req, self._guided_done, 'GUIDED mode')
        elif self.phase == 'arm' and not self.request_in_progress:
            req = ArmMotors.Request(); req.arm = True
            self._call(self.arm_client, req, self._arm_done, 'arming')
        elif self.phase == 'takeoff' and not self.request_in_progress:
            req = Takeoff.Request(); req.alt = self.TAKEOFF_ALTITUDE
            self._call(self.takeoff_client, req, self._takeoff_done, 'takeoff')
        elif self.phase == 'land' and not self.request_in_progress:
            req = ModeSwitch.Request(); req.mode = self.LAND_MODE
            self._call(self.mode_client, req, self._land_done, 'LAND mode')
        elif self.phase == 'wait_landing' and self.status is not None and not self.status.flying:
            self.finished = True
            self.get_logger().info('Landing completed; flight sequence finished')

    def armed_or_starting(self):
        if self.status is not None and self.status.armed:
            self.armed_seen = True
            return True
        # Status is published asynchronously and can briefly lag the takeoff reply.
        return (not self.armed_seen and self.takeoff_accepted_at is not None
                and time.monotonic() - self.takeoff_accepted_at < 5.0)

    def _call(self, client, request, callback, label):
        self.request_in_progress = True
        future = client.call_async(request)
        future.add_done_callback(partial(callback, label))
        self.get_logger().info(f'Requesting {label}')

    def _result(self, label, future):
        self.request_in_progress = False
        try:
            return future.result()
        except Exception as exc:
            self.fail(f'{label} service call failed: {exc}')
            return None

    def _prearm_done(self, label, future):
        r = self._result(label, future)
        if r is None: return
        if not r.success and r.message == 'Vehicle is Not Armable':
            self.next_prearm_check_at = time.monotonic() + self.PREARM_RETRY_SECONDS
            self.get_logger().info('Waiting for ArduPilot EKF/GPS readiness before arming')
        elif not r.success: self.fail(f'Pre-arm rejected: {r.message}')
        else:
            self.get_logger().info(f'Pre-arm passed: {r.message}')
            self.phase = 'guided'

    def _guided_done(self, label, future):
        r = self._result(label, future)
        if r is None: return
        if not r.status or r.curr_mode != self.GUIDED_MODE:
            self.fail(f'Could not set GUIDED mode (current mode {r.curr_mode})')
        else:
            self.get_logger().info('GUIDED mode enabled')
            self.phase = 'arm'

    def _arm_done(self, label, future):
        r = self._result(label, future)
        if r is None: return
        if not r.result: self.fail('ArduPilot rejected motor arming')
        else:
            self.get_logger().info('Motors armed')
            self.phase = 'takeoff'

    def _takeoff_done(self, label, future):
        r = self._result(label, future)
        if r is None: return
        if not r.status: self.fail('ArduPilot rejected takeoff')
        else:
            self.takeoff_accepted_at = time.monotonic()
            self.armed_seen = bool(self.status and self.status.armed)
            self.get_logger().info(f'Takeoff to {self.TAKEOFF_ALTITUDE:.1f} m accepted')
            self.phase = 'airborne'

    def _land_done(self, label, future):
        r = self._result(label, future)
        if r is None: return
        if not r.status or r.curr_mode != self.LAND_MODE:
            self.fail(f'Could not set LAND mode (current mode {r.curr_mode})')
        else:
            self.get_logger().info('LAND mode enabled; waiting for touchdown')
            self.phase = 'wait_landing'

    def fail(self, message):
        self.failed = True
        self.get_logger().error(message)


def spin_flight_node(node):
    import rclpy
    try:
        while rclpy.ok() and not node.finished and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted by user')
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
