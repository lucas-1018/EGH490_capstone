#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool


def quat_to_yaw(q):
    # standard ZYX yaw extraction from a geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    def __init__(self, kp, ki, kd, out_limit, i_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit if i_limit is not None else out_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.out_limit, min(self.out_limit, out))


class PIDWaypointController(Node):
    def __init__(self):
        super().__init__('pid_waypoint_controller')

        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('waypoints_topic', '/bluerov2/waypoints')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel_unstamped')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('set_mode_service', '/mavros/set_mode')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')
        self.declare_parameter('guided_mode_name', 'GUIDED')
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('service_retry_period', 2.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('control_yaw', True)
        self.declare_parameter('yaw_tolerance', 0.1)
        # use_waypoint_yaw: hold the heading stored in each Path pose's
        # orientation (reactive planner sets +/-90 deg for +/-Y sweeps so the
        # sonar fan stays cross-track) instead of pointing at the waypoint.
        self.declare_parameter('use_waypoint_yaw', False)

        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.15)
        self.declare_parameter('max_lin_vel', 0.5)

        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.05)
        self.declare_parameter('kd_z', 0.2)
        self.declare_parameter('max_z_vel', 0.3)

        # sonar-based altitude hold -- if enabled, the z-axis PID targets a
        # constant distance above the seafloor (from sonar_altitude.py)
        # instead of the fixed world-frame z baked into each waypoint.
        self.declare_parameter('use_sonar_altitude', False)
        self.declare_parameter('altitude_topic', '/bluerov2/altitude')
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('altitude_hold_tolerance', 0.5)
        # pre_descend: hold x/y and descend to target_altitude BEFORE
        # starting to track waypoints, rather than doing both at once.
        self.declare_parameter('pre_descend', True)
        # when there's no valid altitude reading (sonar sees nothing --
        # e.g. reports infinity), descend at this fixed rate to actively
        # search for the bottom instead of freezing in place indefinitely.
        self.declare_parameter('search_descend_rate', 0.15)

        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.1)
        self.declare_parameter('max_yaw_rate', 0.5)

        self.pid_x = PID(self.gp('kp_xy'), self.gp('ki_xy'), self.gp('kd_xy'), self.gp('max_lin_vel'))
        self.pid_y = PID(self.gp('kp_xy'), self.gp('ki_xy'), self.gp('kd_xy'), self.gp('max_lin_vel'))
        self.pid_z = PID(self.gp('kp_z'), self.gp('ki_z'), self.gp('kd_z'), self.gp('max_z_vel'))
        self.pid_yaw = PID(self.gp('kp_yaw'), self.gp('ki_yaw'), self.gp('kd_yaw'), self.gp('max_yaw_rate'))

        self.control_yaw = self.get_parameter('control_yaw').value
        self.use_waypoint_yaw = self.get_parameter('use_waypoint_yaw').value
        self.waypoint_yaws = []
        self.reached_total = 0
        self.waypoint_tolerance = self.gp('waypoint_tolerance')
        self.yaw_tolerance = self.gp('yaw_tolerance')

        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_latched.reliability = QoSReliabilityPolicy.RELIABLE

        # mavros publishes odometry/state as BEST_EFFORT -- a default (RELIABLE)
        # subscription is incompatible and silently receives nothing.
        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_best_effort.durability = QoSDurabilityPolicy.VOLATILE

        self.waypoints = []
        self.wp_index = 0
        self.have_odom = False
        self.pos = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.use_sonar_altitude = self.get_parameter('use_sonar_altitude').value
        self.target_altitude = self.gp('target_altitude')
        self.altitude_hold_tolerance = self.gp('altitude_hold_tolerance')
        self.search_descend_rate = self.gp('search_descend_rate')
        self.current_altitude = None
        self.have_altitude = False
        # if pre-descend is off, or we're not using sonar altitude at all,
        # there's nothing to wait for -- go straight to waypoint tracking.
        self.mission_started = not (self.use_sonar_altitude and
                                     self.get_parameter('pre_descend').value)
        self._last_no_altitude_warn = 0.0
        self._mission_complete_logged = False

        self.guided_mode_name = self.get_parameter('guided_mode_name').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.retry_period = self.gp('service_retry_period')
        self.mavros_state = State()
        self.have_state = False
        self._last_service_attempt = 0.0

        self.create_subscription(Path, self.get_parameter('waypoints_topic').value,
                                  self.on_path, qos_latched)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                  self.on_odom, qos_best_effort)
        self.create_subscription(State, self.get_parameter('state_topic').value,
                                  self.on_state, qos_best_effort)
        if self.use_sonar_altitude:
            self.create_subscription(Float32, self.get_parameter('altitude_topic').value,
                                      self.on_altitude, 10)

        self.set_mode_client = self.create_client(
            SetMode, self.get_parameter('set_mode_service').value)
        self.arming_client = self.create_client(
            CommandBool, self.get_parameter('arming_service').value)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.waypoint_reached_pub = self.create_publisher(Int32, '/bluerov2/waypoint_reached', 10)

        rate = self.gp('control_rate_hz')
        self.timer = self.create_timer(1.0 / rate, self.control_step)

        self.get_logger().info('PID waypoint controller started, waiting for waypoints + odom + mavros state...')

    def gp(self, name):
        return float(self.get_parameter(name).value)

    def on_state(self, msg: State):
        self.mavros_state = msg
        self.have_state = True

    def ensure_guided_and_armed(self):
        """Fire-and-forget async calls to switch mode / arm, rate-limited so we
        don't spam the FCU every control tick while waiting for a response."""
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.have_state:
            if now - self._last_service_attempt >= self.retry_period:
                self._last_service_attempt = now
                self.get_logger().warn(
                    f"No /mavros/state received yet on "
                    f"'{self.get_parameter('state_topic').value}' -- waiting.")
            return False

        if not self.mavros_state.connected:
            if now - self._last_service_attempt >= self.retry_period:
                self._last_service_attempt = now
                self.get_logger().warn('mavros reports FCU not connected yet -- waiting.')
            return False

        ready = (self.mavros_state.mode == self.guided_mode_name and
                 (self.mavros_state.armed or not self.auto_arm))

        if ready:
            return True

        if now - self._last_service_attempt < self.retry_period:
            return False
        self._last_service_attempt = now

        self.get_logger().info(
            f"Not ready: mode='{self.mavros_state.mode}' (want '{self.guided_mode_name}'), "
            f"armed={self.mavros_state.armed}")

        if self.mavros_state.mode != self.guided_mode_name:
            if self.set_mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = self.guided_mode_name
                self.get_logger().info(f'Requesting mode {self.guided_mode_name}...')
                self.set_mode_client.call_async(req)
            else:
                self.get_logger().warn('/mavros/set_mode service not available yet')

        if self.auto_arm and not self.mavros_state.armed:
            if self.arming_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self.get_logger().info('Requesting arm...')
                self.arming_client.call_async(req)
            else:
                self.get_logger().warn('/mavros/cmd/arming service not available yet')

        return False

    def on_path(self, msg: Path):
        self.waypoints = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                           for p in msg.poses]
        self.waypoint_yaws = [quat_to_yaw(p.pose.orientation) for p in msg.poses]
        self.wp_index = 0
        self._mission_complete_logged = False
        self.pid_x.reset(); self.pid_y.reset(); self.pid_yaw.reset()
        if len(self.waypoints) == 1:
            w = self.waypoints[0]
            self.get_logger().info(f'New waypoint ({w[0]:.2f}, {w[1]:.2f})')
        else:
            self.get_logger().info(f'Received {len(self.waypoints)} waypoints')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def on_altitude(self, msg: Float32):
        self.current_altitude = msg.data
        self.have_altitude = True

    def compute_altitude_vz(self, dt):
        if self.have_altitude:
            ez = self.target_altitude - self.current_altitude
            return self.pid_z.update(ez, dt)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_no_altitude_warn > 3.0:
            self._last_no_altitude_warn = now
            self.get_logger().warn(
                'No valid altitude reading -- descending at search rate '
                f'({self.search_descend_rate} m/s) to find the bottom.')
        return -self.search_descend_rate

    def control_step(self):
        if not self.ensure_guided_and_armed():
            # Not ready yet (disconnected / wrong mode / disarmed) -- hold at
            # zero rather than let stale PID output fire the instant it's armed.
            self.publish_zero()
            return

        if not self.have_odom:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_service_attempt >= self.retry_period:
                self._last_service_attempt = now
                self.get_logger().warn(
                    f"No odometry received yet on "
                    f"'{self.get_parameter('odom_topic').value}' -- waiting.")
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # ---- pre-descend phase: hold x/y, drive to target_altitude before
        # touching any waypoints ----
        if not self.mission_started:
            vz = self.compute_altitude_vz(dt)
            cmd = Twist()
            cmd.linear.z = vz
            self.cmd_pub.publish(cmd)

            if self.have_altitude and \
                    abs(self.target_altitude - self.current_altitude) < self.altitude_hold_tolerance:
                self.get_logger().info(
                    f'Reached target altitude ({self.current_altitude:.2f}m) -- starting waypoint mission')
                self.mission_started = True
                self.pid_z.reset()
            return

        if self.wp_index >= len(self.waypoints):
            if self.waypoints:
                # a single-waypoint (reactive) path finishing is not "mission
                # complete" -- the planner will send the next one shortly
                if not self._mission_complete_logged and len(self.waypoints) > 1:
                    self._mission_complete_logged = True
                    px, py, pz = self.pos
                    self.get_logger().info(
                        f'Mission complete -- all {len(self.waypoints)} waypoints visited. '
                        f'Holding station near ({px:.2f}, {py:.2f}, {pz:.2f}).')
                # hold x/y but keep the sonar altitude hold running
                hold = Twist()
                if self.use_sonar_altitude:
                    hold.linear.z = self.compute_altitude_vz(dt)
                self.cmd_pub.publish(hold)
            return

        tx, ty, tz = self.waypoints[self.wp_index]
        px, py, pz = self.pos

        ex, ey = tx - px, ty - py

        if self.use_sonar_altitude:
            ez = self.target_altitude - self.current_altitude if self.have_altitude else 0.0
            dist = math.sqrt(ex * ex + ey * ey)  # waypoint-reached check ignores z here
        else:
            ez = tz - pz
            dist = math.sqrt(ex * ex + ey * ey + ez * ez)

        desired_yaw = self.yaw
        if self.control_yaw and self.use_waypoint_yaw:
            desired_yaw = self.waypoint_yaws[self.wp_index]
        elif self.control_yaw and dist > self.waypoint_tolerance:
            desired_yaw = math.atan2(ey, ex)
        eyaw = wrap_pi(desired_yaw - self.yaw)

        if dist < self.waypoint_tolerance and abs(eyaw) < self.yaw_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.wp_index} ({tx:.2f},{ty:.2f},{tz:.2f})')
            self.waypoint_reached_pub.publish(Int32(data=self.reached_total))
            self.reached_total += 1
            self.wp_index += 1
            self.pid_x.reset(); self.pid_y.reset(); self.pid_z.reset(); self.pid_yaw.reset()
            self.publish_zero()
            return

        # world-frame velocity commands
        vx_world = self.pid_x.update(ex, dt)
        vy_world = self.pid_y.update(ey, dt)
        vz = self.compute_altitude_vz(dt) if self.use_sonar_altitude else self.pid_z.update(ez, dt)
        wz = self.pid_yaw.update(eyaw, dt) if self.control_yaw else 0.0

        # mavros's setpoint_velocity plugin interprets this Twist as WORLD-frame
        # (LOCAL_NED after its own ENU->NED conversion), not body-frame -- so we
        # publish the world-frame PID output directly, with no extra rotation.
        cmd = Twist()
        cmd.linear.x = vx_world
        cmd.linear.y = vy_world
        cmd.linear.z = vz
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PIDWaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
