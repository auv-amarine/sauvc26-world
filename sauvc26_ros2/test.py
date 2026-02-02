#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.msg import OverrideRCIn


class SubAltHold(Node):
    def __init__(self):
        super().__init__('test')

        self.armed = False
        self.mode = ""

        # Publisher setpoint raw local (paling flexible)
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # Publisher RC override (backup - paling direct)
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        # Subscriber state
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        # Client set_mode
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        
        # Client arming
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        while not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Menunggu service /mavros/set_mode...')
        
        while not self.arming_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Menunggu service /mavros/cmd/arming...')

        # Setup RC Override - METODE PALING SIMPLE
        self.rc_cmd = OverrideRCIn()
        self.rc_cmd.channels = [65535] * 18
        # Channels untuk ArduSub (cek di QGround parameter RCMAP_*)
        # Biasanya: ch5=forward, ch6=lateral, ch3=throttle, ch4=yaw
        self.rc_cmd.channels[4] = 1650  # Channel 5: Forward (1500=stop, 1650=maju)
        self.rc_cmd.channels[5] = 1500  # Channel 6: Lateral
        self.rc_cmd.channels[2] = 1500  # Channel 3: Throttle (vertical)
        self.rc_cmd.channels[3] = 1500  # Channel 4: Yaw
        
        # Setup PositionTarget - VELOCITY MODE
        self.setpoint = PositionTarget()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_BODY_NED
        # Ignore position & acceleration, hanya pakai velocity
        self.setpoint.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        self.setpoint.velocity.x = 0.5  # Forward 0.5 m/s
        self.setpoint.velocity.y = 0.0
        self.setpoint.velocity.z = 0.0
        self.setpoint.yaw = 0.0

        # Timer kirim setpoint (20 Hz)
        self.timer = self.create_timer(0.05, self.send_cmd)
        
        # Timer untuk setup (arm dan set mode) setelah state ready
        self.setup_timer = self.create_timer(0.5, self.setup_vehicle)
        self.setup_done = False
        self.state_received = False
        self.arm_requested = False

    def state_cb(self, msg: State):
        self.armed = msg.armed
        self.mode = msg.mode
        self.state_received = True

    def setup_vehicle(self):
        if self.setup_done:
            return
        
        # Tunggu sampai state data masuk
        if not self.state_received:
            self.get_logger().info('Menunggu state data dari MAVROS...', throttle_duration_sec=2.0)
            return
        
        self.get_logger().info(f'State received - Armed: {self.armed}, Mode: {self.mode}')
        self.setup_done = True
        self.setup_timer.cancel()
        
        # Set mode GUIDED jika belum (async)
        if self.mode != 'GUIDED':
            req_mode = SetMode.Request()
            req_mode.custom_mode = 'GUIDED'
            future_mode = self.set_mode_cli.call_async(req_mode)
            future_mode.add_done_callback(self.mode_callback)
            self.get_logger().info('Requesting GUIDED mode...')
        
        # Request arm jika belum armed (async, tidak blocking)
        if not self.armed and not self.arm_requested:
            self.arm_requested = True
            req_arm = CommandBool.Request()
            req_arm.value = True
            future_arm = self.arming_cli.call_async(req_arm)
            future_arm.add_done_callback(self.arm_callback)
            self.get_logger().info('Requesting ARM...')
        
        # Monitor terus
        self.monitor_timer = self.create_timer(1.0, self.monitor_state)
    
    def mode_callback(self, future):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info('✓ Mode GUIDED set!')
            else:
                self.get_logger().error('✗ Gagal set GUIDED mode')
        except Exception as e:
            self.get_logger().error(f'✗ Set mode error: {e}')
    
    def arm_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('✓ Vehicle ARMED!')
            else:
                self.get_logger().error(f'✗ Gagal ARM - Code: {result.result}')
                self.get_logger().warn('Coba manual: ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"')
        except Exception as e:
            self.get_logger().error(f'✗ Arm service error: {e}')
    
    def monitor_state(self):
        self.get_logger().info(
            f'Armed: {self.armed} | Mode: {self.mode} | '
            f'Sending RC: ch5={self.rc_cmd.channels[4]}',
            throttle_duration_sec=3.0
        )

    def send_cmd(self):
        # Update timestamp
        self.setpoint.header.stamp = self.get_clock().now().to_msg()
        
        # Kirim RC override (paling reliable untuk BlueROV2)
        self.rc_pub.publish(self.rc_cmd)
        
        # Kirim setpoint raw (backup)
        self.setpoint_raw_pub.publish(self.setpoint)


def main():
    rclpy.init()
    node = SubAltHold()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
