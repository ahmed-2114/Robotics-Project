import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math

class JointBridge(Node):
    def __init__(self):
        super().__init__('joint_bridge_node')
        
        # Subscribe to joint states from simulation
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
            
        # Publish commands to ESP32
        self.publisher_ = self.create_publisher(Float32MultiArray, '/robot_arm_command', 10)
        
        # --- TUNABLE FREQUENCY ---
        # 10Hz = 0.1s. Ideally, match this with your ESP32 loop speed.
        # Too fast (>20Hz) can choke the serial buffer.
        # Too slow (<5Hz) will look jerky.
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Joint Names must match your URDF and Gazebo exactly
        self.target_joints = ['J1', 'J2', 'J3', 'J4', 'J5']
        
        # Buffer to store the latest positions
        self.latest_positions = [0.0] * 5
        self.data_received = False
        
        self.get_logger().info('Robust Bridge Node Started at 10Hz.')

    def listener_callback(self, msg):
        try:
            # We map the incoming joint states to our fixed array order [J1, J2, J3, J4, Gripper]
            for i, target_name in enumerate(self.target_joints):
                if target_name in msg.name:
                    index = msg.name.index(target_name)
                    pos = float(msg.position[index])
                    self.latest_positions[i] = pos
            
            self.data_received = True
            
        except ValueError as e:
            # Occasional missing joints in state messages are normal during startup
            pass

    def timer_callback(self):
        # Only publish if we have actually received data from simulation
        if self.data_received:
            command_msg = Float32MultiArray()
            command_msg.data = self.latest_positions
            self.publisher_.publish(command_msg)
            
            # Debugging: Print first joint value every 10 cycles (1 second)
            # self.get_logger().info(f'Sending: {self.latest_positions}') 
        else:
            self.get_logger().warn('Waiting for /joint_states data...', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = JointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()