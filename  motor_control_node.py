#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import Float32MultiArray

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Publisher: Send CAN frames to the bridge
        self.can_pub = self.create_publisher(
            Frame, 
            'CAN/can0/transmit', 
            10
        )
        
        # Subscriber: Receive high-level motor commands
        self.motor_sub = self.create_subscription(
            Float32MultiArray,
            'motor_commands',
            self.motor_command_callback,
            10
        )
        
        # Subscriber: Receive CAN frames from the bridge
        self.can_sub = self.create_subscription(
            Frame,
            'CAN/can0/receive',
            self.can_feedback_callback,
            10
        )
        
        # Publisher: Send parsed motor feedback
        self.feedback_pub = self.create_publisher(
            Float32MultiArray,
            'motor_feedback',
            10
        )
        
        self.get_logger().info('Motor Control Node initialized')
    
    def motor_command_callback(self, msg):
        """
        Convert high-level motor commands to CAN frames
        Expected msg.data format: [motor_id, velocity, position, ...]
        """
        try:
            motor_id = int(msg.data[0])
            velocity = msg.data[1] if len(msg.data) > 1 else 0.0
            position = msg.data[2] if len(msg.data) > 2 else 0.0
            
            # Create CAN Frame
            can_frame = Frame()
            can_frame.id = motor_id
            can_frame.is_extended = False
            can_frame.is_error = False
            can_frame.is_rtr = False
            can_frame.dlc = 8
            
            # Pack motor command data (adjust based on your motor protocol)
            # Example: velocity in bytes 0-3, position in bytes 4-7
            velocity_bytes = int(velocity * 100).to_bytes(4, 'little', signed=True)
            position_bytes = int(position * 100).to_bytes(4, 'little', signed=True)
            can_frame.data = list(velocity_bytes) + list(position_bytes)
            
            # Publish to CAN bridge
            self.can_pub.publish(can_frame)
            self.get_logger().info(
                f'Sent CAN frame: ID={motor_id}, vel={velocity}, pos={position}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing motor command: {e}')
    
    def can_feedback_callback(self, msg):
        """
        Convert CAN frames to motor feedback messages
        """
        try:
            # Parse CAN frame based on your motor controller's protocol
            motor_id = float(msg.id)
            
            # Example: Extract velocity and position from data bytes
            velocity = int.from_bytes(msg.data[0:4], 'little', signed=True) / 100.0
            position = int.from_bytes(msg.data[4:8], 'little', signed=True) / 100.0
            
            # Create feedback message
            feedback = Float32MultiArray()
            feedback.data = [motor_id, velocity, position]
            
            # Publish feedback
            self.feedback_pub.publish(feedback)
            self.get_logger().debug(f'Motor {int(motor_id)} feedback: vel={velocity}, pos={position}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing CAN feedback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
