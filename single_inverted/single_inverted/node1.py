import rclpy
from rclpy.node import Node

from custom_msgs.msg import TorqueInput, States

class Node1(Node):
    def __init__(self):
        super().__init__('node1')

        # Publishers and Subscribers
        self.state_subscriber = self.create_subscription(States, '/state_feedback', self.state_callback, 5)
        
        self.torque_publisher = self.create_publisher(TorqueInput, '/torque_input', 5)
        self.timer=self.create_timer(0.5,self.publishtorque)
        
        
    def publishtorque(self):    
        msg=TorqueInput()
        msg.torque_value=1.0
        self.torque_publisher.publish(msg)
        self.get_logger().info(f"published torque {msg}")

        
    
    def state_callback(self, msg):
        # Extract the state feedback
        theta = msg.theta
        theta_dot = msg.theta_dot

        # Simple PD Controller to stabilize the pendulum
        # torque = -self.kp * theta - self.kd * theta_dot

        # # Limit the torque to a reasonable range
        # torque = max(min(torque, 5.0), -5.0)

        # # Publish the computed torque
        # torque_msg = TorqueInput()
        # torque_msg.torque_value = torque
        # self.torque_publisher.publish(torque_msg)

        self.get_logger().info(f"Received states: Theta: {theta:.2f}, Theta_dot: {theta_dot:.2f}")

def main(args=None):
    rclpy.init(args=args)
    controller = Node1()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
