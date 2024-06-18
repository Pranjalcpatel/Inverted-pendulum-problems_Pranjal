import rclpy
from rclpy.node import Node
import numpy as np

from custom_msgs.msg import TorqueInput, States

class PendulumController(Node):
    def __init__(self):
        super().__init__('from_bottom')

        # Publishers and Subscribers
        self.state_subscriber = self.create_subscription(States, '/state_feedback', self.state_callback, 5)
        self.torque_publisher = self.create_publisher(TorqueInput, '/torque_input', 5)

        # Controller Parameters
        self.kp = 15.0  # Proportional gain
        self.kd = 5.0   # Derivative gain

        self.get_logger().info('Pendulum Controller node initialized')

    def state_callback(self, msg):
        # Extract the state feedback
        theta = msg.theta
        theta_dot = msg.theta_dot

        if(theta>np.pi/2 or theta<-np.pi/2):
            # Adjust theta to be relative to the upright position (pi)
            theta = theta - np.pi

            # Ensure theta is within -pi to pi range
            theta = (theta + np.pi) % (2 * np.pi) - np.pi

            # Simple PD Controller to stabilize the pendulum
            torque = -self.kp * theta - self.kd * theta_dot

            # Limit the torque to a reasonable range
            torque = max(min(torque, 5.0), -5.0)

            # Publish the computed torque
            torque_msg = TorqueInput()
            torque_msg.torque_value = torque
            self.torque_publisher.publish(torque_msg)

            self.get_logger().info(f"Published Torque: {torque:.2f} for Theta: {theta:.2f}, Theta_dot: {theta_dot:.2f}")
        elif(theta_dot>0):
            torque_msg = TorqueInput()
            torque_msg.torque_value = 1.0
            self.torque_publisher.publish(torque_msg)
            self.get_logger().info(f"theta is {theta} and so published torque is {torque_msg}")
        else:
            torque_msg = TorqueInput()
            torque_msg.torque_value = -1.0
            self.torque_publisher.publish(torque_msg)
            self.get_logger().info(f"theta is {theta} and so published torque is {torque_msg}")

def main(args=None):
    rclpy.init(args=args)
    controller = PendulumController()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
