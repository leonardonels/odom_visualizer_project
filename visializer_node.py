import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(
            Odometry,
            'debug/odometry',
            self.odom_callback,
            qos_profile)

        self.circuit_data = pd.read_csv('vallelunga1_circuit.csv')
        self.fig, self.ax = plt.subplots()
        self.ax.plot(self.circuit_data['x'], self.circuit_data['y'], label='Circuito')
        self.point_plot, = self.ax.plot([], [], 'go', markersize=8, label='Posizione attuale')
        self.external_plot, = self.ax.plot([], [], 'ro', markersize=6, label='Punto esterno')
        self.yaw_line, = self.ax.plot([], [], 'r-', linewidth=2, label='Yaw')
        self.debby_line, = self.ax.plot([], [], 'b-', linewidth=2, label='debby')
        self.ax.legend()
        plt.ion()
        plt.show()
    
    def odom_callback(self, msg):
        nn_pos_x = msg.twist.twist.linear.x
        nn_pos_y = msg.twist.twist.linear.y

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        q=R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        r_matrix=q.as_matrix()

        self.point_plot.set_data(nn_pos_x, nn_pos_y)
        
        self.external_plot.set_data(pos_x, pos_y)

        vector_length = 100.0
        end_x = pos_x + vector_length * r_matrix[0][2]
        end_y = pos_y + vector_length * r_matrix[2][2]

        self.yaw_line.set_data([pos_x, end_x], [pos_y, end_y])

        debby_length = 10.0
        debby_x = nn_pos_x + debby_length * msg.pose.pose.position.z
        debby_y = nn_pos_y + debby_length * msg.twist.twist.linear.z

        self.debby_line.set_data([nn_pos_x, debby_x], [nn_pos_y, debby_y])

        
        theta = np.arctan2(r_matrix[2,2], r_matrix[0,2])
        print(f'x: {pos_x}, y: {pos_y}, yaw: {theta}')

        
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    visualizer_node = VisualizerNode()
    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            visualizer_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
