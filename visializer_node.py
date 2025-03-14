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
        self.current_pose = Odometry()
        self.node_res = Odometry()

        self.subscription = self.create_subscription(
            Odometry,
            'debug/odometry',
            self.node_callback,
            qos_profile)

        self.subscription = self.create_subscription(
            Odometry,
            'odometry',
            self.ac_callback,
            qos_profile
        )

        self.circuit_data = pd.read_csv('vallelunga_x_y_r_v_undersampled_512.csv')
        self.fig, self.ax = plt.subplots()
        self.ax.plot(self.circuit_data['x'], self.circuit_data['y'], label='Circuito')
        self.ax.scatter(x='x', y='y', data=self.circuit_data, s=4, label='samppling')
        self.point_plot, = self.ax.plot([], [], 'go', markersize=5, label='Posizione attuale')
        self.external_plot, = self.ax.plot([], [], 'ro', markersize=6, label='Punto esterno')
        self.yaw_line, = self.ax.plot([], [], 'r-', linewidth=2, label='Yaw')
        self.debby_line, = self.ax.plot([], [], 'b-', linewidth=2, label='debby')
        self.ax.legend()
        plt.ion()
        plt.show()

    def ac_callback(self, msg):
        self.current_pose = msg
        self.odom_callback()

    def node_callback(self,msg):
        self.node_res = msg
        self.odom_callback()
    
    def odom_callback(self):
        if(self.node_res._header.stamp == self.current_pose.header.stamp):
            nn_pos_x = self.node_res.pose.pose.position.x
            nn_pos_y = self.node_res.pose.pose.position.y

            pos_x = self.current_pose.pose.pose.position.x
            pos_y = self.current_pose.pose.pose.position.y

            q=R.from_quat([self.current_pose.pose.pose.orientation.x,self.current_pose.pose.pose.orientation.y,self.current_pose.pose.pose.orientation.z,self.current_pose.pose.pose.orientation.w])
            r_matrix=q.as_matrix()

            self.point_plot.set_data([nn_pos_x], [nn_pos_y])

            self.external_plot.set_data([pos_x], [pos_y])

            vector_length = 10.0
            end_x = nn_pos_x + vector_length * r_matrix[0][2]
            end_y = nn_pos_y + vector_length * r_matrix[2][2]

            self.yaw_line.set_data([pos_x, end_x], [pos_y, end_y])

            debby_length = 1.0
            debby_x = nn_pos_x + debby_length * self.node_res.twist.twist.linear.x
            debby_y = nn_pos_y + debby_length * self.node_res.twist.twist.linear.y

            self.debby_line.set_data([nn_pos_x, debby_x], [nn_pos_y, debby_y])


            theta = np.arctan2(r_matrix[2,2], r_matrix[0,2])
            #print(f'x: {pos_x}, y: {pos_y}, yaw: {theta}')


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
