#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.transform_broadcaster import TransformBroadcaster

from teensy32_tof_msgs.msg import ToFData

# from utils.kalman import Kalman
from teensy32_tof_bringup.utils.kalman import Kalman
from filterpy.kalman import KalmanFilter

import numpy as np


class ToFNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name="tof_node")
        # Subscriptions
        self._sub_tof_raw = self.create_subscription(
            msg_type=ToFData,
            topic="/microROS/tof_data",
            callback=self._sub_cb_tof_raw,
            qos_profile=1,
        )

        # Publishers
        self._pub_tof_filtered = self.create_publisher(
            msg_type=ToFData,
            topic="tof_filtered",
            qos_profile=1,
        )

        # Timers
        self._timer_tof_filtered_pub = self.create_timer(
            timer_period_sec=1/30,
            callback=self._timer_cb_tof_filtered_pub
        )

        self.tof0: float = 0.0
        self.tof1: float = 0.0
        self.tof0_f: float = 0.0
        self.tof1_f: float = 0.0

        self.kalman0 = KalmanFilter(dim_x=2, dim_z=1)
        self.kalman0.x = np.array([[self.tof0, 0]]).transpose()
        self.kalman0.F = np.array([[1,1],[0,1]])
        self.kalman0.H = np.array([[1, 0]])
        self.kalman0.P = np.identity(self.kalman0.dim_x) * 5
        self.kalman0.R = 5

        self.kalman1 = KalmanFilter(dim_x=2, dim_z=1)
        self.kalman1.x = np.array([[self.tof1, 0]]).transpose()
        self.kalman1.F = np.array([[1,1],[0,1]])
        self.kalman1.H = np.array([[1, 0]])
        self.kalman1.P = np.identity(self.kalman1.dim_x) * 5
        self.kalman1.R = 5
        return
    
    def _sub_cb_tof_raw(self, msg: ToFData) -> None:
        """Callback method for ToF raw data subscriber"""
        self.tof0 = msg.tof0
        self.tof1 = msg.tof1
        self.kalman0.predict()
        self.kalman0.update([self.tof0])
        self.kalman1.predict()
        self.kalman1.update([self.tof1])
        self.get_logger().info(f"\nRaw {[self.tof0, self.tof1]}\nFiltered: {[self.kalman0.x[0][0], self.kalman1.x[0][0]]}")

        self.tof0_f = self.kalman0.x[0,0]
        self.tof1_f = self.kalman1.x[0,0]
        return
    
    def _timer_cb_tof_filtered_pub(self) -> None:
        """Callback method for the filtered ToF data"""
        msg = ToFData()
        msg.tof0 = self.tof0_f
        msg.tof1 = self.tof1_f
        self._pub_tof_filtered.publish(msg=msg)
        return 
    



def main():

    rclpy.init()

    tof_node = ToFNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(node=tof_node, executor=executor)

    rclpy.shutdown()

    return


if __name__ == "__main__":
    main()