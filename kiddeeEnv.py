import rclpy
import gym
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSHistoryPolicy
import time
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist


class kidDeeEnv(gym.Env):
    def __init__(self) -> None:
        super().__init__()
        rclpy.init()
        self.node=rclpy.create_node('kidDeeEnv')
        qos_sub = QoSProfile(depth=1,reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST)
        qos_pub = QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE)
        self._scan=self.node.create_subscription(LaserScan,'scan',self.scan_callback,qos_sub)
        self._odom=self.node.create_subscription(Odometry,'odom',self.odom_callback,qos_sub)
        self._rest_world=self.node.create_client(Empty,'reset_simulation')
        while not self._rest_world.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.empty_req=Empty.Request()
        self.command_velocity=self.node.create_publisher(Twist,'cmd_vel',qos_pub)
        self._scan_msg=LaserScan()
        self._odom_msg=Odometry()
        
    def scan_callback(self,msg):
        self._scan_msg=msg
        show=np.asarray(msg.ranges)
        #print(type(show))
        #print(len(show))
        #self.node.get_logger().info(f'test{show}\n')
        #print(f'{msg.ranges}')
    def odom_callback(self,msg):
        self._odom_msg=msg
        #show=msg.pose.pose.position
        #self.node.get_logger().info(f'{show.x},{show.y}\n')
    def reset(self):
        self.future=self._rest_world.call_async(self.empty_req)
        rclpy.spin_until_future_complete(self.node, self.future)
        time.sleep(1)
        obs=self.take_observation()
        return obs 
    def take_observation(self):
        rclpy.spin_once(self.node)
        odom=self._odom_msg
        rclpy.spin_once(self.node)
        scan=self._scan_msg
        #print(odom.header.frame_id)
        #print(type(odom.header.frame_id))
        while not scan.header.frame_id=='base_scan':
            #print(scan.header.frame_id)
            #print(type(scan.header.frame_id))
            rclpy.spin_once(self.node)
            scan=self._scan_msg
        while not odom.header.frame_id=='odom':
            rclpy.spin_once(self.node)
            odom=self._odom_msg
        #rclpy.spin_once(self.node,timeout_sec=10)
        return scan,odom

#env = kidDeeEnv()
#while True:
 #   rclpy.spin_once(env.node)
  #  time.sleep(1)

        