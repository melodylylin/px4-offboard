import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
# from px4_msgs.msg import TrajectorySetpoint
# from px4_msgs.msg import VehicleStatus

def quat_product(qa, qb):
    r1 = qa[0]
    v1 = qa[1:]
    r2 = qb[0]
    v2 = qb[1:]
    res = np.zeros((4,))
    res[0] = r1 * r2 - np.dot(v1, v2)
    res[1:] = r1 * v2 + r2 * v1 + np.cross(v1, v2)
    return res

class MoCapStream(Node):
    def __init__(self):
        super().__init__("px4_mocap_stream")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_ALL,
            depth=1
        )
        self.timer = self.create_timer(1/30, self.timer_cb)

        self.mocap_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/drone1/pose',
            self.pose_cb,
            10
        )

        self.mocap_odom_pub_ = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile
        )

        self.drone_pose = None

        self.get_logger().info("Starting MoCap stream to PX4...")

    def pose_cb(self, msg: PoseStamped):
        self.drone_pose = msg

    def timer_cb(self):
        if self.drone_pose is not None:
            x = self.drone_pose.pose.position.x
            y = self.drone_pose.pose.position.y
            z = self.drone_pose.pose.position.z

            qw = self.drone_pose.pose.orientation.w
            qx = self.drone_pose.pose.orientation.x
            qy = self.drone_pose.pose.orientation.y
            qz = self.drone_pose.pose.orientation.z
	
    	    R_ENU2NED = np.array([[2*sqrt(2)/2**2 - 1, 2*(sqrt(2)/2*sqrt(2)/2), 0],
        		  	  [0, 2*sqrt(2)/2**2 - 1, 0],
        		  	  [0, 0, -1]])
	    q = R@np.array([qw,qx,qy,qz])

            msg = VehicleOdometry()
            msg.timestamp = int(Clock().now().nanoseconds / 1000)
            msg.position = np.array([y, x, -z], dtype=np.float32)
            
            # q_flu = np.array([qw, qx, qy, qz], dtype=np.float32)
            q_frd = np.array([q[0], q[1], q[2], q[3]], dtype=np.float32)
            msg.q = q_frd
            # self.get_logger().info(f"Pos: {msg.position}")
            msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD

            self.mocap_odom_pub_.publish(msg)

def main(args=None):
    rclpy.init()

    try:
        node = MoCapStream()
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__=="__main__":
    main()
