import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import math
from collections import deque
from time import sleep
from datetime import datetime
import numpy as np
from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan

K1 = 1
Kp = -0.5
Ki = 0.0000000000
Kd = -0.00000
a1 = 0.3
global sum_error
sum_error = 0.0
global prev_error
prev_error = 0.0
global i 
i = 0
QOS_PROFILE_DEFAULT = 10
PI = math.pi
k_turn = 0

LEFT_TURN = +1.0
RIGHT_TURN = -1.0
TURN_MIN = 0.0
TURN_MAX = 3.0
SPEED_MIN = 0.0
SPEED_MAX = 1.5
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25

class LineFollower(Node):
    """ Initializes line follower node with the required publishers and subscriptions.

        Returns:
            None
    """
    def __init__(self):
        super().__init__('line_follower')
        self.linear_velocity = 0.75  # set your desired value
        self.angular_velocity = -0.7    # set your desired value
        self.single_line_steer_scale = 1.0    # set your desired value
        self.q1 = deque()
        self.res = 0
        self.get_logger().info('Started')
        
        # Subscription for edge vectors.
        self.subscription_vectors = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self.edge_vectors_callback,
            QOS_PROFILE_DEFAULT
        )

        # Publisher for joy (for moving the rover in manual mode).
        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT
        )

        # Subscription for traffic status.
        self.subscription_traffic = self.create_subscription(
            TrafficStatus,
            '/traffic_status',
            self.traffic_status_callback,
            QOS_PROFILE_DEFAULT
        )

        # Subscription for LIDAR data.
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT
        )

        self.traffic_status = TrafficStatus()
        self.obstacle_detected = False
        self.ramp_detected = False

    """ Operates the rover in manual mode by publishing on /cerebri/in/joy.

        Args:
            speed: the speed of the car in float. Range = [-1.0, +1.0];
                Direction: forward for positive, reverse for negative.
            turn: steer value of the car in float. Range = [-1.0, +1.0];
                Direction: left turn for positive, right turn for negative.

        Returns:
            None
    """
    def rover_move_manual_mode(self, speed, turn):
        # Ensure speed and turn are floats within the valid range
        speed = float(speed)
        turn = float(turn)
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]

        # Delay for 0.5 seconds before publishing
        #sleep(0.05)

        self.publisher_joy.publish(msg)

    """ Analyzes edge vectors received from /edge_vectors to achieve line follower application.
        It checks for existence of ramps & obstacles on the track through instance members.
            These instance members are updated by the lidar_callback using LIDAR data.
        The speed and turn are calculated to move the rover using rover_move_manual_mode.

        Args:
            message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

        Returns:
            None
    """
    def lidar_callback(self, message):
        global sum_1, obs_dis, r, k_turn, a_diff, addUp, res
        a_diff = 0
        self.res = 0

        # TODO: participants need to implement logic for detection of ramps and obstacles.
        for i in range(len(message.ranges)):
            r = message.ranges[i]
            obs_dis = r
            if(obs_dis > 5 or math.isinf(obs_dis)):
                obs_dis = 5
            if (119 < i < 241):
                self.res -= obs_dis * (180 - i)

        shield_vertical = 4
        shield_horizontal = 1
        theta = math.atan(shield_vertical / shield_horizontal)

        # Get the middle half of the ranges array returned by the LIDAR.
        length = float(len(message.ranges))
        ranges = message.ranges[int(length / 4): int(3 * length / 4)]

        # Separate the ranges into the part in the front and the part on the sides.
        length = float(len(ranges))
        front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
        side_ranges_right = ranges[0: int(length * theta / PI)]
        side_ranges_left = ranges[int(length * (PI - theta) / PI):]

        # process front ranges.
        angle = theta - PI / 2
        for i in range(len(front_ranges)):
            if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
                self.obstacle_detected = True
                return
            angle += message.angle_increment

        # process side ranges.
        side_ranges_left.reverse()
        for side_ranges in [side_ranges_left, side_ranges_right]:
            angle = 0.0
            for i in range(len(side_ranges)):
                if (side_ranges[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                    self.obstacle_detected = True
                    return
                angle += message.angle_increment

        self.obstacle_detected = False

    def edge_vectors_callback(self, message):
        global Kp
        global Ki
        global K1
        global Kd
        global sum_error
        global prev_error
        global i
        speed = SPEED_MAX
        turn = TURN_MIN
        vectors = message
        window_center = vectors.image_width / 2
        steer, sum_1 = 0, 0
        m0x1 = 0
        global a1
        m1x1 = 0

        # NOTE: participants may improve algorithm for line follower.
        if (vectors.vector_count == 0):  # no vector detected straight velocity only
            speed = 0.8
            turn = 0
            pass

        if (vectors.vector_count == 1):    # 1 vector detected turn one side
            if(vectors.vector_1[1].x > vectors.vector_1[0].x):
                x = (vectors.vector_1[1].x - vectors.vector_1[0].x) / 320
                y = (vectors.vector_1[1].y - vectors.vector_1[0].y) / 97
            else:
                x = (vectors.vector_1[0].x - vectors.vector_1[1].x) / 320
                y = (vectors.vector_1[0].y - vectors.vector_1[1].y) / 97
            if(abs(x) != 0 and abs(y) != 0):
                turn = math.atan2(y, x) * 2.9 * a1
                if(abs(y) < 0.2):
                    turn = math.atan2(y, x) * a1 / abs(y * abs(x) / 4)
                    #print(turn)
                speed = 0.8 * (1 - a1 * abs(turn))
                if speed <= 0.2:
                    speed = 0.2
            else:
                turn = 0
                speed = 0.8
            if abs(vectors.vector_1[1].y - vectors.vector_1[0].y) <= 45:
                steer = -turn * 1.25
                speed = 0.8 * speed

        if (vectors.vector_count == 2): # 2 vector detected -> magnitude equal
            m0x1 = (vectors.vector_1[1].x)
            m1x1 = (vectors.vector_2[1].x)
            m0y1 = (vectors.vector_1[1].y)
            m1y1 = (vectors.vector_2[1].y)
            m0x0 = (vectors.vector_1[0].x)
            m1x0 = (vectors.vector_2[0].x)
            m0y0 = (vectors.vector_1[0].y)
            m1y0= (vectors.vector_2[0].y)
            v_1X = (m0x1-m0x0)
            v_1Y = (m0y1-m0y0)
            v_2X = (m1x1-m1x0)
            v_2Y = (m1y1-m1y0)
            e = math.sqrt(math.pow(v_1X,2)+math.pow(v_1Y,2))-math.sqrt(math.pow(v_2X,2)+math.pow(v_2Y,2))
            # print(e)
            turn=e*-.0008
			#print(f"x1_1={m0x1},y1_1={m0y1},x2_1={m1x1},y2_1={m1y1},x1_0={m0x0},y1_0={m0y0},x2_0={m1x0},y2_0={m1y0}")
            slope1 = math.atan2(v_1Y,v_1X)
            slope2=math.atan2(v_2Y,v_2X)
			#print(f"slope1={slope1},slope2={slope2}")
            if(m0x1==0 and 205<m0y1<=239 and m0x0==319 and 155<m0y0<180 and 120<m1x0<189 and 140<m1y1<160 and 140<m1y0<160 and m1x1==319):
                x_alpha=0
                while(x_alpha<1500 ):
                    turn = -1
                    speed = 0.3
                    x_alpha+=1
                    self.rover_move_manual_mode(speed, turn)
                    self.get_logger().warn('Started')
            if(m0x1==0.0 and 140<m0y1<160 and m1x1==319.0 and 190<m1y1<=239.0 and 40<m0x0<125 and 140<m0y0<149.0 and 0<=m1x0<100.0 and 160<m1y0<189.0):
                x_alpha_2=0
                while(x_alpha_2<2500):
                 turn = 1
                 speed = 0.3
                 x_alpha_2+=1
                 self.rover_move_manual_mode(speed,turn)
                 self.get_logger().warn('conditoion2')
            if(2.9<slope1<3.1 and 2.0<slope2<2.8):
                turn*=-10
            if(1.7<slope1<1.9 and slope2<.2):
                turn*=-10
            self.res = self.res/1000
            #print(self.res)
            turn = abs(1-abs(self.res))*turn + self.res
            print(turn)
			
   
        if(abs(speed)>0.6):
            speed=0.6
            
        try:
            x = turn/abs(turn)
        except ZeroDivisionError:
            x=0
            
        if(abs(turn)>1):
             turn=1*x
		# print(vectors.vector_1[1].x,vectors.vector_1[0].x,vectors.vector_1[1].y,vectors.vector_1[0].y)


		# if (self.traffic_status.stop_sign is True):
		# 	speed = SPEED_MIN
		# 	print("stop sign detected")

		# if self.ramp_detected is True:
		# 	speed=0.0000
		# 	# TODO: participants need to decide action on detection of ramp/bridge.
		# 	print("ramp/bridge detected")

		# # if self.obstacle_detected is True:
		# # 	# TODO: participants need to decide action on detection of obstacle.
		# # 	print("obstacle detected")
		# turn=steer
		# print(speed,turn)
        self.rover_move_manual_mode(speed, turn)

    """ Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
    def traffic_status_callback(self, message):
        self.traffic_status = message

    """ Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""

def main(args=None):
	rclpy.init(args=args)

	line_follower = LineFollower()

	rclpy.spin(line_follower)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	line_follower.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()