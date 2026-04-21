# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge

# import cv2
# import numpy as np
# import time


# class BirdEyeViewController(Node):
#     def __init__(self):
#         super().__init__("turtlebot3_bird_eye_view_control")

#         self.bridge = CvBridge()
#         self.sub = self.create_subscription(Image, "/image_raw", self.image_cb, 10)
#         self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
 
#         self.h = self.homographie()

#         self.get_logger().info("Bird eye view Controller started.")

#     def image_cb(self, msg):
#         self.get_logger().info("Callback")

#         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#         cv2.imshow("Test", frame)
#         cv2.waitKey()

#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.cmd_pub.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = BirdEyeViewController()

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math
import numpy as np
import cv2
import time
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class SimpleSuiveur(Node):
 
    def __init__(self):
        super().__init__('simple_camera')
 
        self.cv_bridge = CvBridge()
 
        self.scan_subscriber = self.create_subscription(Image, 'image_raw', self.cam_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # self.orb = cv2.ORB_create()
        # # create BFMatcher object
        # self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.cv_reference = (cv2.imread('/home/turtlebot/turtlebot3_ws/src/va55/frame_reference.jpg', cv2.IMREAD_GRAYSCALE))

        self.cv_reference = cv2.resize(self.cv_reference, None, None, 0.25, 0.25)

        self.cv_reference =  cv2.blur(self.cv_reference,(3,3))

#        self.kp1, self.des1 = self.orb.detectAndCompute(self.cv_reference,None)
        self.s_start = self.cv_reference.flatten()
        self.error_list = []

    def cam_callback(self, image):
 
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='mono8')

        cv_image = cv2.resize(cv_image, None, None, 0.25, 0.25)

        cv_image = cv2.blur(cv_image,(3,3))

        cv_image_width = cv_image.shape[1]
        cv_image_height = cv_image.shape[0]

        s_current  = cv_image.flatten()

        error = s_current - self.s_start

        # self.get_logger().info(
        #     f'- cv_image = {cv_image}'
        # )

        sum_error = np.sum((error)**2)
        self.error_list.append(sum_error)


        # plt.plot(range(len(self.error_list)),self.error_list)
        # plt.title("error plot")
        # plt.ylabel("eror")
        # plt.xlabel("time")

        # fig = plt.figure()
        # fig.canvas.draw()
        # # converting matplotlib figure to Opencv image
        # plot = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
        #                      sep='')
        # plot = plot.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        # plot = cv2.cvtColor(plot, cv2.COLOR_RGB2BGR)
        # # Displaying the Combined Image:
        # cv2.imshow("Image", plot)
        # cv2.waitKey(1)


        error_image = cv2.subtract(cv_image, self.cv_reference)

        # concatenate image Horizontally
        Hori = np.concatenate((self.cv_reference, cv_image), axis=1)

        # concatenate image Vertically
        Verti = np.concatenate((Hori, error_image), axis=1)

        cv2.imshow('img', Verti)
        cv2.waitKey(1)

        cv_image_grad_u = cv2.Sobel(cv_image, cv2.CV_64F, 1, 0, ksize=3)
        cv_image_grad_v = cv2.Sobel(cv_image, cv2.CV_64F, 0, 1, ksize=3)


        lambda_gain = 10
        Z = 5



        # # find the keypoints and descriptors with ORB
        # kp2, des2 = self.orb.detectAndCompute(cv_image,None) 
        
        # # Match descriptors.
        # matches = self.bf.match(self.des1,des2)
         
        # # Sort them in the order of their distance.
        # matches = sorted(matches, key = lambda x:x.distance)

        # # Draw first 10 matches.
        # img3 = cv2.drawMatches(self.cv_reference,self.kp1,cv_image,kp2,matches[:50],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        # cv2.imshow('matches', img3)
        # cv2.waitKey(1)

        # # Loop to get U anbd u_star
        # U = []
        # U_star = []
        # best_matches = matches[:50]
        # for mat in best_matches:

        #     # Get the matching keypoints for each of the images
        #     img1_idx = mat.queryIdx
        #     img2_idx = mat.trainIdx

        #     U_star.append(self.kp1[img1_idx].pt)
        #     U.append(kp2[img2_idx].pt)


        # e = np.array(U) - np.array(U_star)
        # e = e.reshape((len(best_matches) * 2 ,1))

        au = int(700 / 4)
        av = int(700 /4)
        u0 = int(320 /4)
        v0 = int(240 / 4)

        Lu = []
        for v in range(cv_image_height):
            for u in range(cv_image_width):                
                x = (u - u0) / au
                y = (v - v0) / av

                Lu_i = np.array([cv_image_grad_u[v][u] * au*x/Z + cv_image_grad_v[v][u]*av*y/Z , cv_image_grad_u[v][u]* au * (-(1+x**2)) + cv_image_grad_v[v][u]* (-av*x*y)])
                Lu.append(Lu_i)

        Lu = np.vstack(Lu)
        Lu_pinv = np.linalg.pinv(Lu)
    
        vc = -lambda_gain * (Lu_pinv @ error)

        print(vc)
        
        
        vel_msg = Twist()
        vel_msg.linear.x = vc[0]
        vel_msg.angular.z = - 10 * vc[1]
        
        # Affichage pour debug
        self.get_logger().info(
            f'- output vel_msg = {vel_msg}'
        )
        
        self.vel_publisher.publish(vel_msg)


def main(args=None):
    
    rclpy.init(args=args)
 
    simple_suiveur = SimpleSuiveur()
    rclpy.spin(simple_suiveur)
 
    simple_suiveur.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()