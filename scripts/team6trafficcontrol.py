#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

## Global variables stop_box_flag = 0
stop_light_flag = 0
stop_state = 1
stop_box_flag = 0

GREEN = 0
RED = 1
YELLOW = 2

prob_thresh = 0.50   
# stopbox = [1.007, 1.5554, 1.7062, 2.9193] #xmin xmax ymin ymax
stopbox = [-1.95, -1.25, -2.60, -1.55]

class SigIntException(BaseException): pass

def stop(signal, frame):
   rospy.loginfo("You pressed ctrl-c")
   raise SigIntException

def callback_odom(msg): # if the car is within the stop zone
   global stop_box_flag
   ego_x = msg.pose.pose.position.x
   ego_y = msg.pose.pose.position.y
   
   if (stopbox[0] < ego_x < stopbox[1]) and (stopbox[2] < ego_y < stopbox[3]):
      stop_box_flag = 1
   else:
      stop_box_flag = 0
   #print(stopbox[0], ego_x , stopbox[1], stopbox[2] , ego_y , stopbox[3])   

bbox_msg = BoundingBoxes()
bbox_msg_received = False

def callback_bbox(msg):
   global bbox_msg_received, bbox_msg
   bbox_msg = msg
   bbox_msg_received = True

def ros_node_main():
   global bbox_msg_received, bbox_msg
   global stop_light_flag, stop_box_flag, stop_state
   rospy.init_node('TL_Processor', anonymous=True)

   sub_bbox       = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, callback_bbox)
   sub_odom       = rospy.Subscriber('/odometry/filtered', Odometry, callback_odom, queue_size=1)
   pub_TL         = rospy.Publisher('/TL_Status', String, queue_size=5)
   pub_auto_mode  = rospy.Publisher('/auto_mode', Bool, queue_size=1, latch=True)  
   rate = rospy.Rate(2)
   try:
      while not rospy.is_shutdown():
         # main loop here
         if bbox_msg_received == True:
            # handle bbox_msg
            red_flag = False
            green_flag = False
            yellow_flag = False
            for boundingbox in bbox_msg.bounding_boxes: # iterate for multiple detections
               if boundingbox.id == GREEN:
                  green_flag = True
               elif boundingbox.id == YELLOW:
                  yellow_flag = True
               elif boundingbox.id == RED:
                  red_flag = True
            light_str = 'Green' # lowest priority
            stop_light_flag = 0
            if red_flag: # highest priority
               light_str = 'Red'
               stop_light_flag = 1
            elif yellow_flag: # next highest priority
               light_str = 'Yellow'
               stop_light_flag = 1
            light_colour = String()
            light_colour.data = light_str
            pub_TL.publish(light_colour)
            bbox_msg_received = False

         if ((stop_box_flag != 0) and (stop_light_flag != 0)) and (stop_state != 1):
            rospy.loginfo("stop")
            pub_data = Bool()
            pub_data.data = False   
            pub_auto_mode.publish(pub_data)
            stop_state = 1  

         if ((stop_box_flag != 1) or (stop_light_flag != 1)) and (stop_state != 0):
            rospy.loginfo("go")
            pub_data = Bool()
            pub_data.data = True   
            pub_auto_mode.publish(pub_data) 
            stop_state = 0   
         #print("stop light", stop_light_flag, "in box", stop_box_flag, "stop state", stop_state)
         

         try:
             rate.sleep()
         except:
             break
   except SigIntException:
      pass
   print("Quit received.")

if __name__ == '__main__':
   ros_node_main()
