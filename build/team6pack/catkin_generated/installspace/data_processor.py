#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int32

class ROS_pub_sub():
    def __init__(self):
        #Init ros node
        rospy.init_node('talker_listener', anonymous=True)

        #Define publisher and subscriber
        self.sub = rospy.Subscriber('/MinjuHong', Int32, self.callback_chatter)
        self.pub_processed = rospy.Publisher('/MinjuHong_processed', Int32, queue_size=10)

        #Define ros node rate
        self.rate = rospy.Rate(30)  #30Hz
  
    def callback_chatter(self, msg):
        #Parse the string data in the message
        chat_data = msg.data
        rospy.loginfo("I heard %d", chat_data)

        #Process
        processed_chat_data = chat_data + 111

        #Publish a processed message
        msg_processed = Int32()
        msg_processed.data = processed_chat_data
        self.pub_processed.publish(msg_processed)

        rospy.loginfo("I sent %d", processed_chat_data)

def main():
    #Create a class instance
    pub_sub_node = ROS_pub_sub()

    #Main loop
    while not rospy.is_shutdown():
        
        #Rate control
        pub_sub_node.rate.sleep()

if __name__ == '__main__':
    main()
    