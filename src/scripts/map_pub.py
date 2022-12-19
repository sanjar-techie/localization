#!/usr/bin/env python2
# Software License Agreement (BSD License)

import rospy
import sys
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

map_got = False
map_obj = OccupancyGrid()

# https://stackoverflow.com/questions/40438261/using-sigint-to-kill-a-function-in-python-3
class SigIntException(BaseException): pass

def stop(signal, frame):
    print("You pressed ctrl-c")
    raise SigIntException

def callback(data):
    global map_got, map_pub
    if not map_got:
        map_obj = data
        map_got = True
    return

def map_pub(rate):
    global map_got, map_pub
    rospy.init_node('map_pub', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, callback, queue_size=1)
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    rate = rospy.Rate(rate) # 10hz

    try:
        while not rospy.is_shutdown():
            if map_got:
                pub.publish(map_obj)
            try:
                rate.sleep()
            except:
                break

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except SigIntException:
        pass
    rospy.loginfo('Quit received.')

if __name__ == '__main__':
    try:
        rate = 30
        for argv_element in sys.argv:
            try: 
                float(argv_element)
            except ValueError:
                continue
            rate = float(argv_element)
            break
        map_pub(rate)
    except rospy.ROSInterruptException:
        pass
