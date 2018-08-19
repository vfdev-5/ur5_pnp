#!/usr/bin/env python
# BEGIN ALL
# BEGIN SHEBANG
#!/usr/bin/env python
# END SHEBANG

#https://github.com/osrf/rosbook/blob/master/code/basics/src/topic_publisher.py
# http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers

# BEGIN IMPORT
import rospy
# END IMPORT

# BEGIN STD_MSGS
from std_msgs.msg import String
# END STD_MSGS


rospy.init_node('topic_publisher')

# BEGIN PUB
pub = rospy.Publisher('toArduino', String, queue_size=100)
pub2 = rospy.Publisher('fromArduino', String, queue_size=100)
pub3 = rospy.Publisher('chatter', String, queue_size=100)
# END PUB

# BEGIN LOOP
rate = rospy.Rate(0.2)

count = 0
while not rospy.is_shutdown():
    if count % 2 == 0:
        print("SUCKER:OFF")
        pub.publish("SUCKER:OFF")
    else:
        print("SUCKER:ON")
        pub.publish("SUCKER:ON")
    count += 1
    rate.sleep()
# END LOOP
# END ALL
