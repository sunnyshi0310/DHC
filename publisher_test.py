#!/usr/bin/env python

import rospy # MUST import it if you are writting a ROS node
from std_msgs.msg import String # Import a predefined message type "String"

def talker():
    rospy.init_node('talker') # Initialize the node with name "talker"
    pub = rospy.Publisher('chatter',String,queue_size = 10) # Declare a publisher that your node "talker" will publish messages to the topic "chatter". The format of the message is defined as "String", i.e. the topic using the message type "String". 
    
    rate = rospy.Rate(10) # 10 hz
    
    while not rospy.is_shutdown():
        content = "welcome to EE175 %s"
        
        pub.publish(content) # Please make sure the content has format "String" (consistent with what we declared in pub)
        
        rate.sleep() # Offer a convenient way for looping at the desired time
        
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
