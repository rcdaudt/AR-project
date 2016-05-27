#!/usr/bin/env python

import roslib 
import rospy

from driver import driver

if __name__ == '__main__':
    try:
        pilot = driver()
        pilot.load_goals()
        pilot.drive()
            
    except rospy.ROSInterruptException:
        pass
