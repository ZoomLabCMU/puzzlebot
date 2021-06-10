#!/usr/bin/env python
import sys
import rospy
from puzzlebot_control.hardware_wrap import HardwareWrap 

if __name__ == "__main__":
    try:
        hc = HardwareWrap(int(sys.argv[1]))
        hc.start()
    except rospy.ROSInterruptException:
        pass
