#!/usr/bin/env python

import rospy

from tao_synth_ros.msg import Force, Instrument, Output
from tao_synth_ros.srv import *

class Guitar:
    def __init__(self):
        # decay = rospy.get_param("~decay", 4)

        force_pub = rospy.Publisher("force", Force, queue_size=10)

        names = ["1E", "2B", "3G", "4D", "5A", "6E"]
        pitches = [329.63, 246.94, 196.00, 146.83, 110.00, 82.41]

        # calculate fret positions
        num_frets = 12
        final_fret = 0.5
        fret_positions = []
        fr = 0.5 ** (1.0 / num_frets)
        pos = 1.0
        for i in range(num_frets):
            pos *= fr
            fret_positions.append(pos)
        rospy.loginfo("fret positions: " + str(fret_positions))

        return
        mag = 0.4
        for i in range(len(names)):
            force = Force()
            force.name = names[i]
            force.force = mag
            mag *= 1.33
            force.x = 0.45
            force_pub.publish(force)
            rospy.sleep(0.25)

if __name__ == '__main__':
    rospy.init_node('guitar')
    guitar = Guitar()


