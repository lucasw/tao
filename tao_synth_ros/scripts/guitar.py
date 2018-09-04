#!/usr/bin/env python

import rospy

from tao_synth_ros.msg import Force, Instrument, Output, Stop
from tao_synth_ros.srv import *

class Guitar:
    def __init__(self):
        # decay = rospy.get_param("~decay", 4)

        self.force_pub = rospy.Publisher("force", Force, queue_size=12)
        self.stop_pub = rospy.Publisher("stop", Stop, queue_size=12)

        rospy.sleep(1.0)

        self.names = ["1E", "2B", "3G", "4D", "5A", "6E"]
        pitches = [329.63, 246.94, 196.00, 146.83, 110.00, 82.41]

        # calculate fret positions
        num_frets = 12
        final_fret = 0.5
        fret_positions = []
        fr = 0.5 ** (1.0 / num_frets)
        pos = 1.0
        for i in range(num_frets + 1):
            fret_positions.append(pos)
            pos *= fr
        rospy.loginfo("fret positions: " + str(fret_positions))

        # self.play_all_strings()

        song = [
            [0, "3G"],
            [None, None],
            [0, "3G"],
            [None, None],
            [3, "2B"],
            [None, None],
            [3, "2B"],
            [None, None],
            [0, "1E"],
            [None, None],
            [0, "1E"],
            [None, None],
            [3, "2B"],
            [None, None],
            [None, None],
            [1, "2B"],
            [None, None],
            [1, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [None, None],
            [2, "3G"],
            [None, None],
            [2, "3G"],
            [None, None],
            [0, "3G"],
            [None, None],
            [3, "2B"],
            [None, None],
            [3, "2B"],
            [None, None],
            [1, "2B"],
            [None, None],
            [1, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [None, None],
            [2, "3G"],
            [None, None],
            [3, "2B"],
            [None, None],
            [3, "2B"],
            [None, None],
            [1, "2B"],
            [None, None],
            [1, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [None, None],
            [2, "3G"],
            [None, None],
            [0, "3G"],
            [None, None],
            [0, "3G"],
            [None, None],
            [3, "2B"],
            [0, "1E"],
            [None, None],
            [0, "1E"],
            [None, None],
            [3, "2B"],
            [None, None],
            [1, "2B"],
            [None, None],
            [1, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [0, "2B"],
            [None, None],
            [2, "3G"],
            [None, None],
            [2, "3G"],
            [None, None],
            [0, "3G"],
            [None, None],
            [None, None],
            ]


        for note in song:
            if note[0] != None:
                stop = Stop()
                stop.name = note[1]
                stop.instrument_name = note[1]
                stop.amount = 0.8
                stop.x = fret_positions[note[0]]
                self.stop_pub.publish(stop)
                force = Force()
                force.name = note[1]
                force.x = 0.8
                # TODO(lucasw) need lookup table for standard forces
                force.force = 0.4
                self.force_pub.publish(force)
                rospy.loginfo(str(stop.x) + " " + str(note))
                if rospy.is_shutdown():
                    return
            rospy.sleep(0.5)
        return

        # self.play_all_strings(0.5)

    def play_all_strings(self, delay=0.25):
        mag = 0.4
        for i in range(len(self.names)):
            force = Force()
            force.name = self.names[i]
            force.force = mag
            mag *= 1.33
            force.x = 0.45
            self.force_pub.publish(force)
            rospy.sleep(delay)

if __name__ == '__main__':
    rospy.init_node('guitar')
    guitar = Guitar()


