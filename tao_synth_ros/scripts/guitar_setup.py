#!/usr/bin/env python

import rospy

from tao_synth_ros.msg import Force, Instrument, Output
from tao_synth_ros.srv import *

class GuitarSetup:
    def __init__(self):
        rospy.wait_for_service('add_assembly')
        # TODO(lucasw) need an add_compound service
        self.add_assembly = rospy.ServiceProxy('add_assembly', AddAssembly)
        req = AddAssemblyRequest()
        # The units might be seconds, smaller decay is a more quickly decaying sound
        decay = rospy.get_param("~decay", 4)

        force_pub = rospy.Publisher("force", Force, queue_size=10)

        names = ["1E", "2B", "3G", "4D", "5A", "6E"]
        pitches = [329.63, 246.94, 196.00, 146.83, 110.00, 82.41]

        y_pos = 0.0
        for i in range(len(names)):
            instr = Instrument()
            instr.name = names[i]
            instr.pitch_x = pitches[i]
            instr.position.y = y_pos
            instr.decay = decay
            y_pos += 1.0
            req.instruments.append(instr)

            output = Output()
            output.name = instr.name + "_output"
            output.instrument_name = instr.name
            output.x = 0.9
            req.outputs.append(output)

        try:
            resp = self.add_assembly(req)
            rospy.loginfo(resp)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

        rospy.sleep(2.0)

        mag = 0.4
        for i in range(len(names)):
            force = Force()
            force.name = names[i]
            force.force = mag
            mag *= 1.2
            force.x = 0.45
            force_pub.publish(force)
            rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('guitar_setup')
    guitar_setup = GuitarSetup()


