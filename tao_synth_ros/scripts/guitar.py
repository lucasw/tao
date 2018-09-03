#!/usr/bin/env python

import rospy

from tao_synth_ros.msg import Force
from tao_synth_ros.srv import *

class Guitar:
    def __init__(self):
        rospy.wait_for_service('add_instrument')
        # TODO(lucasw) need an add_compound service
        self.add_instrument = rospy.ServiceProxy('add_instrument', AddInstrument)
        req = AddInstrumentRequest()
        req.decay = rospy.get_param("~decay", 50)

        force_pub = rospy.Publisher("force", Force, queue_size=10)

        names = ["1E", "2B", "3G", "4D", "5A", "6E"]
        pitches = [329.63, 246.94, 196.00, 146.83, 110.00, 82.41]

        for i in range(len(names)):
            req.name = names[i]
            req.pitch_x = pitches[i]
            try:
                resp = self.add_instrument(req)
                rospy.loginfo(resp)
            except rospy.service.ServiceException as e:
                rospy.logerr(e)
            req.position.y += 1.0

        rospy.sleep(1.0)

        for i in range(len(names)):
            force = Force()
            force.name = names[i]
            force.force = 0.8
            force.x = 0.45
            force_pub.publish(force)
            rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('guitar')
    guitar = Guitar()


