#!/usr/bin/env python3

import rospy
from sm import CarryLuggage

if __name__ == '__main__':
    rospy.init_node("carry_luggage")
    carry_luggage = CarryLuggage()
    outcome = carry_luggage.execute()
    print(outcome)
    rospy.spin()