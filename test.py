#!/ usr/ bin/ env python
import sys
import rospy
import time
sys . path . insert (1, ’/ path /to/ folder / niryo_one_python_api ’)

#example code given by the staff and modified with joints done in lab
from niryo_one_api import *
rospy . init_node (’ niryo_one_example_python_api ’)

print (" --- Start ")

n = NiryoOne ()
n. calibrate_auto ()
c = 0

try :
    print (’test counting ’. format(c))
    c = c + 1
    joints = n.get_joints()
    print ( joints )

    #origin point
    joints = [-0.98846 , -1.1007 , -0.45279 , 0.00628 , -0.11135 , -1.0275]
    n.move_joints(joints)

    #overtheblock point
    joints = [-0.96321 , -1.0474 , -0.46775 , -1.0988 , -0.01518 , 0.02025]
    n.move_joints(joints)

    #inter point
    joints = [-0.36404 , -0.58769 , -0.58444 , 0.08011 , -0.40491, 0.14678]
    n.move_joints(joints)

    #final point
    joints = [0.23902 , -1.1541 , -0.25332 , 0.13195 , -0.1974 , 0.14172]
    n.move_joints(joints)


except NiryoOneException as e:
    print(e)