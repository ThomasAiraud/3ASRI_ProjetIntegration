from time import sleep
from hc10_test import *

deg_to_rad = 0.0174533

jointPlanning(move_group, 0, -41*deg_to_rad, 35*deg_to_rad, 0, 0)
rospy.sleep(2)
jointPlanning(move_group, 26*deg_to_rad, -42*deg_to_rad, 64*deg_to_rad, 45*deg_to_rad, -38*deg_to_rad)
rospy.sleep(2)
jointPlanning(move_group, -35*deg_to_rad, -23*deg_to_rad, 46*deg_to_rad, -77*deg_to_rad, -48*deg_to_rad)
rospy.sleep(2)
jointPlanning(move_group, -55*deg_to_rad, 19*deg_to_rad, 82*deg_to_rad, -79*deg_to_rad, -113*deg_to_rad)
rospy.sleep(2)
jointPlanning(move_group, 0, 0, 142*deg_to_rad, 0, -110*deg_to_rad)
rospy.sleep(2)
jointPlanning(move_group, 78*deg_to_rad, 12*deg_to_rad, -73*deg_to_rad, -122*deg_to_rad, 89*deg_to_rad)
#jointPlanning(move_group, -114*deg_to_rad, -44*deg_to_rad, -100*deg_to_rad, -103*deg_to_rad, -115*deg_to_rad)
rospy.sleep(2)
jointPlanning(move_group, 0, -41*deg_to_rad, 35*deg_to_rad, 0, 0)
