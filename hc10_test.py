# Python 2/3 compatibility imports
from __future__ import print_function
from pickle import TRUE
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def collisionUpdate(scene, box_name, box_is_known=False, box_is_attached=False, timeout=5):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

def init():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("hc10_test", anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "hc10_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    return move_group, robot, scene, display_trajectory_publisher

def displayInfos(move_group, robot):
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    return planning_frame, eef_link, group_names

def jointPlanning(move_group, a=0, b=0, c=0, d=0, e=0, f=0):
    # We get the joint values from the group and change some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = a
    joint_goal[1] = b #-tau / 8
    joint_goal[2] = c
    joint_goal[3] = d #-tau / 4
    joint_goal[4] = e
    joint_goal[5] = f #tau / 6  # 1/6 of a turn

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

def posePlanning(move_group, w=1.0, x=0.4, y=0.1, z=0.4):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = w
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()

def pathPlanning(move_group, robot, display_trajectory_publisher, path=[[0.0,0.2,0.1],[0.1,0.0,0.0],[0.0,0.1,0.0]], scale=1.0):
    waypoints = []

    wpose = move_group.get_current_pose().pose
    for pose in path:
        wpose.position.x = pose[0]
        wpose.position.y = pose[1]
        wpose.position.z = pose[2]
        waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    move_group.execute(plan, wait=True)

def addBox(scene, box_name="box", w=1.0, x=0, y=0, z=0.1, size=(0.075, 0.075, 0.075)):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"
    box_pose.pose.orientation.w = w
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z # above the panda_hand frame
    scene.add_box(box_name, box_pose, size)
    if(collisionUpdate(scene, box_name, box_is_known=True)):
        print("[" + box_name + "] was well added.")
    else:
        print("[" + box_name + "] was not well added!")

def attacheBox(robot, scene, eef_link, box_name):
    grasping_group = "hc10_hand"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    if(collisionUpdate(scene, box_name, box_is_attached=True)):
        print("[" + box_name + "] was well attached.")
    else:
        print("[" + box_name + "] was not well attached!")

def detacheBox(scene, eef_link, box_name):
    scene.remove_attached_object(eef_link, name=box_name)
    if(collisionUpdate(scene, box_name, box_is_known=True)):
        print("[" + box_name + "] was well detached.")
    else:
        print("[" + box_name + "] was not well detached!")

def removeBox(scene, box_name):
    scene.remove_world_object(box_name)
    if(collisionUpdate(scene, box_name)):
        print("[" + box_name + "] was well removed.")
    else:
        print("[" + box_name + "] was not well removed!")



move_group, robot, scene, display_trajectory_publisher = init()
planning_frame, eef_link, group_names = displayInfos(move_group, robot)
box_name = "box"
