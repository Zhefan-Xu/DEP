#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
import message_filters 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from DEP.msg import Goal

target_model_name = "drone"
eps = 1e-6
first_time = True
idx = 0
def callback(all_states, goal):
	global first_time, idx
	if (first_time):
		first_time = False
		for name in all_states.name:
			if (name == target_model_name):
				break
			idx += 1
	current_pose = all_states.pose[idx]

	rospy.loginfo("Goal: %s %s %s %s", goal.x, goal.y, goal.z, goal.yaw)
	# Current x,y,z, rx, ry, rz, rw
	cx = current_pose.position.x
	cy = current_pose.position.y
	cz = current_pose.position.z
	# crx = current_pose.orientation.x
	# cry = current_pose.orientation.y
	# crz = current_pose.orientation.z
	# crw = current_pose.orientation.w
	nx = goal.x
	ny = goal.y
	nz = goal.z		
	dx = nx - cx
	dy = ny - cy
	dz = nz - cz
	# reach = abs(dx) < 0.05 and abs(dy) < 0.05 and abs(dz) < 0.05
	# rospy.loginfo(("Distance remain: %s", (abs(dx)**2+abs(dy)**2+abs(dz)**2)**0.5))
	reach = (abs(dx)**2+abs(dy)**2+abs(dz)**2)**0.5 < 0.01
	if (not reach):
		# Next Goal Position x, y, z, rx, ry, rz, rw
		target_twist.linear.x = goal.linear_velocity * dx/(dx**2+dy**2+dz**2+eps)**0.5	
		target_twist.linear.y = goal.linear_velocity * dy/(dx**2+dy**2+dz**2+eps)**0.5	
		target_twist.linear.z = goal.linear_velocity * dz/(dx**2+dy**2+dz**2+eps)**0.5
		target_twist.angular.z = goal.angular_velocity
		target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		while not rospy.is_shutdown():
			connections = pub.get_num_connections()
			if (connections > 0):
				pub.publish(target_state)
				break
			rospy.Rate(10).sleep()
	else:
		# if (goal.is_last == True):
		target_twist.linear.x = 0
		target_twist.linear.y = 0	
		target_twist.linear.z = 0
		target_twist.angular.z = 0
		target_pose = Pose()
		target_pose.position.x = nx
		target_pose.position.y = ny
		target_pose.position.z = nz
		q = quaternion_from_euler(0, 0, goal.yaw)
		target_pose.orientation.x = q[0]
		target_pose.orientation.y = q[1]
		target_pose.orientation.z = q[2]
		target_pose.orientation.w = q[3]
		# else:
			# target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		pub.publish(target_state)



rospy.init_node("p2p_path", anonymous=True)
state_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates)
goal_sub = message_filters.Subscriber("goal", Goal)
ts = message_filters.ApproximateTimeSynchronizer([state_sub, goal_sub], 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)
# rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
target_state = ModelState()
# linear_velocity = 0.3
# angular_velocity = 0.5
target_twist = Twist()
target_pose = Pose()
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
rospy.spin()