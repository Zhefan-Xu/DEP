#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
import message_filters 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from DEP.msg import Goal
import math

target_model_name = "drone"
eps = 1e-6
first_time = True
idx = 0
rotate = False
def callback(all_states, goal):
	global first_time, idx, rotate
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
	crx = current_pose.orientation.x
	cry = current_pose.orientation.y
	crz = current_pose.orientation.z
	crw = current_pose.orientation.w
	(current_roll, current_pitch, current_yaw) = euler_from_quaternion([crx, cry, crz, crw])
	if current_yaw < 0:
		current_yaw = 2*math.pi - (-current_yaw)
	rospy.loginfo("current_yaw: %s", current_yaw)

	nx = goal.x
	ny = goal.y
	nz = goal.z		
	dx = nx - cx
	dy = ny - cy
	dz = nz - cz
	# reach = abs(dx) < 0.05 and abs(dy) < 0.05 and abs(dz) < 0.05
	# rospy.loginfo(("Distance remain: %s", (abs(dx)**2+abs(dy)**2+abs(dz)**2)**0.5))
	reach = (abs(dx)**2+abs(dy)**2+abs(dz)**2)**0.5 < 0.02
	if (not reach):
		# Next Goal Position x, y, z, rx, ry, rz, rw
		target_twist.linear.x = linear_velocity * dx/(dx**2+dy**2+dz**2+eps)**0.5	
		target_twist.linear.y = linear_velocity * dy/(dx**2+dy**2+dz**2+eps)**0.5	
		target_twist.linear.z = linear_velocity * dz/(dx**2+dy**2+dz**2+eps)**0.5
		# target_twist.angular.z = goal.angular_velocity
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
	elif (not rotate):
		# if (goal.is_last == True):
		target_twist.linear.x = 0
		target_twist.linear.y = 0	
		target_twist.linear.z = 0
		target_twist.angular.z = 0
		target_pose = Pose()
		target_pose.position.x = nx
		target_pose.position.y = ny
		target_pose.position.z = nz
		# q = quaternion_from_euler(0, 0, goal.yaw)
		# target_pose.orientation.x = q[0]
		# target_pose.orientation.y = q[1]
		# target_pose.orientation.z = q[2]
		# target_pose.orientation.w = q[3]
		# else:
			# target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		pub.publish(target_state)
		rotate = True

	# Then move yaw:
	dyaw = (goal.yaw - current_yaw)
	rospy.loginfo("dyaw: %s", dyaw)
	rotation_finish = abs(dyaw) < 0.06
	if (reach):
		if (not rotation_finish):
			target_twist.linear.x = 0
			target_twist.linear.y = 0
			target_twist.linear.z = 0
			if (abs(dyaw) < math.pi):
				angular_velocity_direction = dyaw/abs(dyaw)
			else:
				angular_velocity_direction = -dyaw/abs(dyaw)
			target_twist.angular.z = angular_velocity * angular_velocity_direction
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
linear_velocity = 0.3
angular_velocity = 0.8
target_twist = Twist()
target_pose = Pose()
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
rospy.spin()