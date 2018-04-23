import rospy
from geometry_msgs.msg import Twist

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward',anonymous=False)
		rospy.loginfo("ctrl+c to stop")
		rospy.on_shutdown(self.shutdown)
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if 			you're not using TurtleBot2
		self.cmd_vel=rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
		r=rospy.Rate(10);
#0,1s
		move_cmd=Twist()
		move_cmd.linear.x=0.2
		move_cmd.angular.z=0
		while not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			r.sleep()
	def shutdown(self):
		rospy.loginfo("stop turtlebot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
if __name__=='__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("node terminated")
