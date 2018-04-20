import roslib
import rospy
#import rospy
from geometry_msgs.msg import Twist
from math import radians
from nav_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import numpy as np
#global grid_map
def getmap(msg):
	global current_x,current_y,yaw_x,yaw_y,yaw_z,yaw_w,rad
	#print "sdgg"    
	#print msg.data
	global grid_map
	global map_info
	#print"sb liu"
	grid_map=msg.data
	map_info=msg.info
	#print current_x,current_y
	#move()
	explore()
	#print grid_map
def get_gain(grid,x,y,reso):
	up=y-int(0.5/reso)
	gain=0
	down=y+int(0.5/reso)
	#print up,down,y
	for h in range(up,down):
		for w in range(x-(int(0.5/reso)-h+y),x+(int(0.5/reso)-h+y)):
			try:
				if grid[w][h]==-1:
						gain+=1
			except:
				a=0
	return gain			
def explore():
	#while not rospy.is_shutdown():
	#global grid_map
	#print len(grid_map)
	width=map_info.width
	global current_x,current_y,yaw_x,yaw_z,yaw_y,yaw_w,rad
	height=map_info.height
	print current_x,current_y
	ori_x=map_info.origin.position.x
	ori_y=map_info.origin.position.y
	reso=map_info.resolution
	current_grid_x=int((current_x-ori_x)/reso)
	current_grid_y=int((current_y-ori_y)/reso)
	if current_x==0 and current_y==0:
		return
	
	count=0
	grid=[]
	for  h in range(0,height):
		temp=[]
		for w in range(0,width):
			temp.append(grid_map[count])
			count+=1
		grid.append(temp)
	front_cell=[]
	for i in range(0,len(grid)):
		for j in range(0,len(grid[i])):
			if grid[i][j]==0:
				try:
					up=grid[i][j+1]
				except:
					up=-100
				try:
					left=grid[i-1][j]
				except:
					left=-100
				try:
					right=grid[i+1][j]
				except:
					right=-100
				try:
					down=grid[i][j-1]
				except:
					down=-100
				if up==-1 or left==-1 or right==-1 or down==-1:
					tt=(i,j)
					front_cell.append(tt)
	beta=10
	max_gain=0
	t_x=0
	t_y=0
	#get info gain
	
	for i in range(0,len(front_cell)):
		y=front_cell[i][0]
		x=front_cell[i][1]
		info_gain=get_gain(grid,x,y,reso)
		dis=math.sqrt((x-current_grid_x)**2+(y-current_grid_y)**2)
		gain=info_gain-dis*beta
		if gain>=max_gain:
			t_x=x
			t_y=y
			max_gain=gain
	print max_gain
	target_x=ori_x+t_x*reso
	target_y=ori_y+t_y*reso
	print "target:"+str(target_x)+" "+str(target_y)
	print "current:"+str(current_x)+" "+str(current_y)
	#move to target
	
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	move_base.wait_for_server(rospy.Duration(5))
	goal = MoveBaseGoal()
	last_x=current_x
	last_y=current_y
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose = Pose(Point(target_x, target_y, 0.000),Quaternion(0,0,0,1))
	move_base.send_goal(goal)
	success = move_base.wait_for_result(rospy.Duration(600))
	# plan B
	rospy.Subscriber('odom',Odometry,getpos)
	#print last_x,last_y
	#print current_x,current_y
	move_dis=((current_x-last_x)**2+(current_y-last_y)**2)**0.5
	
	if move_dis<0.1:
		print"explore "
		turnaround(2*math.pi)
		move(target_x,target_y)
	d_x=target_x-last_x
	d_y=target_y-last_y
	dd=d_y/d_x
	f_dis=(d_x**2+d_y**2)**0.5
	if d_x<0:
		f_x=target_x-(1/dd**2+1)**0.5
	else:
		f_x=target_x+ (1/dd**2+1)**0.5
	if d_y<0:
		f_y=target_y-(dd**2/dd**2+1)**0.5
	else:
		f_y=target_y+ (dd**2/dd**2+1)**0.5
	f_y_c=int((f_y-ori_y)/reso)
	f_x_c=int((f_x-ori_x)/reso)
	#vc=911
	
	try:
		vc=grid[f_x_c][f_y_c]
		
	except:
		vc=100
	if f_x_c<0 or f_y_c<0:
		vc=100
	#print "vc:"+str(f_x)+str(f_y)
	#print d_x,d_y
	#print "f_dis:"+str(f_dis)
	if f_dis<0.5:
		if vc!=100:
			'''
			goal.target_pose.pose = Pose(Point(f_x, f_y,0.000),Quaternion(0,0,0,1))
			move_base.send_goal(goal)
			success = move_base.wait_for_result(rospy.Duration(600))
			
			print "vc:"+str(f_x)+str(f_y)
			print "vc_grid:"+str(f_x_c)+" "+str(f_y_c)+" "+str(vc)
			'''
			print "fix"
			move(f_x,f_y)
'''
			f_rad=np.arctan((f_x-target_x)/(f_y-target_y))
			cmd_vel=rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
			r=rospy.Rate(1);
			move_cmd=Twist()
			move_cmd.linear.x=0
			move_cmd.angular.z=(f_rad-rad)/2
			for i in range(0,2):
				cmd_vel.publish(move_cmd)
				r.sleep()
			move_cmd.linear.x=((f_x-target_x)**2+(f_y-target_y)**2)**0.5*0.5
			move_cmd.linear.y=0
			move_cmd.angular.z=0
			for i in range(0,2):
				cmd_vel.publish(move_cmd)
				r.sleep()
			turnaround(2*math.pi)
'''
	#turnaround(2*math.pi)
	
	#print current_grid_x,current_grid_y
		
	#print current_x/map_info.resolution,current_y/map_info.resolution
def turnaround(angle):
	cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	twist=Twist()
	twist.angular.z=angle/2
	r = rospy.Rate(10)
	for i in range(0,2):
		cmd_vel.publish(twist)
		r.sleep()
def turn(target_x,target_y):
	print "turn"
	global current_x,current_y,yaw_x,yaw_y,yaw_z,yaw_w,rad
	cmd_vel=rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
	r=rospy.Rate(1)
	rad=math.atan2(2*(yaw_w*yaw_z+yaw_x*yaw_y),1-2*(yaw_y*yaw_y+yaw_z*yaw_z))	
	target_rad=math.atan2(target_y-current_y,target_x-current_x)
	speed=0.1
	last=abs(rad-target_rad)
	while abs(rad-target_rad)>0.05:
			#print abs(rad-target_rad),rad,target_rad
			move_cmd=Twist()
			move_cmd.linear.x=0
			if last>=abs(rad-target_rad):
				speed=speed
			else:
				speed=-speed
			move_cmd.angular.z=speed
			last=abs(rad-target_rad)
			cmd_vel.publish(move_cmd)
			rospy.Subscriber('odom',Odometry,getpos)
			
			rad=math.atan2(2*(yaw_w*yaw_z+yaw_x*yaw_y),1-2*(yaw_y*yaw_y+yaw_z*yaw_z))
			
			
			r.sleep()
def move(target_x,target_y):
		global current_x,current_y,yaw_x,yaw_y,yaw_z,yaw_w,rad
		cmd_vel=rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
		r=rospy.Rate(1)
		#target_x=6
		#target_y=6
		#print current_x, current_y
		
		turn(target_x,target_y)
		
		dis=((target_x-current_x)**2+(target_y-current_y)**2)**0.5
		last_dis=dis
		speed=0.5
		print "go"
		while dis>0.5:
			
			move_cmd=Twist()
			
			if last_dis>=dis:
				turn(target_x,target_y)
			else:
				speed=-speed
			move_cmd.linear.x=speed
			move_cmd.angular.z=0
			last_dis=dis
			cmd_vel.publish(move_cmd)
			dis=((target_x-current_x)**2+(target_y-current_y)**2)**0.5
			print dis,current_x,current_y
			rospy.Subscriber('odom',Odometry,getpos)
			r.sleep()
		
def getpos(msg):
	global current_x,current_y,yaw_x,yaw_y,yaw_z,yaw_w,rad
	current_x=msg.pose.pose.position.x
	current_y=msg.pose.pose.position.y
	#print current_x, current_y	
	yaw_x=msg.pose.pose.orientation.x
	yaw_y=msg.pose.pose.orientation.y
	yaw_z=msg.pose.pose.orientation.z
	yaw_w=msg.pose.pose.orientation.w
	rad=math.atan2(2*(yaw_w*yaw_z),1-2*(yaw_z**2))
	#move()
	
	#print current_x,current_y
	#print msg.pose.pose.position.x,msg.pose.pose.position.y
		
if __name__ == "__main__":
	rospy.init_node('OccupancyGrid', anonymous=True) #make node 
	#global grid_map
	current_x=0
	current_y=0
	current_z=0
	yaw_x=0
	yaw_y=0
	yaw_z=0
	yaw_w=0
	rad=0
	rospy.Subscriber('odom',Odometry,getpos)
	#move()
    	rospy.Subscriber('/map',OccupancyGrid,getmap)
    	rospy.spin()
	#explore()
    	
