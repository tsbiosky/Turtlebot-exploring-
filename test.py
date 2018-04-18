import roslib
import rospy

from nav_msgs.msg import *
#global grid_map
def getmap(msg):
	#print "sdgg"    
	#print msg.data
	global grid_map
	global map_info
	#print"sb liu"
	grid_map=msg.data
	map_info=msg.info
	explore()
	#print grid_map
def explore():
	#while not rospy.is_shutdown():
	#global grid_map
	#print len(grid_map)
	width=map_info.width
	global current_x,current_y
	height=map_info.height
	#print current_x,current_y
	count=0
	grid=[]
	for  h in range(0,height):
		temp=[]
		for w in range(0,width):
			temp.append(grid_map[count])
			count+=1
		grid.append(temp)
	front_cell=[]
	for i in range(0,grid):
		for j in range(0,grid[i]):
			if grid[i][j]==0:
				try:
					up=grid[i][j+1]
				except:
					up=-1
				try:
					left=grid[i-1][j]
				except:
					left=-1
				try:
					right=grid[i+1][j]
				except:
					right=-1
				try:
					down=grid[i][j-1]
				except:
					down=-1

	if current_x!=0 and current_y!=0:
		
	#print current_x/map_info.resolution,current_y/map_info.resolution
	
def getpos(msg):
	global current_x,current_y
	current_x=msg.pose.pose.position.x
	current_y=msg.pose.pose.position.y
	#print msg.pose.pose.position.x,msg.pose.pose.position.y
		
if __name__ == "__main__":
	rospy.init_node('OccupancyGrid', anonymous=True) #make node 
	#global grid_map
	current_x=0
	current_y=0
	rospy.Subscriber('odom',Odometry,getpos)
    	rospy.Subscriber('/map',OccupancyGrid,getmap)
    	rospy.spin()
	#explore()
    	
