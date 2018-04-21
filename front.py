import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from nav_msgs.msg import *
def getmap(msg):
	#print "sdgg"    
	#print msg.data
	#print"sb liu"
	grid_map=msg.data
	map_info=msg.info
	global grid 
	global front_cell
	global ori_x
	global ori_y
	ori_x=map_info.origin.position.x
	ori_y=map_info.origin.position.y
	grid=[]
	width=map_info.width
	height=map_info.height
	count=0
	cell=[]
	for  h in range(0,height):
		temp=[]
		for w in range(0,width):
			temp.append(grid_map[count])
			count+=1
		grid.append(temp)
	front_cell=[]
	#print len(grid),height
	for i in range(0,len(grid)):
		#print(len(grid)),len(grid[i])
		for j in range(0,len(grid[i])):
			if grid[i][j]==0:
					try:
						up=grid[i][j+1]
					except:
						up=100
					try:
						left=grid[i-1][j]
					except:
						left=100
					try:
						right=grid[i+1][j]
					except:
						right=100
					try:
						down=grid[i][j-1]
					except:
						down=100
					if up==-1 or left==-1 or right==-1 or down==-1:
						if up<1 and left<1 and right <1 and down <1:
							tt=(i,j)
							front_cell.append(tt)
	#print len(front_cell)	
	draw()	
	#print len(front_cell)
def draw():
	global ori_x,ori_y,front_cell
	#print len(front_cell)
	topic = 'visualization_marker_array'
	publisher = rospy.Publisher(topic, MarkerArray,queue_size=100)
	
	#rospy.init_node('register')
	
	markerArray = MarkerArray()
	count = 0
	MARKERS_MAX = 100
	while not rospy.is_shutdown():
		#print len(front_cell)
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 255.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = ori_x+front_cell[count%len(front_cell)][1]*0.05
		marker.pose.position.y = ori_y+front_cell[count%len(front_cell)][0]*0.05
		marker.pose.position.z = 0
		#print marker.pose.position.x,marker.pose.position.y
		# We add the new marker to the MarkerArray, removing the oldest
		# marker from it# when necessary
		if(count > MARKERS_MAX):
	       		markerArray.markers.pop(0)
		markerArray.markers.append(marker)
	
		# Renumber the marker IDs
		#print (xmax-xmin)*0.05,(ymax-ymin)*0.05
	
		# Publish the MarkerArray
		publisher.publish(markerArray)
		count += 1
		rospy.sleep(0.01)
if __name__ == "__main__":
	grid=[]
	front_cell=[]
	ori_x=None
	ori_y=None
	rospy.init_node('OccupancyGrid')
	rospy.Subscriber('/map',OccupancyGrid,getmap)
	rospy.spin()
