import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from nav_msgs.msg import *
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker,queue_size=100)

rospy.init_node('register')

markerArray = MarkerArray()
target_x=0
target_y=0
count = 0
MARKERS_MAX = 100
ori_x=None
ori_y=None
def callback(msg):
	global target_x,target_y
	(a,b)=msg.data.split(' ')
	target_x=float(a)
	target_y=float(b)
def getmap(msg):
	grid_map=msg.data
	map_info=msg.info
	global ori_x
	global ori_y
	ori_x=map_info.origin.position.x
	ori_y=map_info.origin.position.y
while not rospy.is_shutdown():
   rospy.Subscriber("s_target", String, callback)
   rospy.Subscriber('/map',OccupancyGrid,getmap)
   marker = Marker()
   marker.header.frame_id = "/map"
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 0.2
   marker.scale.y = 0.2
   marker.scale.z = 0.2
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 0.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   print target_x,target_y
   marker.pose.position.x = target_x
   marker.pose.position.y =target_y
   #marker.pose.position.x = ori_x	
   #marker.pose.position.y = ori_y
   marker.pose.position.z = 0

   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
   #if(count > MARKERS_MAX):
       #markerArray.markers.pop(0)

   markerArray.markers.append(marker)

   # Renumber the marker IDs
   id = 0
   for m in markerArray.markers:
       m.id = id
       id += 1

   # Publish the MarkerArray
   publisher.publish(marker)

   count += 1

   rospy.sleep(0.01)
