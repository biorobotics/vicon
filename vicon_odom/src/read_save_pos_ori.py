#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rospy
import tf
from tf import transformations as ts
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Transform, Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

from csv import writer
inp = None

def write_row(file_name, List):
    with open("odometry_data_"+file_name+'.csv', 'a') as f_object:
     
        # Pass this file object to csv.writer()
        # and get a writer object
        writer_object = writer(f_object)
     
        # Pass the list as an argument into
        # the writerow()
        writer_object.writerow(List)
     
        # Close the file object
        f_object.close()

def callback(data):
    print("data is heard")
    pos = data.pose.pose.position
    ori = data.pose.pose.orientation
    timestamp = data.header.stamp
    time = float(timestamp.secs) + float(timestamp.nsecs)/1e9
    List = [time, pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
    write_row(inp,List)
    #print("data is: ", List)
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    # eigenbot_leg_marker
    rospy.Subscriber("/sparse_eigenbot_marker/odom", Odometry, callback)
    # rospy.Subscriber("/eigenbot_leg_marker/odom", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    inp = raw_input('name of the csv file:\n')
    List = ['timestamp','px', 'py', 'pz','ox', 'oy', 'oz','ow']
    write_row(inp,List)
    listener()