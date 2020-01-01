/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Description: Node that subscribe to odometry message publisher and broadcast its associated
 *  transform frame
 *  Author: Matthieu Magnon
 *
 *  rosrun odometry_transform_broadcaster broadcast
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::TransformStamped odom_trans;
void odomMessageCallback(const nav_msgs::Odometry& odom_msg);
tf::TransformBroadcaster * odom_broadcaster;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_transform_broadcaster");

    ros::NodeHandle nh;

    odom_broadcaster = new tf::TransformBroadcaster();

    ros::Subscriber odom_msg_sub = nh.subscribe("/odom", 100, odomMessageCallback);

    ROS_INFO("Odometry transform broadcaster node launched");
    
    ros::spin();

    return 0;
}

void odomMessageCallback(const nav_msgs::Odometry& odom_msg)
{
    odom_trans.header.stamp = odom_msg.header.stamp;
    odom_trans.header.frame_id = odom_msg.header.frame_id;
    odom_trans.child_frame_id = odom_msg.child_frame_id;

    odom_trans.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_trans.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_trans.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_trans.transform.rotation = odom_msg.pose.pose.orientation;

    //send the transform
    odom_broadcaster->sendTransform(odom_trans);

    return;
}
