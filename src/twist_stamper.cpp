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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

geometry_msgs::TwistStamped twist_stamped_msg;
void cmdMessageCallback(const geometry_msgs::Twist& twist_msg);
ros::Publisher *stamped_pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_stamper");

    ros::NodeHandle nh;

    ros::Subscriber cmd_msg_sub = nh.subscribe("/cmd_vel", 100, cmdMessageCallback);

    stamped_pub = new ros::Publisher;

    *stamped_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_stamped", 1000);

    ROS_INFO("Twist stamper node launched");
    
    ros::spin();

    return 0;
}

void cmdMessageCallback(const geometry_msgs::Twist& twist_msg)
{
    twist_stamped_msg.header.stamp = ros::Time::now();

    twist_stamped_msg.twist = twist_msg;

    stamped_pub->publish(twist_stamped_msg);

    return;
}
