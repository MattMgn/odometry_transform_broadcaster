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
 *  Description: Node that subscribe to imu message publisher and broadcast its associated
 *  transform frame
 *  Author: Matthieu Magnon
 *
 *  rosrun odometry_transform_broadcaster broadcast_imu
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

geometry_msgs::TransformStamped imu_trans;
void imuMessageCallback(const sensor_msgs::Imu& imu_msg);
tf::TransformBroadcaster * imu_broadcaster;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_transform_broadcaster");

    ros::NodeHandle nh;

    imu_broadcaster = new tf::TransformBroadcaster();

    ros::Subscriber imu_msg_sub = nh.subscribe("/imu", 100, imuMessageCallback);

    ROS_INFO("Imu transform broadcaster node launched");
    
    ros::spin();

    return 0;
}

void imuMessageCallback(const sensor_msgs::Imu& imu_msg)
{
    imu_trans.header.stamp = imu_msg.header.stamp;
    imu_trans.header.frame_id = imu_msg.header.frame_id;
    imu_trans.child_frame_id = "world";

    imu_trans.transform.rotation = imu_msg.orientation;

    //send the transform
    imu_broadcaster->sendTransform(imu_trans);

    return;
}
