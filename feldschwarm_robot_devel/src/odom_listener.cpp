#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "feldschwarm_robot_devel/odom_msg.h"

//Publish odom_xy  laser_odom_xy delta_x  distance_gt_  distance_laserOdom deltaDistance

class OdomListener{

public:
    
    ros::NodeHandle n;
    ros::Subscriber odomSub;
    ros::Subscriber laserOdomSub;
    ros::Publisher odomDataPub;




    feldschwarm_robot_devel::odom_msg odom_data;
    //feldschwarm_robot_devel::odom_msg laser_odom;
    OdomListener()
    {
        odomDataPub=n.advertise<feldschwarm_robot_devel::odom_msg>("odom_listener",100);
        laserOdomSub=n.subscribe("laser_odom_to_init",100,&OdomListener::cb_laser,this);
        odomSub=n.subscribe("odom",100,&OdomListener::cb_odom,this);

    }
    ~OdomListener()
    {

    }
    
    void cb_laser(const nav_msgs::Odometry::ConstPtr& laser_msg)
    { 
        odom_data.x_laser=laser_msg->pose.pose.position.x;
        odom_data.y_laser=laser_msg->pose.pose.position.y;
        
        odomDataPub.publish(odom_data);
        ROS_INFO("laser call back");
        ROS_INFO("laser odometry x: y:(%.2f,%.2f)",odom_data.x_laser,odom_data.y_laser);


    }
    
     void cb_odom(const nav_msgs::Odometry::ConstPtr& odom_msg)
    { 
        //gt_odom.header.frame_id = "/odom";
        //gt_odom.child_frame_id = "/robot_footprint";
        odom_data.x_gt=odom_msg->pose.pose.position.x;
        odom_data.y_gt=odom_msg->pose.pose.position.y;

        
        //gt_odom.pose.pose.position.y=odom_msg->pose.pose.position.y;
        
        odomDataPub.publish(odom_data);
        ROS_INFO("gt call back");
        
        ROS_INFO("groundTruth odometry x: y:(%.2f,%.2f)",odom_data.x_gt,odom_data.y_gt);



    }
    
    
        
        
      

}; 

int main(int argc, char* argv[])
{
ros::init(argc, argv,"OdomListener");
OdomListener listenOdom;

ros::spin();



}