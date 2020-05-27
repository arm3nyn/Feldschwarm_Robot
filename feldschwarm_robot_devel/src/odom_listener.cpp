#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

//Publish odom_xy  laser_odom_xy delta_x  distance_gt_  distance_laserOdom deltaDistance

class OdomListener{

private:
    
    ros::NodeHandle n;
    ros::Subscriber odomSub;
    ros::Subscriber laserOdomSub;
    ros::Publisher gtOdomPub;



public:
    nav_msgs::Odometry gt_odom;
    nav_msgs::Odometry laser_odom;
    OdomListener()
    {
        gtOdomPub=n.advertise<nav_msg::Odometry>("odom_listener",1000);
        laserOdomSub=n.subscribe("chatter",1000,&OdomListener::cb_laser,this);
        odomSub=n.subscribe("chatter2",1000,&OdomListener::cb_odom,this);

    }
    ~OdomListener()
    {

    }
    void cb_laser(const sensor_msgs::PointCloud2::ConstPtr& laser_msg)
    { 
        laser_odom.pose.pose.position.x=laser_msg->pose.pose.position.x;
        laser_odom.pose.pose.position.y=laser_msg->pose.pose.position.y;


    }
    void cb_odom(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        gt_odom.pose.pose.position.x=odom_msg->pose.pose.position.x;
        gt_odom.pose.pose.position.y=odom_msg->pose.pose.position.y;
        gtOdomPub.publish(gt_odom);


    }
    
    
        
        
      

}; 

int main(int argc, char* argv[])
{
ros::init(argc, argv,"OdomListener");
OdomListener listenOdom;

ros::spin();



}