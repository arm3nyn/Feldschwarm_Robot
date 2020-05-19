#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h> 
class OdomListener{

private:
    
    ros::Subscriber odom_sub;
    ros::Subscriber laser_odom_sub;

public:

    void cb_laser(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {

    }
    void cb_odom(const)
    {

    }
    OdomListener()
    {
        ros::NodeHandle n;
        laser_odom_sub=n.subscribe("chatter",10,&OdomListener::cb_laser,this);
        odom_sub=n.subscribe("chatter2",10,&OdomListener::cb_odom,this);
    }   

}; 

int main(int argc, char* argv[])
{
ros::init(argc, argv,"OdomListener");
OdomListener listenOdom;

while(ros::ok)
{

}



}