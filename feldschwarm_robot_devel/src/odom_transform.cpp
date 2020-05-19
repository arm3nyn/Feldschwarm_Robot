#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

class OdomTransform{

private:

    ros::NodeHandle nh;
    ros::ServiceClient setClient;
    ros::ServiceClient getClient;

    tf::TransformListener listener;
    //tf::StampedTransform transform;
    geometry_msgs::TransformStamped transform;

    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::GetModelState getmodelstate;

    tf::TransformBroadcaster tf2Odom;
    ros::Publisher odom_pub =nh.advertise<nav_msgs::Odometry>("odom",10);
public:
    OdomTransform(){
        setClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        getClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        // Set Gazebo state
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = "feldschwarm_robot";
        modelstate.reference_frame = "world";
        setmodelstate.request.model_state = modelstate;

        // Get Gazebo state
        getmodelstate.request.model_name = "feldschwarm_robot";
        getmodelstate.request.relative_entity_name = "world";
        //nav_msgs::Odometry odom;
       
    }    
        
    void run(){
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Rate rate(100);

        while (ros::ok()){
          geometry_msgs::TransformStamped transformStamped;

          try{
         transformStamped = tfBuffer.lookupTransform("odom", "camera_init",
                                  ros::Time(0));
          }
      catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
            // sleep
      }
            

            // Get tf between odom and laser_odom
            //try{listener.lookupTransform("odom","laser_odom", ros::Time(0), transform);}
           // catch (tf::TransformException ex){continue;}
            //ROS_DEBUG(transform);

            //tf::Transform odom_to_laser(transform.getBasis(), transform.getOrigin());

            // get robot pose from Gazebo
            getClient.waitForExistence();
            getClient.call(getmodelstate);

            // get translation
            double x, y, z;
            //x = getmodelstate.response.pose.position.x;
            //y = getmodelstate.response.pose.position.y;
           // z = getmodelstate.response.pose.position.z;
           x=transform.transform.translation.x;
           y=transform.transform.translation.y;
           z=transform.transform.translation.z; 
           ROS_DEBUG("%d %d %d ",x,y,z);  
            //orientation
            geometry_msgs::Quaternion geoQuat = getmodelstate.response.pose.orientation;
            tf::Quaternion tfQuat(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);

            // calculate tf between map and odom
           // tf::Transform laser_to_map = tf::Transform(tfQuat, tf::Vector3(x, y, z)).inverse();
            //tf::Transform map_to_odom = (odom_to_laser * laser_to_map).inverse();

            // Publish TF (laser_odom to odom)
            //tf2Odom.sendTransform(tf::StampedTransform(map_to_odom, ros::Time::now(), "/map", "/odom"));
            //odom_pub.publish(odom);
            rate.sleep();
        }
    }        



};

int main(int argc, char** argv){

ros::init(argc, argv, "Feldschwarm_robot");

OdomTransform myOdom;
myOdom.run();
return 0;




}