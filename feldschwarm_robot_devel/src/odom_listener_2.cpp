#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace gazebo {
    class OdomDriftPlugin : public ModelPlugin {
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        double alpha;
        double odomrate;
        std::string odomtopic;
        bool publishgroundtruth;
        std::string velocitytopic;
        double vel;
        ros::Publisher odompub;
        ros::Subscriber velsub;
        geometry_msgs::Vector3 truepose_old;
        geometry_msgs::Vector3 odomdrifted;
        geometry_msgs::Vector3 velcmd;
        std::thread thread;

    public:
        OdomDriftPlugin() {}

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&OdomDriftPlugin::OnUpdate, this));

            // Read the plugin settings from the model XML file
            this->alpha = _sdf->GetElement("alpha")->GetValue();
            this->odomrate = _sdf->GetElement("odomrate")->GetValue();
            this->odomtopic = _sdf->GetElement("odomtopic")->GetValue();
            this->publishgroundtruth = _sdf->GetElement("publishgroundtruth")->GetValue();
            this->velocitytopic = _sdf->GetElement("velocitytopic")->GetValue();

            // Convert the vel element value to double
            sdf::ElementPtr velElement = _sdf->GetElement("vel");
            sdf::ParamPtr velParam = velElement->GetValue();
            this->vel = std::stod(velParam->GetAsString());

            // Initialize ROS node
            int argc = 0;
            char **argv;
            ros::init(argc, argv, "gazebo_odom");
            ros::NodeHandle n;

            // Initialize publisher and subscriber
            this->odompub = n.advertise<nav_msgs::Odometry>("odom_drifted", 1);
            this->velsub = n.subscribe("/cmd_vel", 1, &OdomDriftPlugin::velocityCallback, this);

            // Initialize class variables
            this->odomdrifted.x = 0;
            this->odomdrifted.y = 0;
            this->odomdrifted.z = 0;
            this->truepose_old.x = 0;
            this->truepose
