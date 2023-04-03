//==============================================================================
// Autonomous Vehicle Library
//
// Description: Navigation node that uses measurements directly from sensors
//              to form a navigation message. If an aspect of the navigation
//              message is not measured by a sensor, it will be NaN.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  /navigation/nav (avl_navigation/NavigationMsg)
//
// Subscribers: device/ahrs (avl_devices/AhrsMsg)
//              device/velocity (geometry_msgs/Vector3)
//              device/gps (avl_devices/GpsMsg)
//              device/depth (std_msgs/Float64)
//              device/height (std_msgs/Float64)
//==============================================================================

// Node base class
#include <avl_core/node.h>
#include <avl_core/monitored_subscriber.h>

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/matrix.h>

// ROS message includes
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/AhrsMsg.h>
#include <avl_msgs/GpsMsg.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SensorNavNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        SensorNavNode constructor
    //--------------------------------------------------------------------------
    SensorNavNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Subscribers for navigation data
    MonitoredSubscriber<AhrsMsg> ahrs_sub;
    MonitoredSubscriber<geometry_msgs::Vector3> velocity_sub;
    MonitoredSubscriber<GpsMsg> gps_sub;
    MonitoredSubscriber<std_msgs::Float64> depth_sub;
    MonitoredSubscriber<std_msgs::Float64> height_sub;

    // Publisher for navigation messages
    ros::Publisher nav_pub;

    // Timer and its duration for iterations
    ros::Timer iteration_timer;
    ros::Duration iteration_duration;

    // Variables to store latest sensor data
    double roll = NAN;
    double pitch = NAN;
    double yaw = NAN;
    double vn = NAN;
    double ve = NAN;
    double vd = NAN;
    double lat = NAN;
    double lon = NAN;
    double alt = NAN;
    double depth = NAN;
    double height = NAN;

    // Settings for useing GPS yaw and velocity. If true, the corresponding
    // measurement from GPS will be used. If false, measurements from another
    // sensor will be used. From config file
    bool use_gps_yaw;
    bool use_gps_vel;

private:

    //--------------------------------------------------------------------------
    // Name:        ahrs_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void ahrs_msg_callback(const AhrsMsg& message)
    {
        roll = message.theta.x;
        pitch = message.theta.y;
        if (!use_gps_yaw)
            yaw = message.theta.z;
    }

    //--------------------------------------------------------------------------
    // Name:        velocity_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void velocity_msg_callback(const geometry_msgs::Vector3& message)
    {
        if (!use_gps_vel)
        {
            Vector3d v_b = {message.x, message.y, message.z};
            Vector3d theta_n_b = {roll, pitch, yaw};
            Matrix3d C_n_b = avl::euler_to_matrix(theta_n_b);
            Vector3d v_n = C_n_b.transpose() * v_b;
            vn = v_n(0);
            ve = v_n(1);
            vd = v_n(2);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        gps_msg_callback
    // Description: Called when a message is received on the topic. Publishes a
    //              nav message with the most recent GPS and AHRS measurements.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void gps_msg_callback(const GpsMsg& message)
    {

        lat = message.lat;
        lon = message.lon;
        alt = message.alt;

        // Calculate GPS velocity from ground speed and track angle
        double ground_speed = message.ground_speed;
        double track_angle =  message.track_angle;
        Vector3d v_n = {ground_speed*cos(track_angle),
                        ground_speed*sin(track_angle),
                        0.0};

        if (use_gps_yaw)
            yaw = message.track_angle;

        if (use_gps_vel)
        {
            vn = v_n(0);
            ve = v_n(1);
            vd = v_n(2);
        }

    }

    //--------------------------------------------------------------------------
    // Name:        depth_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void depth_msg_callback(const std_msgs::Float64& message)
    {
        depth = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        height_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void height_msg_callback(const std_msgs::Float64& message)
    {
        height = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by the monitored subscriber.
    // Arguments:   - fault: fault event structure
    //--------------------------------------------------------------------------
    void fault_callback(const avl::SubscriberFault& fault)
    {

        if (fault.topic == "device/ahrs")
        {
            roll = NAN;
            pitch = NAN;
            if (!use_gps_yaw)
                yaw = NAN;
            ahrs_sub.reset();
        }
        else if (fault.topic == "device/gps")
        {
            lat = NAN;
            lon = NAN;
            alt = NAN;
            if (use_gps_yaw)
                yaw = NAN;
            gps_sub.reset();
        }
        else if (fault.topic == "device/depth")
        {
            depth = NAN;
            depth_sub.reset();
        }
        else if (fault.topic == "device/velocity")
        {
            if (!use_gps_vel)
            {
                vn = NAN;
                ve = NAN;
                vd = NAN;
            }
            velocity_sub.reset();
        }
        else if (fault.topic == "device/height")
        {
            height = NAN;
            height_sub.reset();
        }

    }

    //--------------------------------------------------------------------------
    // Name:        iteration_callback
    // Description: Called when the iteration timer expires.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void iteration_callback(const ros::TimerEvent& event)
    {
        NavigationMsg nav_msg;
        nav_msg.roll =  avl::wrap_to_pi(roll);
        nav_msg.pitch = avl::wrap_to_pi(pitch);
        nav_msg.yaw =   avl::wrap_to_2pi(yaw);
        nav_msg.vn =    vn;
        nav_msg.ve =    ve;
        nav_msg.vd =    vd;
        nav_msg.lat =   lat;
        nav_msg.lon =   lon;
        nav_msg.alt =   alt;
        nav_pub.publish(nav_msg);
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get config file parameters
        use_gps_yaw = get_param<bool>("~use_gps_yaw");
        use_gps_vel = get_param<bool>("~use_gps_vel");

        // Set up the publishers and subscribers
        nav_pub = node_handle->advertise<NavigationMsg>("nav/inertial_nav", 1);
        ahrs_sub.subscribe("device/ahrs", 1,
            &SensorNavNode::ahrs_msg_callback,
            &SensorNavNode::fault_callback, this);
        velocity_sub.subscribe("device/velocity", 1,
            &SensorNavNode::velocity_msg_callback,
            &SensorNavNode::fault_callback, this);
        gps_sub.subscribe("device/gps", 1,
            &SensorNavNode::gps_msg_callback,
            &SensorNavNode::fault_callback, this);
        depth_sub.subscribe("device/depth", 1,
            &SensorNavNode::depth_msg_callback,
            &SensorNavNode::fault_callback, this);
        height_sub.subscribe("device/height", 1,
            &SensorNavNode::height_msg_callback,
            &SensorNavNode::fault_callback, this);

        ahrs_sub.set_message_rate(0.5);
        velocity_sub.set_message_rate(0.5);
        gps_sub.set_message_rate(0.5);
        depth_sub.set_message_rate(0.5);
        height_sub.set_message_rate(0.5);

        ahrs_sub.enable_message_rate_check(true);
        velocity_sub.enable_message_rate_check(true);
        gps_sub.enable_message_rate_check(true);
        depth_sub.enable_message_rate_check(true);
        height_sub.enable_message_rate_check(true);

        // Set up the iteration timer
        double iteration_rate = get_param<double>("~iteration_rate");
        iteration_duration = ros::Duration(1.0/iteration_rate);
        iteration_timer = node_handle->createTimer(iteration_duration,
            &SensorNavNode::iteration_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    SensorNavNode node(argc, argv);
    node.start();
    return 0;
}
