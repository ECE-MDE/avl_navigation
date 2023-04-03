//==============================================================================
// Autonomous Vehicle Library
//
// Description: Converts a pressure message in PSI to a depth in meters using
//              the simple relationship:
//                  P = rho * g * h    --->   h = P / (rho * g)
//              where P is the pressure in PSI, rho is the density of water,
//              and g is the acceleration of gravity, and h is the depth in
//              meters.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/depth (std_msgs/Float64)
//
// Subscribers: device/pressure (std_msgs/Float64)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// ROS messages and services
#include <std_msgs/Float64.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class DepthNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        DepthNode constructor
    //--------------------------------------------------------------------------
    DepthNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Subscriber for pressure messages
    ros::Subscriber pressure_sub;

    // Publisher for depth messages
    ros::Publisher depth_pub;

    // Depth measurement
    double depth = 0;

    // Density of water in kg/m^3 from the config file
    double rho;

    // Acceleration due to gravity in m/s^2 from the config file
    double g;

    // Exponential filter smoothing factor from the config file
    double alpha;

    // PSI to Pa conversion constant
    const double PSI_TO_PA = 6894.7572932;

private:

    //--------------------------------------------------------------------------
    // Name:        pressure_msg_callback
    // Description: Called when apressure message is received. Calculates and
    //              publishes the corresponding depth.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void pressure_msg_callback(const std_msgs::Float64& message)
    {

        // Simple depth calculation:
        //     P = rho * g * h    --->   h = P / (rho * g)
        double P = message.data * PSI_TO_PA;
        double h = P / (rho * g);

        // Exponential filter
        depth = alpha*h + (1-alpha)*depth;

        // Create and publish message
        std_msgs::Float64 depth_msg;
        depth_msg.data = depth;
        depth_pub.publish(depth_msg);

        log_data("[depth] %.4f %.4f %.4f", P, h, depth);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[depth] pressure depth_raw depth_filt");
        add_data_header("[depth] Pa m m");

        // Set up the publishers and subscribers
        pressure_sub = node_handle->subscribe("device/pressure", 1,
            &DepthNode::pressure_msg_callback, this);
        depth_pub = node_handle->advertise<std_msgs::Float64>("device/depth", 1);

        // Get the config file parameters
        rho = get_param<double>("~water_density");
        g = get_param<double>("~gravity_acceleration");
        alpha = get_param<double>("~alpha");

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
    DepthNode node(argc, argv);
    node.start();
    return 0;
}
