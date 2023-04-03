//==============================================================================
// Autonomous Vehicle Library
//
// Description: Processes published sensor data real-time through a navigation
//              filter and publishes the navigation messages generated from the
//              navigation filter estimate.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  nav/gyrocompass (avl_msgs/GyrocompassMsg)
//
// Subscribers: device/imu (avl_msgs/ImuMsg)
//              device/gps (avl_msgs/GpsMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/time.h>

// Inertial navigation filter
#include <avl_navigation/algorithm/gyrocompass_mukf.h>

// ROS message includes
#include <avl_msgs/GyrocompassMsg.h>
#include <avl_msgs/ImuMsg.h>
#include <avl_msgs/GpsMsg.h>
using namespace avl_msgs;

// Alias for double vector
typedef std::vector<double> doubles_t;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class GyrocompassNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        GyrocompassNode constructor
    //--------------------------------------------------------------------------
    GyrocompassNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Gyrocompass filter
    GyrocompassMukf mukf;
    MatrixXd P0;
    MatrixXd Q;
    MatrixXd R;

    // Flag indicating if the filter has been initialized
    bool initialized = false;

    // Publishers and subscribers
    ros::Publisher gyrocompass_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;

    // Most recent IMU measurements
    Vector3d w_ib_b;
    Vector3d f_ib_b;

    // Most recent GPS position
    Vector3d p_b;

    // Minimum number of sats for a GPS lock from config file
    int min_sats;

private:

    //--------------------------------------------------------------------------
    // Name:        imu_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void imu_msg_callback(const ImuMsg& msg)
    {

        // Ignore invalid measurements
        if (!msg.valid)
            return;

        // Turn the IMU measurements into vectors
        w_ib_b(0) = msg.angular_velocity.x;
        w_ib_b(1) = msg.angular_velocity.y;
        w_ib_b(2) = msg.angular_velocity.z;

        f_ib_b(0) = msg.linear_acceleration.x;
        f_ib_b(1) = msg.linear_acceleration.y;
        f_ib_b(2) = msg.linear_acceleration.z;

        // If the filter is initialized, iterate
        if (initialized)
        {

            // Iterate the filter using the IMU data
            mukf.iterate(w_ib_b, f_ib_b, msg.dt, p_b);
            VectorXd x = mukf.get_state();
            VectorXd P = mukf.get_cov();

            // Create and publish a gyrocompass message
            GyrocompassMsg gc_msg;
            gc_msg.roll =  x(3);
            gc_msg.pitch = x(4);
            gc_msg.yaw =   x(5);
            gc_msg.roll_std =  sqrt(P(9));
            gc_msg.pitch_std = sqrt(P(10));
            gc_msg.yaw_std =   sqrt(P(11));
            gyrocompass_pub.publish(gc_msg);

            // Log the state and covariance
            log_data("[x] %s", avl::to_string(x, 9).c_str());
            log_data("[P] %s", avl::to_string(P, 9).c_str());

        }

    }

    //--------------------------------------------------------------------------
    // Name:        gps_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - msg: message received on the topic
    //--------------------------------------------------------------------------
    void gps_msg_callback(const GpsMsg& msg)
    {

        // Construct the GPS position vector
        Vector3d p_b_meas = {msg.lat, msg.lon, msg.alt};

        // Only process the GPS measurement if it has a lock
        if (p_b_meas.allFinite() && msg.num_sats >= min_sats)
        {

            // If the filter is already initialized, just save the position
            if (initialized)
            {
                p_b = p_b_meas;
                log_data("[gps] %d %0.9f %0.9f %0.3f",
                    msg.num_sats, msg.lat, msg.lon, msg.alt);
            } // Nav initialized

            // If the filter is not yet initalized, initialize it. Only
            // initialize if the acceleration is close to the acceleration of
            // gravity
            else if (abs(f_ib_b.norm() - 9.81) < 0.1)
            {

                // Calculate initial attitude from IMU
                Vector3d theta_n_b = attitude_from_imu(w_ib_b, f_ib_b);
                Matrix3d C_n_b = euler_to_matrix(theta_n_b);

                // Set initial state
                GyroState x0;
                x0.C_b_i = Matrix3d::Identity();
                x0.C_n_b = C_n_b;
                x0.g_b_i = f_ib_b;
                x0.C_n0_i = C_n_b;
                // std::cout << x0.g_b_i << std::endl;

                // Initialize the filter
                mukf.init(x0, P0, Q, R, p_b_meas);
                initialized = true;

                // Log the initial state and covariance
                VectorXd x = mukf.get_state();
                VectorXd P = mukf.get_cov();
                log_data("[x] %s", avl::to_string(x, 9).c_str());
                log_data("[P] %s", avl::to_string(P, 9).c_str());

                log_info("-------------------------------------------");
                log_info("Gyrocompass initialized!");
                log_info("    w_ib_b:  %.4f, %.4f, %.4f rad/s", w_ib_b(0), w_ib_b(1), w_ib_b(2));
                log_info("    f_ib_b:  %.4f, %.4f, %.4f m/s^2", f_ib_b(0), f_ib_b(1), f_ib_b(2));
                log_info("    roll:    %.3f deg", avl::rad_to_deg(theta_n_b(0)));
                log_info("    pitch:   %.3f deg", avl::rad_to_deg(theta_n_b(1)));
                log_info("    yaw:     %.3f deg", avl::rad_to_deg(theta_n_b(2)));
                log_info("    lat:     %.3f deg", avl::rad_to_deg(p_b_meas(0)));
                log_info("    lon:     %.3f deg", avl::rad_to_deg(p_b_meas(1)));
                log_info("    alt:     %.3f m",   p_b_meas(2));
                log_info("-------------------------------------------");

            } // Nav not initialized

        } // GPS has lock

    } // GPS message

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        log_data("[x] a b c d e f g h i j k l");
        log_data("[x] a b c d e f g h i j k l");
        log_data("[P] a b c d e f g h i j k l");
        log_data("[P] a b c d e f g h i j k l");

        // Get config file parameters
        min_sats = get_param<int>("~min_sats");
        P0 = avl::from_std_vector(get_param<doubles_t>("~P0")).asDiagonal();
        Q =  avl::from_std_vector(get_param<doubles_t>("~Q")).asDiagonal();
        R =  avl::from_std_vector(get_param<doubles_t>("~R")).asDiagonal();

        // Set up the publishers and subscribers
        gyrocompass_pub = node_handle->advertise<GyrocompassMsg>(
            "nav/gyrocompass", 1);
        imu_sub = node_handle->subscribe("device/imu", 1,
            &GyrocompassNode::imu_msg_callback, this);
        gps_sub = node_handle->subscribe("device/gps", 1,
            &GyrocompassNode::gps_msg_callback, this);

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
    GyrocompassNode node(argc, argv);
    node.start();
    return 0;
}
