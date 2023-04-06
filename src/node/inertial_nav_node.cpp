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
// Publishers:  nav/inertial_nav (avl_msgs/NavigationMsg)
//              nav/gps_residual (geometry_msgs/Vector3)
//              nav/range_residual (geometry/msgs/Vector3)
//              nav/dvl_residual (geometry/msgs/Vector3)
//              nav/depth_residual (std_msgs/Float64)
//
// Subscribers: device/imu (avl_msgs/ImuMsg)
//              device/ahrs (avl_msgs/AhrsMsg)
//              device/velocity (geometry_msgs/Vector3)
//              device/depth (std_msgs/Float64)
//              nav/range (avl_msgs/RangeMsg)
//              device/multibeam (avl_msgs/MultibeamMsg)
//              device/gps (avl_msgs/GpsMsg)
//              nav/gyrocompass (avl_msgs/GyrocompassMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/time.h>

// Inertial navigation filter
#include <avl_navigation/algorithm/sins_mukf_std.h>
#include <avl_navigation/algorithm/moving_avg.h>

// Terrain map class
#include <avl_navigation/algorithm/terrain_map.h>

// ROS message includes
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/ImuMsg.h>
#include <avl_msgs/AhrsMsg.h>
#include <avl_msgs/PathfinderDvlMsg.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <avl_msgs/MultibeamMsg.h>
#include <avl_msgs/RangeMsg.h>
#include <avl_msgs/GpsMsg.h>
#include <avl_msgs/GyrocompassMsg.h>
using namespace avl_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

// Alias for double vector
typedef std::vector<double> doubles_t;


//==============================================================================
//                              ENUM DEFINITION
//==============================================================================

// Enum listing init modes
enum InitMode
{
    MODE_GPS,
    MODE_DVL,
    MODE_MAG,
    MODE_ZERO,
    MODE_SIM,
    MODE_GYRO
};

//==============================================================================
//                              STRUCT DEFINITION
//==============================================================================

// Struct containing sensor configuration information
struct SensorConfig
{

    // Sensor is enabled and should be processed
    bool enabled = false;

    // Measurement noise covariance matrix
    MatrixXd R;

    // Measurement lever arm
    Vector3d l_bS_b = Vector3d::Zero();

    // Rejection threshold stddev multiplier
    VectorXd threshold;

};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class InertialNavNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        InertialNavNode constructor
    //--------------------------------------------------------------------------
    InertialNavNode(int argc, char **argv) : Node(argc, argv),
        avg_north_err(60), avg_east_err(60)
    {

    }

private:

    // Navigation filter pointer
    NavFilter* filter;

    // Filter initial state, its covariance, and process noise covariance
    int num_states;
    MatrixXd P0;
    MatrixXd Q;

    // Publisher for inertial navigation estimates
    bool nav_initialized = false;
    ros::Publisher nav_pub;
    ros::Publisher nav_status_pub;
    ros::Publisher gps_residual_pub;
    ros::Publisher range_residual_pub;
    ros::Publisher dvl_residual_pub;
    ros::Publisher depth_residual_pub;
    ros::Publisher nav_state_cov_pub;

    // Initialization mode from config file
    InitMode init_mode;
    double init_min_speed;

    // Subscribers for navigation data
    ros::Subscriber imu_sub;
    ros::Subscriber dvl_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber range_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber ahrs_sub;
    ros::Subscriber gc_sub;

    // Aiding sensor configurations
    SensorConfig dvl;
    SensorConfig depth;
    SensorConfig range;
    SensorConfig gps;

    // Most recent IMU measurements. Needed for some aiding sensor calculations
    Vector3d w_ib_b;
    Vector3d f_ib_b;
    Vector3d m_b;

    // Altitude of the water surface for depth measurements. Saved from GPS
    // measurement when nav is initialized
    double alt_surface;

    // Minimum number of sats for a GPS lock from config file
    int min_sats;

    // Acoustic ranging whitelist
    std::vector<int> range_whitelist;

    // DVL velocity for heading initialization from DVL and GPS
    Vector3d v_eb_b_dvl = {NAN, NAN, NAN};

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

    // Latest gyrocompass message and stddev threshold
    bool gc_msg_received = false;
    GyrocompassMsg gc_msg;
    double gc_std_threshold;

    // Moving averages for average position error
    MovingAvg avg_north_err;
    MovingAvg avg_east_err;

private:

    //--------------------------------------------------------------------------
    // Name:        load_sensor_config
    // Description: Loads a sensor config from the config file.
    // Arguments:   - name: Name of sensor as per config file.
    // Returns:     Loaded sensor config struct. If the measurement is not
    //              enabled, the struct will have default values.
    //--------------------------------------------------------------------------
    SensorConfig load_sensor_config(std::string name)
    {

        SensorConfig config;

        // Get the enabled flag from the config file
        config.enabled = get_param<bool>("~"+name+"/enabled");

        // Construct the measurement noise covariance matrix
        config.R = avl::from_std_vector(
            get_param<doubles_t>("~"+name+"/R")).asDiagonal();

        // Construct the sensor lever arm vector
        config.l_bS_b = avl::from_std_vector(
            get_param<doubles_t>("~"+name+"/l_bS_b")).asDiagonal();

        // Get the threshold from the config file
        config.threshold = avl::from_std_vector(
            get_param<doubles_t>("~"+name+"/threshold"));

        return config;

    }

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the
    //              communication architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle RESET NAV commands
        if (command_name == "RESET NAV")
        {

            init_mode = params.get("INIT MODE").to_enum<InitMode>();
            log_info("resetting navigation with init mode %d", init_mode);
            nav_initialized = false;

            // Publish an initial nav status message
            std_msgs::Bool status_msg;
            status_msg.data = nav_initialized;
            nav_status_pub.publish(status_msg);

            result = true;
            return true;

        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        gyrocompass_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void gyrocompass_msg_callback(const GyrocompassMsg& msg)
    {
        gc_msg = msg;
        gc_msg_received = true;
    }

    //--------------------------------------------------------------------------
    // Name:        imu_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void imu_msg_callback(const ImuMsg& msg)
    {

        // Turn the IMU measurements into vectors
        w_ib_b(0) = msg.angular_velocity.x;
        w_ib_b(1) = msg.angular_velocity.y;
        w_ib_b(2) = msg.angular_velocity.z;

        f_ib_b(0) = msg.linear_acceleration.x;
        f_ib_b(1) = msg.linear_acceleration.y;
        f_ib_b(2) = msg.linear_acceleration.z;

        // Don't iterate if the filter is not initialized or data is invalid
        if (!nav_initialized || !msg.valid)
            return;

        // Iterate the filter using the IMU data
        filter->iterate(w_ib_b, f_ib_b, msg.dt);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Publish and log the filter state if it is valid
        if(filter->valid())
        {

            VectorXd x = filter->get_state();
            VectorXd P = filter->get_cov();

            // Create and send a navigation message
            NavigationMsg nav_msg;
            nav_msg.roll =  avl::wrap_to_pi(x(0));
            nav_msg.pitch = avl::wrap_to_pi(x(1));
            nav_msg.yaw =   avl::wrap_to_2pi(x(2));
            nav_msg.vn =    x(3);
            nav_msg.ve =    x(4);
            nav_msg.vd =    x(5);
            nav_msg.lat =   x(6);
            nav_msg.lon =   x(7);
            nav_msg.alt =   x(8);
            nav_msg.yaw_std = sqrt(P(2,2));

            Vector2d avg_pos_err = {avg_north_err.get_average(), avg_east_err.get_average()};
            nav_msg.avg_pos_err = avg_pos_err.norm();
            nav_pub.publish(nav_msg);

            // Create and send navigation state covariances
            Float64MultiArray nav_state_cov_msg;
            nav_state_cov_msg.data.resize(P.rows());
            for(int i = 0; i < P.rows(); i++) {
                nav_state_cov_msg.data[i] = P(i, i);
            }
            nav_state_cov_pub.publish(nav_state_cov_msg);

            VectorXd y(6);
            y << w_ib_b, f_ib_b;
            log_data("[imu] %s", avl::to_string(y, 9).c_str());
            log_data("[x] %s", avl::to_string(x, 9).c_str());
            log_data("[P] %s", avl::to_string(P, 9).c_str());

        }

    }

    //--------------------------------------------------------------------------
    // Name:        dvl_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - msg: message received on the topic
    //--------------------------------------------------------------------------
    void dvl_msg_callback(const PathfinderDvlMsg& msg)
    {

        // Only handle valid DVL measurements
        if (msg.valid)
        {

            // Format the DVL velocity
            v_eb_b_dvl = {msg.vx, msg.vy, msg.vz};

            // Process the DVL measurement if nav is initialized
            if (dvl.enabled && nav_initialized && v_eb_b_dvl.allFinite() &&
                w_ib_b.allFinite())
            {

                MeasInfo info = filter->process_body_velocity(
                    v_eb_b_dvl,
                    w_ib_b,
                    dvl.R,
                    dvl.threshold,
                    dvl.l_bS_b);

                log_data("[dvl] %s", info.to_string().c_str());

                // Get residual from info
                Vector3d dvl_inno = info.innovation.segment(0,3);

                geometry_msgs::Vector3 dvl_res_msg;
                dvl_res_msg.x = dvl_inno(0);
                dvl_res_msg.y = dvl_inno(1);
                dvl_res_msg.z = dvl_inno(2);
                dvl_residual_pub.publish(dvl_res_msg);
            }

        }
        else
        {
            v_eb_b_dvl = {NAN, NAN, NAN};
        }

    }

    //--------------------------------------------------------------------------
    // Name:        depth_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void depth_msg_callback(const std_msgs::Float64& msg)
    {

        VectorXd y(1);
        y << msg.data;

        if (depth.enabled && nav_initialized && y.allFinite())
        {

            MeasInfo info = filter->process_depth(
                y,
                alt_surface,
                depth.R,
                depth.threshold,
                depth.l_bS_b);

            log_data("[depth] %s", info.to_string().c_str());

            // Get depth residual from info
            double depth_inno = info.innovation(0);
            std_msgs::Float64 depth_res_msg;
            depth_res_msg.data = depth_inno;
            depth_residual_pub.publish(depth_res_msg);          

        }

    }

    //--------------------------------------------------------------------------
    // Name:        range_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - msg: message received on the topic
    //--------------------------------------------------------------------------
    void range_msg_callback(const RangeMsg& msg)
    {

        // Range measurement elements
        int src_id =          msg.src_id;
        Vector3d p_source = { msg.lat, msg.lon, msg.alt };
        VectorXd y(1);
        y << msg.range;

        // Process the range measurement
        if (range.enabled && nav_initialized && y.allFinite() &&
            (range_whitelist.empty() ||
             avl::has_element(range_whitelist, src_id)))
        {

            MeasInfo info = filter->process_range(
                y,
                p_source,
                range.R,
                range.threshold,
                range.l_bS_b);

            // Get range residual from info
            Vector3d range_inno = info.innovation.segment(0,3);
            geometry_msgs::Vector3 range_res_msg;
            range_res_msg.x = range_inno(0);
            range_res_msg.y = range_inno(1);
            range_res_msg.z = range_inno(2);
            range_residual_pub.publish(range_res_msg);  
            // Log range information
            VectorXd x = filter->get_state();
            log_data("[range] %d %s %s %s",
                src_id,
                avl::to_string(p_source, 9).c_str(),
                avl::to_string(x.segment(6,3), 9).c_str(),
                info.to_string().c_str());

        }

    }

    //--------------------------------------------------------------------------
    // Name:        gps_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - msg: message received on the topic
    //--------------------------------------------------------------------------
    void gps_msg_callback(const GpsMsg& msg)
    {

        // GPS measurement elements - GPS track angle/vel log format
        Vector3d p_b_gps =    {msg.lat, msg.lon, msg.alt};
        double ground_speed = msg.ground_speed;
        double track_angle =  msg.track_angle;

        // Calculate GPS velocity from ground speed and track angle
        Vector3d v_eb_n_gps = {ground_speed*cos(track_angle),
                               ground_speed*sin(track_angle),
                               0.0};

        // Only process the GPS measurement if it has a lock
        if (p_b_gps.allFinite() && msg.num_sats >= min_sats)
        {

            if (gps.enabled && nav_initialized)
            {

                // Process GPS position and velocity
                VectorXd gps_meas(5);
                gps_meas << p_b_gps, v_eb_n_gps(0), v_eb_n_gps(1);
                MeasInfo info = filter->process_gps(
                    gps_meas,
                    w_ib_b,
                    gps.R,
                    gps.threshold,
                    gps.l_bS_b);

                log_data("[gps] %d %s", msg.num_sats,
                    info.to_string().c_str());

                // Update average position errors
                Matrix3d T_p_rn = curvilinear_to_cartesian(p_b_gps);
                Vector3d gps_inno = T_p_rn*info.innovation.segment(0,3);
                avg_north_err.add_sample(gps_inno(0));
                avg_east_err.add_sample(gps_inno(1));

                // Residual from gps
                geometry_msgs::Vector3 gps_res_msg;
                gps_res_msg.x = gps_inno(0);
                gps_res_msg.y = gps_inno(1);
                gps_res_msg.z = gps_inno(2);
                gps_residual_pub.publish(gps_res_msg);  

            }

            // Initialize the navigation if it is not already
            else if (!nav_initialized)
            {

                // Fill the initial state vector
                VectorXd x0 = VectorXd::Zero(num_states);

                // Set initial attitude from IMU. Heading will be changed later
                Vector3d theta_n_b = attitude_from_imu(w_ib_b, f_ib_b);
                x0.segment(0,3) = theta_n_b;

                // Set initial velocity and position from GPS
                x0.segment(3,3) = v_eb_n_gps;
                x0.segment(6,3) = p_b_gps;

                // Save the surface altitude
                alt_surface = p_b_gps(2);

                // Attempt to initialize heading using the chosen method
                bool heading_initialized = false;

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Initialize with GPS heading
                if (init_mode == MODE_GPS)
                {
                    if (v_eb_n_gps.norm() >= init_min_speed)
                    {
                        x0(2) = msg.track_angle;
                        heading_initialized = true;
                    }
                }

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Initialize with GPS+DVL heading
                else if (init_mode == MODE_DVL)
                {

                    log_data("[init] %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                        v_eb_b_dvl(0), v_eb_b_dvl(1), v_eb_b_dvl(2),
                        v_eb_n_gps(0), v_eb_n_gps(1), v_eb_n_gps(2),
                        (v_eb_b_dvl - v_eb_n_gps).norm());

                    // Check that the vehicle ismoving and that the GPS and DVL
                    // velocities have similar magnitudes
                    double v_dvl_mag = v_eb_b_dvl.segment(0,2).norm();
                    double v_gps_mag = v_eb_n_gps.segment(0,2).norm();
                    double v_diff = abs(v_dvl_mag - v_gps_mag);
                    if (v_eb_b_dvl.allFinite() &&
                        v_eb_n_gps.allFinite() &&
                        v_dvl_mag >= init_min_speed &&
                        v_diff <= 0.1)
                    {

                        // Calculate heading from DVL and GPS
                        double vx = v_eb_b_dvl(0);
                        double vy = v_eb_b_dvl(1);
                        double vN = v_eb_n_gps(0);
                        double vE = v_eb_n_gps(1);
                        x0(2) = atan2(vx*vE-vy*vN, vx*vN+vy*vE);
                        heading_initialized = true;

                        log_info("===========================================================");
                        log_info("DVL mode initialization");
                        log_info("    v_gps_N:  %.3f m/s", v_eb_n_gps(0));
                        log_info("    v_gps_E:  %.3f m/s", v_eb_n_gps(1));
                        log_info("    v_dvl_x:  %.3f m/s", v_eb_b_dvl(0));
                        log_info("    v_dvl_y:  %.3f m/s", v_eb_b_dvl(1));
                        log_info("    v_diff:   %.3f m/s", v_diff);
                        log_info("    yaw:      %.3f deg", avl::rad_to_deg(x0(2)));
                        log_info("===========================================================");

                    }

                }

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Initialize with magnetometer heading
                else if (init_mode == MODE_MAG)
                {
                    x0.segment(0,3) = attitude_from_imu_mag(f_ib_b, m_b);
                    heading_initialized = true;
                }

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Initialize with simulation ground truth
                else if (init_mode == MODE_SIM)
                {

                    // Set heading from dynamic manager node initial attitude
                    std::vector<double> initial_att =
                        get_param<std::vector<double>>(
                            "dynamics_manager_node/initial_att");
                    x0(2) = avl::deg_to_rad(initial_att.at(2));

                    // Update initial heading stddev to 0.1 degrees
                    P0(2,2) = 3.0462e-06;

                    heading_initialized = true;

                }

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Initialize with gyrocompass
                else if (init_mode == MODE_GYRO)
                {

                    // Wait until the gyrocompass yaw stddev is below threshold
                    if (gc_msg_received && gc_msg.yaw_std < gc_std_threshold)
                    {

                        // Set initial attitude states
                        x0(0) = gc_msg.roll;
                        x0(1) = gc_msg.pitch;
                        x0(2) = gc_msg.yaw;

                        // Set initial attitude covariances
                        P0(0,0) = gc_msg.roll_std*gc_msg.roll_std;
                        P0(1,1) = gc_msg.pitch_std*gc_msg.pitch_std;
                        P0(2,2) = gc_msg.yaw_std*gc_msg.yaw_std;

                        heading_initialized = true;

                    }

                }

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Initialize to zero heading
                else
                {
                    heading_initialized = true;
                }

                // If heading has been initialized, all states have been
                // initialized and the filter can be started
                if (heading_initialized)
                {

                    // Initialize the filter
                    filter->init(x0, P0, Q);
                    nav_initialized = true;

                    // Log the initial state and covariance
                    VectorXd x = filter->get_state();
                    VectorXd P = filter->get_cov();
                    log_data("[x] %s", avl::to_string(x, 9).c_str());
                    log_data("[P] %s", avl::to_string(P, 9).c_str());

                    log_info("-------------------------------------------");
                    log_info("Nav filter initialized!");
                    log_info("    mode:  %d", init_mode);
                    log_info("    roll:  %.3f deg", avl::rad_to_deg(x0(0)));
                    log_info("    pitch: %.3f deg", avl::rad_to_deg(x0(1)));
                    log_info("    yaw:   %.3f deg", avl::rad_to_deg(x0(2)));
                    log_info("    vN:    %.3f m/s", x0(3));
                    log_info("    vE:    %.3f m/s", x0(4));
                    log_info("    vD:    %.3f m/s", x0(5));
                    log_info("    lat:   %.3f deg", avl::rad_to_deg(x0(6)));
                    log_info("    lon:   %.3f deg", avl::rad_to_deg(x0(7)));
                    log_info("    alt:   %.3f m",   x0(8));
                    log_info("-------------------------------------------");

                    // Publish a nav status message
                    std_msgs::Bool status_msg;
                    status_msg.data = nav_initialized;
                    nav_status_pub.publish(status_msg);

                } // Heading initialized

            } // Nav not initialized

        } // GPS has lock

    } // GPS message

    //--------------------------------------------------------------------------
    // Name:        ahrs_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - msg: message received on the topic
    //--------------------------------------------------------------------------
    void ahrs_msg_callback(const AhrsMsg& msg)
    {
        m_b = {msg.mag.x, msg.mag.y, msg.mag.z};
        log_data("[mag] %.5f %.5f %.5f", m_b(0), m_b(1), m_b(2));
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Instantiate the nav filter
        filter = new SinsMukfStd();
        init_mode = static_cast<InitMode>(get_param<int>("~init_mode"));
        init_min_speed = get_param<double>("~init_min_speed");

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Add data log headers
        add_data_header("[imu] \\omega_{ib,x}^b \\omega_{ib,y}^b \\omega_{ib,z}^b f_{ib,x}^b f_{ib,y}^b f_{ib,z}^b");
        add_data_header("[imu] rad/s rad/s rad/s m/s^2 m/s^2 m/s^2");

        add_data_header("[gps] lat lon alt sats has\\_lock");
        add_data_header("[gps] rad rad m sats bool");

        add_data_header("[mag] mx my mz");
        add_data_header("[mag] Gauss Gauss Gauss");

        add_data_header("[range] src\\_id range\\_meas lat lon alt source\\_lat source\\_lon source\\_alt "
            "range\\_est innovation pos\\_stddev threshold accepted");
        add_data_header("[range] ID m rad rad m rad rad m m m m NA bool");

        add_data_header("[height] lat lon alt alt\\_terrain height\\_meas height\\_est");
        add_data_header("[height] rad rad m m m m");

        add_data_header("[x] roll pitch yaw v_x v_y v_z lat lon alt bgx bgy bgz bax bay baz");
        add_data_header("[x] rad rad rad m/s m/s m/s rad rad m rad/s rad/s rad/s m/s^2 m/s^2 m/s^2");

        add_data_header("[P] cov_roll cov_pitch cov_yaw cov_v_x cov_v_y cov_v_z cov_lat"
            " cov_lon cov_alt cov_bgx cov_bgy cov_bgz cov_bax cov_bay cov_baz");
        add_data_header("[P] rad^2 rad^2 rad^2 (m/s)^2 (m/s)^2 (m/s)^2 rad^2 rad^2 m^2 "
            "(rad/s)^2 (rad/s)^2 (rad/s)^2 (m/s^2)^2 (m/s^2)^2 (m/s^2)^2");

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Construct the initial staet and process noise covariance matrices
        // from the config file
        P0 = avl::from_std_vector(get_param<doubles_t>("~P0")).asDiagonal();
        Q = avl::from_std_vector(get_param<doubles_t>("~Q")).asDiagonal();
        num_states = P0.rows();

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Get config file settings
        if (check_param<std::vector<int>>("~range_whitelist"))
            range_whitelist = get_param<std::vector<int>>("~range_whitelist");
        min_sats = get_param<int>("~min_sats");
        gc_std_threshold = get_param<double>("~gc_std_threshold");

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Load sensor configs from config file
        dvl =   load_sensor_config("dvl");
        depth = load_sensor_config("depth");
        range = load_sensor_config("range");
        gps =   load_sensor_config("gps");

        // Set up the publishers and subscribers
        nav_pub =        node_handle->advertise<NavigationMsg>("nav/inertial_nav", 1);
        gps_residual_pub = node_handle->advertise<Vector3>("nav/gps_residual", 1);
        range_residual_pub = node_handle->advertise<Vector3>("nav/range_residual", 1);
        dvl_residual_pub = node_handle->advertise<Vector3>("nav/dvl_residual", 1);
        depth_residual_pub = node_handle->advertise<Float64>("nav/depth_residual", 1);
        nav_state_cov_pub = node_handle->advertise<Float64MultiArray>("nav/state_cov", 1);

        nav_status_pub = node_handle->advertise<std_msgs::Bool>("nav/status", 1, true);
        imu_sub =   node_handle->subscribe("device/imu",      1,  &InertialNavNode::imu_msg_callback, this);
        dvl_sub =   node_handle->subscribe("device/dvl",      1,  &InertialNavNode::dvl_msg_callback, this);
        depth_sub = node_handle->subscribe("device/depth",    1,  &InertialNavNode::depth_msg_callback, this);
        range_sub = node_handle->subscribe("nav/range",       10, &InertialNavNode::range_msg_callback, this);
        gps_sub =   node_handle->subscribe("device/gps",      1,  &InertialNavNode::gps_msg_callback, this);
        ahrs_sub =  node_handle->subscribe("device/ahrs",     1,  &InertialNavNode::ahrs_msg_callback, this);
        gc_sub =    node_handle->subscribe("nav/gyrocompass", 1,  &InertialNavNode::gyrocompass_msg_callback, this);

        // Set the command handler's callback
        command_handler.set_callback(&InertialNavNode::command_callback, this);

        // Publish an initial nav status message
        std_msgs::Bool status_msg;
        status_msg.data = nav_initialized;
        nav_status_pub.publish(status_msg);

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
    InertialNavNode node(argc, argv);
    node.start();
    return 0;
}
