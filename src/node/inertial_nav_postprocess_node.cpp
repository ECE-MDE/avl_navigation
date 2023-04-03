//==============================================================================
// Autonomous Vehicle Library
//
// Description: Post-processes navigation sensor data from one or more
//              timestamped log folders. Log folders must have been previously
//              parsed by the AVL log plotter to include a dat folder.
//
//              This parser will handle the overriding of source position for
//              inter-vehicle range measurements if both vehicles are being
//              parsed.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Nav data parsing classes
#include <avl_navigation/algorithm/nav_parsing.h>

//==============================================================================
//                              ENUM DEFINITION
//==============================================================================

// Enum listing init modes
enum InitMode
{
    MODE_GPS,
    MODE_DVL,
    MODE_MAG,
    MODE_ZERO
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class InertialNavPostprocessNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        InertialNavPostprocessNode constructor
    //--------------------------------------------------------------------------
    InertialNavPostprocessNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Vector of vehicles
    std::vector<Vehicle> vehicles;

    // Filter parameters
    int num_states;
    MatrixXd P0;
    MatrixXd Q;

    // Time bounds for looping through all data
    double t_start;
    double t_end;

    // GPS lock settings from config file
    int min_sats;

    // GPS depth cutoff from config file
    double gps_depth_cutoff;

    // Initialization mode from config file
    InitMode init_mode;
    double init_min_speed;

    // Current depth
    double curr_depth = 0.0;
    double alpha = 0.1;

private:

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Create a Vehicle class instance for each log folder path
        auto log_folder_paths =
            get_param<std::vector<std::string>>("~log_folder_paths");
        for (auto path : log_folder_paths)
        {
            vehicles.emplace_back(Vehicle(path));
            log_info("loaded data for vehicle %d (%s)",
                vehicles.back().id, path.c_str());
        }

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Construct the initial staet and process noise covariance matrices
        // from the config file
        P0 = avl::from_std_vector(get_param<doubles_t>("~P0")).asDiagonal();
        Q = avl::from_std_vector(get_param<doubles_t>("~Q")).asDiagonal();
        num_states = P0.rows();

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Determine the start and end timestamps by finding the minimum and
        // maximum timestamps in the vehicle's data
        log_info("===========================================================");
        log_info("determining start and end timestamps...");
        doubles_t timestamps;
        for (auto v : vehicles)
        {
            timestamps.push_back(v.imu.t(0));
            timestamps.push_back(v.imu.t(v.imu.t.size()-1));
        }
        t_start = *std::min_element(timestamps.begin(), timestamps.end());
        t_end =   *std::max_element(timestamps.begin(), timestamps.end());
        log_info("start:    %.6f s", t_start);
        log_info("end:      %.6f s", t_end);
        log_info("duration: %.2f s", t_end-t_start);
        log_info("===========================================================");

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Get settings from the config file
        min_sats = get_param<int>("~min_sats");
        gps_depth_cutoff = get_param<double>("~gps_depth_cutoff");
        init_mode = static_cast<InitMode>(get_param<int>("~init_mode"));
        init_min_speed = get_param<double>("~init_min_speed");

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {

        // Get the parsing start time so that parsing duration can be calculated
        double t_parse_start = avl::get_epoch_time();
        log_info("parsing IMU measurements and aiding sensor data...");

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Main processing loop. Step between the start and end timestamps
        t_end = t_start + 1800;
        for (double t = t_start; t < t_end; t += 0.001)
        {

            // Stop parsing if ROS is shut down
            if (!ros::ok())
                break;

            // For each vehicle, check for measurements to be processed
            for (Vehicle& v : vehicles)
            {

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check for IMU measurement
                if (v.imu.has_meas() && v.imu.get_time() <= t)
                {

                    if (v.nav_initialized)
                    {

                        // IMU measurement elements
                        VectorXd y = v.imu.get_meas();
                        bool valid = true;//static_cast<bool>(y(6));
                        Vector3d w_ib_b = y.segment(0,3);
                        Vector3d f_ib_b = y.segment(3,3);

                        if (valid && y.allFinite())
                            v.filter->iterate(w_ib_b, f_ib_b, 0.01);

                        // Log the iteration data
                        VectorXd x = v.filter->get_state();
                        VectorXd P = v.filter->get_cov();

                        log_data("[imu] %d %.9f %s",
                            v.id, v.imu.get_time(),
                            avl::to_string(y, 9).c_str());
                        log_data("[x] %d %.9f %s",
                            v.id, v.imu.get_time(),
                            avl::to_string(x, 9).c_str());
                        log_data("[P] %d %.9f %s",
                            v.id, v.imu.get_time(),
                            avl::to_string(P, 9).c_str());

                    }

                    // Advance to the next measurement
                    v.imu.advance();

                } // IMU measurement

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check for GPS measurement
                if (v.gps.has_meas() && v.gps.get_time() <= t)
                {

                    // =========================================================
                    // GPS measurement elements - GPS track angle/vel log format
                    VectorXd gps_data = v.gps.get_meas();
                    Vector3d p_b_gps =    gps_data.segment(0,3);
                    double ground_speed = gps_data(3);
                    double track_angle =  gps_data(4);
                    int num_sats =        gps_data(5);

                    // Calculate GPS velocity from ground speed and track angle
                    Vector3d v_eb_n_gps = {ground_speed*cos(track_angle),
                                           ground_speed*sin(track_angle),
                                           0.0};

                    // // =========================================================
                    // // GPS measurement elements - Old GPS velocity log format
                    // VectorXd gps_data = v.gps.get_meas();
                    // Vector3d p_b_gps = gps_data.segment(0,3);
                    // double vN =        gps_data(3);
                    // double vE =        gps_data(4);
                    // int num_sats =     gps_data(5);
                    //
                    // // Calculate GPS velocity from ground speed and track angle
                    // Vector3d v_eb_n_gps = {vN, vE, 0.0};
                    // double track_angle = atan2(vE, vN);
                    // // =========================================================

                    // IMU measurement for GPS lever arm correction
                    VectorXd y_imu = v.imu.get_meas();
                    Vector3d w_ib_b = y_imu.segment(0,3);
                    Vector3d f_ib_b = y_imu.segment(3,3);

                    // Determine if GPS has a valid lock
                    bool sufficient_sats = num_sats >= min_sats;
                    bool depth_cutoff_disabled = gps_depth_cutoff == -1;
                    bool not_underwater = curr_depth < gps_depth_cutoff;
                    bool gps_has_lock =
                        p_b_gps.allFinite() &&
                        sufficient_sats &&
                        (not_underwater || depth_cutoff_disabled);

                    // Only process the GPS measurement if it has a lock
                    if (gps_has_lock)
                    {

                        // If nav is initialized, pass the GPS measurement to
                        // the navigation filter
                        if (v.gps.enabled && v.nav_initialized)
                        {

                            // Process GPS position and velocity
                            VectorXd y(5);
                            y << p_b_gps, v_eb_n_gps(0), v_eb_n_gps(1);
                            MeasInfo info = v.filter->process_gps(
                                y,
                                w_ib_b,
                                v.gps.R,
                                v.gps.threshold,
                                v.gps.l_bS_b);

                            log_data("[gps] %d %.9f %d %s",
                                v.id, v.gps.get_time(), num_sats,
                                info.to_string().c_str());

                            // Update depth bias with exponential filter
                            double b_depth_new = v.b_depth + curr_depth;
                            v.b_depth = alpha*b_depth_new + (1-alpha)*v.b_depth;
                            log_data("[alt_surface] %.5f %.5f", curr_depth, v.b_depth);

                        } // GPS enabled and nav initialized

                        // If navigation is not yet initialized, attempt to
                        else if (!v.nav_initialized)
                        {

                            // Fill the initial state vector
                            VectorXd x0 = VectorXd::Zero(num_states);

                            // Set initial attitude from IMU. Heading will be
                            // changed later depending in init mode
                            Vector3d theta_n_b = attitude_from_imu(w_ib_b,
                                                                   f_ib_b);
                            x0.segment(0,3) = theta_n_b;

                            // Set initial velocity and position from GPS
                            x0.segment(3,2) = v_eb_n_gps;
                            x0.segment(6,3) = p_b_gps;

                            // Temp b_gz
                            // x0(14) = 0.075;

                            // Save the surface altitude
                            v.alt_surface = p_b_gps(2);

                            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            // Initialize with GPS heading
                            if (init_mode == MODE_GPS)
                            {
                                if (v_eb_n_gps.norm() >= init_min_speed)
                                {
                                    x0(2) = atan2(v_eb_n_gps(1), v_eb_n_gps(0));
                                    v.heading_initialized = true;
                                }
                            }

                            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            // Initialize with GPS+DVL heading
                            else if (init_mode == MODE_DVL)
                            {

                                // Check that the vehicle ismoving and that the
                                // GPS and DVL velocity have similar magnitudes
                                Vector3d v_eb_b_dvl = v.dvl.get_meas();
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
                                    v.heading_initialized = true;

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

                            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            // Initialize with magnetometer heading
                            else if (init_mode == MODE_MAG)
                            {
                                Vector3d m_b = v.mag.get_meas();
                                x0.segment(0,3) = attitude_from_imu_mag(f_ib_b,
                                                                        m_b);
                                v.heading_initialized = true;
                            }

                            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            // Initialize to zero heading
                            else
                            {
                                v.heading_initialized = true;
                            }

                            // If heading has been initialized, all states have
                            // been initialized and the filter can be started
                            if (v.heading_initialized)
                            {

                                // Initialize the filter
                                v.filter->init(x0, P0, Q);
                                v.nav_initialized = true;

                                // Log the initial state and covariance
                                VectorXd x = v.filter->get_state();
                                VectorXd P = v.filter->get_cov();
                                log_data("[imu] %d %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
                                    v.id, v.imu.get_time(),
                                    0,0,0,0,0,0);
                                log_data("[x] %d %.9f %s",
                                    v.id, v.gps.get_time(),
                                    avl::to_string(x, 15).c_str());
                                log_data("[P] %d %.9f %s",
                                    v.id, v.gps.get_time(),
                                    avl::to_string(P, 15).c_str());

                                log_info("===========================================================");
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
                                log_info("===========================================================");

                            } // Heading initialized

                        } // Nav not initialized

                    } // GPS has lock

                    // Advance to the next measurement
                    v.gps.advance();

                } // GPS measurement

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check for range measurement
                if (v.range.has_meas() && v.range.get_time() <= t)
                {

                    // Range measurement elements
                    VectorXd range_data = v.range.get_meas();
                    int src_id =          range_data(0);
                    Vector3d p_source =   range_data.segment(1,3);
                    VectorXd y =          range_data.segment(4,1);

                    // If the range measurement source is another vehicle that's
                    // being parsed, replace the source position that was
                    // reported in the data with the source vehicle's current
                    // position estimate from its filter
                    for (Vehicle& src : vehicles)
                        if (src.id == src_id)
                            p_source = src.filter->get_state().segment(6,3);

                    // Process the range measurement
                    if (v.range.enabled && v.nav_initialized && y.allFinite() &&
                        (v.whitelist.empty() ||
                         avl::has_element(v.whitelist, src_id)))
                    {

                        MeasInfo info = v.filter->process_range(
                            y,
                            p_source,
                            v.range.R,
                            v.range.threshold,
                            v.range.l_bS_b);

                        // Log range information
                        VectorXd x = v.filter->get_state();
                        log_data("[range] %d %.9e %d %s %s %s",
                            v.id,
                            v.range.get_time(),
                            src_id,
                            avl::to_string(p_source, 9).c_str(),
                            avl::to_string(x.segment(6,3), 9).c_str(),
                            info.to_string().c_str());

                    }

                    // Increment the measurement index
                    v.range.advance();

                } // Range measurement

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check for depth measurement
                if (v.depth.has_meas() && v.depth.get_time() <= t)
                {

                    VectorXd y = v.depth.get_meas();
                    y(0) -= v.b_depth;
                    curr_depth = y(0);
                    if (v.depth.enabled && v.nav_initialized && y.allFinite())
                    {
                        MeasInfo info = v.filter->process_depth(
                            y,
                            v.alt_surface,
                            v.depth.R,
                            v.depth.threshold,
                            v.depth.l_bS_b);

                        log_data("[depth] %d %.9f %s",
                            v.id,
                            v.depth.get_time(),
                            info.to_string().c_str());

                    }

                    // Advance to the next measurement
                    v.depth.advance();

                } // Depth measurement

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check for DVL measurement
                if (v.dvl.has_meas() && v.dvl.get_time() <= t)
                {

                    VectorXd dvl_data = v.dvl.get_meas();
                    Vector3d v_eb_b = dvl_data.segment(0,3);
                    double v_err = dvl_data(3);
                    bool data_good = dvl_data(4);
                    // bool data_good = true;
                    // double v_err = 0.0;
                    Vector3d w_ib_b = v.imu.get_prev_meas().segment(0,3);

                    if (v.dvl.enabled && v.nav_initialized && v_eb_b.allFinite() &&
                        w_ib_b.allFinite() && data_good && v_err < 0.1)
                    {
                        MeasInfo info = v.filter->process_body_velocity(
                            v_eb_b,
                            w_ib_b,
                            v.dvl.R,
                            v.dvl.threshold,
                            v.dvl.l_bS_b);

                        log_data("[dvl] %d %.9f %s",
                            v.id,
                            v.dvl.get_time(),
                            info.to_string().c_str());

                    }

                    // Advance to the next measurement
                    v.dvl.advance();

                } // DVL measurement

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check for mag measurement
                if (v.mag.has_meas() && v.mag.get_time() <= t)
                {

                    VectorXd y = v.mag.get_meas();
                    if (v.mag.enabled && v.nav_initialized && y.allFinite())
                    {

                        MeasInfo info = v.filter->process_mag_flux(
                            y,
                            v.mag.R,
                            v.mag.threshold,
                            v.mag.l_bS_b);

                        log_data("[mag] %d %.9f %s",
                            v.id,
                            v.mag.get_time(),
                            info.to_string().c_str());

                    }

                    // Advance to the next measurement
                    v.mag.advance();

                } // Mag measurement

            } // For each vehicle

        } // Main processing loop

        // Calculate total parsing duration
        double t_parse_end = avl::get_epoch_time();
        log_info("parsing completed, took %.4f seconds",
            t_parse_end - t_parse_start);

    } // run

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    InertialNavPostprocessNode node(argc, argv);
    node.start();
    return 0;
}
