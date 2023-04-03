//==============================================================================
// Autonomous Vehicle Library
//
// Description: Helper classes for navigation data parsing.
//==============================================================================

#ifndef NAV_PARSING_H
#define NAV_PARSING_H

// Util functions
#include <avl_core/util/time.h>
#include <avl_core/util/matrix.h>
#include <avl_core/util/string.h>
#include <avl_core/util/ros.h>
#include <avl_core/util/file.h>


// Inertial navigation filter
// #include <avl_navigation/algorithm/sins_err_ekf.h>
// #include <avl_navigation/algorithm/sins_ukf.h>
#include <avl_navigation/algorithm/sins_mukf_std.h>
// #include <avl_navigation/algorithm/sins_mukf_gm.h>
// #include <avl_navigation/algorithm/sins_mukf_old.h>
// #include <avl_navigation/algorithm/sins_mukf_simple.h>

// Alias for double vector
typedef std::vector<double> doubles_t;

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

// Struct containing sensor data to be loaded from a file
class SensorData
{

public:

    // Sensor name
    std::string name;

    // Marks the sensor as enabled or disabled for parsing
    bool enabled = true;

    // Matrix of measurements and vector of their timestamps
    VectorXd t;
    MatrixXd meas;

    // Measurement noise covariance matrix
    MatrixXd R;

    // Measurement lever arm
    Vector3d l_bS_b = Vector3d::Zero();

    // Rejection threshold stddev multiplier
    VectorXd threshold;

    // Measurement index for processing
    int i = 0;

public:

    //--------------------------------------------------------------------------
    // Name:        load
    // Description: Loads sensor data from a dat file according to config file
    //              settings.
    // Arguments:  - dat_folder_path: Path to the dat folder.
    //             - name: Name of sensor as per config file.
    // Returns:     Loaded sensor data struct. If the measurement is not
    //              enabled, the struct will have default values.
    //--------------------------------------------------------------------------
    void load(std::string dat_folder_path, std::string name)
    {

        this->name = name;

        // Create the filepath
        std::string dat_file_path = dat_folder_path + "/" +
            get_param<std::string>("~"+name+"/filename");

        // Do nothing if the data file doesn't exist
        if (!avl::file_exists(dat_file_path))
            return;

        // Load the dat file into a matrix
        MatrixXd dat = avl::csv_to_matrix(dat_file_path, ' ', 2);

        // Get the time vector
        t = dat.col(0);

        // Construct the measurement matrix based on the columns indicated
        // in the config file
        auto cols = get_param<std::vector<int>>("~"+name+"/cols");
        meas = MatrixXd(dat.rows(), cols.size());
        for (size_t i = 0; i < cols.size(); i++)
            meas.col(i) = dat.col(cols.at(i));
        meas.transposeInPlace();

        // Construct the measurement noise covariance matrix
        R = avl::from_std_vector(
            get_param<doubles_t>("~"+name+"/R")).asDiagonal();

        // Construct the sensor lever arm vector
        l_bS_b = avl::from_std_vector(
            get_param<doubles_t>("~"+name+"/l_bS_b")).asDiagonal();

        // Get the enabled flag from the config file
        enabled = get_param<bool>("~"+name+"/enabled");

        // Get the threshold from the config file
        threshold = avl::from_std_vector(
            get_param<doubles_t>("~"+name+"/threshold"));

    }

    //--------------------------------------------------------------------------
    // Name:        advance
    // Description: Advances the measurement index to the next measurement.
    //--------------------------------------------------------------------------
    void advance()
    {
        i++;
    }

    //--------------------------------------------------------------------------
    // Name:        has_meas
    // Description: Checks if there is a measurement at the measurement index or
    //              if it is past the end of the number of measurements.
    // Returns:     True if there is a measurement at the measurement index,
    //              false if there are no measurements or it is past the end of
    //              the number of measurements.
    //--------------------------------------------------------------------------
    bool has_meas()
    {
        return meas.cols() > 0 && i < meas.cols();
    }

    //--------------------------------------------------------------------------
    // Name:        get_prev_meas
    // Description: Gets the previous measurement.
    // Returns:     Latest measurement.
    //--------------------------------------------------------------------------
    VectorXd get_prev_meas()
    {
        return (i > 0) ? meas.col(i-1) : meas.col(0);
    }

    //--------------------------------------------------------------------------
    // Name:        get_meas
    // Description: Gets the measurement at the measurement index.
    // Returns:     Measurement at the measurement index.
    //--------------------------------------------------------------------------
    VectorXd get_meas()
    {
        if (i < meas.cols())
            return meas.col(i);
        else
            throw std::runtime_error("get_meas: reached end of " + name +  " measurements");
    }

    //--------------------------------------------------------------------------
    // Name:        get_time
    // Description: Gets the timestamp of the measurement at the measurement
    //              index.
    // Returns:     Timestamp of the measurement at the measurement index.
    //--------------------------------------------------------------------------
    double get_time()
    {
        if (i < t.size())
            return t(i);
        else
            throw std::runtime_error("get_time: reached end of " + name +  " measurements");
    }

};

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

class Vehicle
{

public:

    // Vehicle ID
    uint8_t id;

    // Aiding sensor data
    SensorData imu;
    SensorData dvl;
    SensorData depth;
    SensorData range;
    SensorData gps;
    SensorData mag;

    // Navigation filter
    NavFilter* filter;
    bool heading_initialized = false;
    bool nav_initialized = false;

    // Altitude of the water surface
    double alt_surface = 0.0;
    double b_depth = 0.0;

    // Acoustic ranging whitelist
    std::vector<int> whitelist;

public:

    //--------------------------------------------------------------------------
    // Name:        Vehicle
    // Description: Vehicle constructor.
    //--------------------------------------------------------------------------
    Vehicle(std::string log_folder_path)
    {
        // filter = new SinsMukfSimple();
        // filter = new SinsUkf();
        filter = new SinsMukfStd();
        // filter = new SinsMukfGm();
        // filter = new SinsMukfOld();
        // filter = new SinsMukfDvl();
        load(log_folder_path);
    }

    //--------------------------------------------------------------------------
    // Name:        load
    // Description: Loads sensor data from the dat files in a timestamped log
    //              folder. The log folder must already have been parsed by the
    //              log plotter to contain a dat folder.
    // Arguments:   - log_folder_path: Path to the timestamped log folder.
    //--------------------------------------------------------------------------
    void load(std::string log_folder_path)
    {

        // Determine the vehicle ID of the data being loaded by parsing the
        // comms manager node info logging of the vehicle ID
        std::string comms_log_filepath = log_folder_path +
            "/log/comms_manager_node.log";
        std::ifstream stream(comms_log_filepath);
        if (stream.is_open())
        {
            std::string line;
            std::getline(stream, line); // Read the first line (initializing node)
            std::getline(stream, line); // Read the vehicle ID line
            id = std::stoi(avl::split(line, "x").at(1), nullptr, 16);
        }
        else
        {
            std::cout << "load: unable to open "
                      << comms_log_filepath
                      << ", using 0 vehicle ID"
                      << std::endl;
            id = 0;
        }

        // Load the sensor data files
        std::string dat_folder_path = log_folder_path + "/dat";

        std::cout << "loading sensor measurements..." << std::endl;

        std::cout << "    loading IMU measurements...   ";
        imu.load(dat_folder_path, "imu");
        std::cout << "loaded " + std::to_string(imu.t.size()) + " IMU measurements" << std::endl;

        std::cout << "    loading DVL measurements...   ";
        dvl.load(dat_folder_path, "dvl");
        std::cout << "loaded " + std::to_string(dvl.t.size()) + " DVL measurements" << std::endl;

        std::cout << "    loading depth measurements... ";
        depth.load(dat_folder_path, "depth");
        std::cout << "loaded " + std::to_string(depth.t.size()) + " depth measurements" << std::endl;

        std::cout << "    loading range measurements... ";
        range.load(dat_folder_path, "range");
        std::cout << "loaded " + std::to_string(range.t.size()) + " range measurements" << std::endl;

        std::cout << "    loading GPS measurements...   ";
        gps.load(dat_folder_path, "gps");
        std::cout << "loaded " + std::to_string(gps.t.size()) + " gps measurements" << std::endl;

        std::cout << "    loading mag measurements...   ";
        mag.load(dat_folder_path, "mag");
        std::cout << "loaded " + std::to_string(mag.t.size()) + " mag measurements" << std::endl;

        std::cout << "finished loading sensor measurements" << std::endl;

        // Load the ranging whitelist
        typedef std::map<std::string, std::string> whitelist_t;
        if (check_param<whitelist_t>("~range_whitelist"))
        {

            // Read the map value for this vehicle ID
            auto list = get_param<whitelist_t>("~range_whitelist");
            std::string id_str = std::to_string(id);

            // Only parse the whitelist if there is an entry for this vehicle ID
            if (list.count(id_str) == 1)
            {

                // Split the string of whitelisted IDs so they can be parsed
                // into ints
                std::vector<std::string> ids = avl::split(list[id_str], " ");

                // Parse the whitelisted ID strings into a vector of ints
                whitelist.clear();
                for (auto val : ids)
                    whitelist.push_back(std::stoi(val));

            }

        }

    }

};

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

class AceinnaData
{

public:

    int N;
    VectorXd t;
    MatrixXd w_ib_b_true;
    MatrixXd f_ib_b_true;
    MatrixXd theta_n_b_true;
    MatrixXd v_eb_n_true;
    MatrixXd v_eb_b_true;
    MatrixXd p_b_true;
    MatrixXd x_true;
    VectorXd distance;

    void load(std::string folder_path)
    {

        // Data file paths
        std::string time_path =          folder_path + "/time.csv";
        std::string ref_gyro_path =      folder_path + "/ref_gyro.csv";
        std::string ref_accel_path =     folder_path + "/ref_accel.csv";
        std::string ref_att_euler_path = folder_path + "/ref_att_euler.csv";
        std::string ref_vel_path =       folder_path + "/ref_vel.csv";
        std::string ref_pos_path =       folder_path + "/ref_pos.csv";

        // Load matrices from file
        t =              avl::csv_to_matrix(time_path,          ',', 1);
        w_ib_b_true =    avl::csv_to_matrix(ref_gyro_path,      ',', 1).transpose();
        f_ib_b_true =    avl::csv_to_matrix(ref_accel_path,     ',', 1).transpose();
        theta_n_b_true = avl::csv_to_matrix(ref_att_euler_path, ',', 1).transpose().colwise().reverse();
        v_eb_n_true =    avl::csv_to_matrix(ref_vel_path,       ',', 1).transpose();
        p_b_true =       avl::csv_to_matrix(ref_pos_path,       ',', 1).transpose();
        N = w_ib_b_true.cols();

        // Convert degrees to radians
        w_ib_b_true             *= (M_PI / 180.0);
        theta_n_b_true          *= (M_PI / 180.0);
        p_b_true.block(0,0,2,N) *= (M_PI / 180.0);

        // Convert NED velocity to body velocity
        v_eb_b_true = MatrixXd(3, N);
        for (int i = 0; i < N; i++)
        {
            Vector3d theta_n_b = theta_n_b_true.col(i);
            Vector3d v_eb_n = v_eb_n_true.col(i);
            Matrix3d C_n_b = avl::euler_to_matrix(theta_n_b);
            v_eb_b_true.col(i) = C_n_b * v_eb_n;
        }

        // Assemble the truth state matrix
        x_true = MatrixXd(9,N);
        x_true.block(0,0,3,N) = theta_n_b_true;
        x_true.block(3,0,3,N) = v_eb_n_true;
        x_true.block(6,0,3,N) = p_b_true;

        // Calculate distance travelled
        distance = VectorXd::Zero(N);
        for (int i = 1; i < N; i++)
            distance(i) = distance(i-1) + ( p_b_true.col(i).segment(0,2) -
                                            p_b_true.col(i-1).segment(0,2) ).norm() * 6371000.0;

    }

};

#endif // NAV_PARSING_H
