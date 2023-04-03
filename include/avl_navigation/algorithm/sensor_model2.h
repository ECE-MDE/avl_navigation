//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a basic sensor model with sensor errors for a sensor
//              with an arbitrary number of outputs. Sensor model includes
//              Gaussian sensor noise, axis misalignment, and bias errors.
//==============================================================================

#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

// GM process
#include <avl_navigation/algorithm/gm_process.h>

// Random number generation
#include <random>
#include <chrono>
using namespace std::chrono;

// Eigen matrices and vectors
#include <Eigen/Core>
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Alias for double vector
typedef std::vector<double> doubles_t;

//==============================================================================
//                              STRUCT DEFINITION
//==============================================================================

// Struct containing info about the sensor error
typedef struct SensorConfig2
{

    double dt;
    VectorXd noise_cov;
    VectorXd bias_cov;
    VectorXd bias_time_const;

} SensorConfig2;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class SensorModel2
{

public:

    //--------------------------------------------------------------------------
    // Name:        from_config_file
    // Description: Constructs a sensor model from config file parameters.
    // Arguments:   - name: Name of parameter containing sensor error settings.
    // Returns:     Sensor model configured from the config file settings.
    //--------------------------------------------------------------------------
    static SensorModel2 from_config_file(std::string name)
    {

        // Get the config file parameters
        double dt =                      avl::get_param<double>("~" + name + "/dt");
        doubles_t noise_cov_vect =       avl::get_param<doubles_t>("~" + name + "/noise_cov");
        doubles_t bias_cov_vect =        avl::get_param<doubles_t>("~" + name + "/bias_cov");
        doubles_t bias_time_const_vect = avl::get_param<doubles_t>("~" + name + "/bias_time_const");

        size_t num_axes = noise_cov_vect.size();

        // Verify input sizes
        if (bias_cov_vect.size() != num_axes ||
            bias_time_const_vect.size() != num_axes)
            throw std::runtime_error("from_config_file: inputs must have same number of elements");

        // Construct the config struct
        SensorConfig2 config;
        config.dt = dt;
        config.noise_cov =       Eigen::Map<VectorXd>(noise_cov_vect.data(), num_axes);
        config.bias_cov =        Eigen::Map<VectorXd>(bias_cov_vect.data(), num_axes);
        config.bias_time_const = Eigen::Map<VectorXd>(bias_time_const_vect.data(), num_axes);

        return SensorModel2(config);

    }

public:


    //--------------------------------------------------------------------------
    // Name:        SensorModel2 constructor
    // Description: SensorModel2 constructor.
    //--------------------------------------------------------------------------
    SensorModel2()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        SensorModel2 constructor
    // Description: SensorModel2 constructor. Sets the number of sensor axes.
    // Arguments:   - config: Sensor error model configuration struct.
    //--------------------------------------------------------------------------
    SensorModel2(SensorConfig2 config)
    {

        this->config = config;
        num_axes = config.noise_cov.size();

        // Verify input sizes
        if (config.bias_cov.size() != num_axes ||
            config.bias_time_const.size() != num_axes)
            throw std::runtime_error("SensorModel: inputs must have same number of elements");

        // Construct sensor noise normal distribution for the sensor axis
        noise_dists.clear();
        for (int i = 0; i < num_axes; i++)
        {
            auto noise_dist = std::normal_distribution<double>(
                0.0, sqrt(config.noise_cov(i)));
            noise_dists.push_back(noise_dist);

        }

        // Generate a seed for the random number generator from the current time
        generator.seed(system_clock::now().time_since_epoch().count());

    }

    //--------------------------------------------------------------------------
    // Name:        SensorModel2 destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~SensorModel2()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        generate_measurements
    // Description: Adds the sensor error model terms to the true sensor
    //              measurement value, inclusing noise.
    // Arguments:   - x: Vector of true sensor values to add error to.
    // Returns:     Vector with sensor errors added.
    //--------------------------------------------------------------------------
    MatrixXd generate_measurements(MatrixXd x_true, MatrixXd& b)
    {

        // Number of measurements
        int N = x_true.cols();

        // Verify input size
        if (x_true.rows() != num_axes)
            throw std::runtime_error("generate_measurements: input rows must "
                "match sensor config");

        // Generate bias and sensor noise for each axis
        b = MatrixXd::Zero(num_axes, N);
        MatrixXd eta(num_axes, N);

        for (int i = 0; i < num_axes; i++)
        {

            // Generate GM process for bias
            GmProcess gm(config.bias_cov(i), config.bias_time_const(i), config.dt);
            b.row(i) = gm.generate(1, N);

            // Generate sensor noise
            for (int j = 0; j < N; j++)
                eta(i,j) = noise_dists.at(i)(generator);

        }

        return x_true + b + eta;

    }

    //--------------------------------------------------------------------------
    // Name:        generate_measurements
    // Description: Adds the sensor error model terms to the true sensor
    //              measurement value, inclusing noise.
    // Arguments:   - x: Vector of true sensor values to add error to.
    // Returns:     Vector with sensor errors added.
    //--------------------------------------------------------------------------
    MatrixXd generate_measurements(MatrixXd x_true)
    {
        MatrixXd b;
        return generate_measurements(x_true, b);
    }

    //--------------------------------------------------------------------------
    // Name:        get_sensor_noise_cov
    // Description: Gets the sensor model's sensor noise covariance matrix.
    // Returns:     The sensor model's sensor noise covariance matrix.
    //--------------------------------------------------------------------------
    MatrixXd get_sensor_noise_cov()
    {
        return config.noise_cov.asDiagonal();
    }

    //--------------------------------------------------------------------------
    // Name:        get_config
    // Description: Gets the sensor model's configuration.
    // Returns:     The sensor model's configuration.
    //--------------------------------------------------------------------------
    SensorConfig2 get_config()
    {
        return config;
    }

private:

    // Sensor error configuration
    SensorConfig2 config;

    // Number of sensor axes
    int num_axes;

    // Random number generator
    // see http://www.cplusplus.com/reference/random/
    std::default_random_engine generator;

    // Vector containing the noise distribution for each axis
    std::vector<std::normal_distribution<double>> noise_dists;

    // GM process for bias for each axis
    // std::vector<GmProcess> gm_processes;

};

#endif // SENSOR_MODEL_H
