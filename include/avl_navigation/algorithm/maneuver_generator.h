//==============================================================================
// Autonomous Vehicle Library
//
// Description: Generates IMU measurements for a series of maneuvers.
//==============================================================================

#ifndef MANEUVER_GENERATOR_H
#define MANEUVER_GENERATOR_H

#include <avl_core/util/matrix.h>
#include <avl_navigation/algorithm/inertial_nav.h>

//==============================================================================
//                              STRUCT DEFINITION
//==============================================================================

// Struct containing maneuver data
typedef struct Maneuver
{

    MatrixXd w_eb_b;
    MatrixXd a_eb_b;

} Maneuver;

// Struct containing maneuver data
typedef struct ManeuverOutput
{

    int N;
    double dt;
    VectorXd t;

    MatrixXd w_ib_b;
    MatrixXd f_ib_b;

    MatrixXd theta_n_b;
    MatrixXd v_eb_n;
    MatrixXd p_b;

} ManeuverOutput;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class ManeuverGenerator
{

public:

    //--------------------------------------------------------------------------
    // Name:        ManeuverGenerator constructor
    // Description: ManeuverGenerator constructor. Sets the time step.
    // Arguments:   - dt: Time step in seconds.
    //--------------------------------------------------------------------------
    ManeuverGenerator(double dt) : dt(dt)
    {

    }

    //--------------------------------------------------------------------------
    // Name:        ManeuverGenerator destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~ManeuverGenerator()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        generate
    // Description: Generates the true IMU measurements from a maneuver.
    // Arguments:   ???
    // Returns:     Angular velocity and linear acceleration for the maneuver.
    //--------------------------------------------------------------------------
    ManeuverOutput generate(std::vector<Maneuver> maneuvers,
        Vector3d theta_n_b0, Vector3d v_eb_n0, Vector3d p_b0)
    {

        // Ensure there are maneuvers
        if (maneuvers.empty())
            throw std::runtime_error("generate: no maneuvers specified");

        // Concatenate all maneuver data
        MatrixXd w_eb_b = maneuvers.at(0).w_eb_b;
        MatrixXd a_eb_b = maneuvers.at(0).a_eb_b;
        for (size_t i = 1; i < maneuvers.size(); i++)
        {
            w_eb_b = avl::hcat(w_eb_b, maneuvers.at(i).w_eb_b);
            a_eb_b = avl::hcat(a_eb_b, maneuvers.at(i).a_eb_b);
        }

        // Total number of iterations
        int N = w_eb_b.cols();

        // Generate time vector
        VectorXd t = VectorXd::Zero(N);
        for (int i = 1; i < N; i++)
            t(i) = t(i-1) + dt;

        // Generate IMU measurements
        MatrixXd w_ib_b_out = MatrixXd::Zero(3,N);
        MatrixXd f_ib_b_out = MatrixXd::Zero(3,N);

        // TEMP: Disturbances
        Vector3d A_w_dist = {0.0, 0.4, 0.0};
        Vector3d F_w_dist = {0.2, 0.1, 0.12};
        Vector3d k_w_dist = {1.1, 0.0, 3.2};

        Vector3d A_f_dist = {0.1, 0.15, 0.2};
        Vector3d F_f_dist = {0.15, 0.13, 0.14};
        Vector3d k_f_dist = {1.0, 2.0, 3.0};

        MatrixXd w_dist = MatrixXd::Zero(3,N);
        MatrixXd f_dist = MatrixXd::Zero(3,N);
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                w_dist(j,i) = A_w_dist(j) * sin(2.0*M_PI*F_w_dist(j)*t(i) + k_w_dist(j));
                f_dist(j,i) = A_f_dist(j) * sin(2.0*M_PI*F_f_dist(j)*t(i) + k_f_dist(j));
            }
        }

        MatrixXd theta_n_b_out = MatrixXd::Zero(3,N);
        MatrixXd v_eb_n_out =    MatrixXd::Zero(3,N);
        MatrixXd p_b_out =       MatrixXd::Zero(3,N);

        // Set initial conditions
        theta_n_b_out.col(0) = theta_n_b0;
        v_eb_n_out.col(0) = v_eb_n0;
        p_b_out.col(0) = p_b0;

        for (int i = 1; i < N; i++)
        {

            // Previous nav state
            Vector3d theta_n_b = theta_n_b_out.col(i-1);
            Matrix3d C_b_n =  avl::euler_to_matrix(theta_n_b).transpose();
            Vector3d v_eb_n = v_eb_n_out.col(i-1);
            Vector3d p_b =    p_b_out.col(i-1);

            // Determine the angular rate of the ECEF frame w.r.t the ECI frame,
            // expressed in the NED frame
            Vector3d w_ie_n = earth_rotation_rate(p_b);
            Matrix3d W_ie_n = skew(w_ie_n);

            // Determine the angular rate of the NED frame w.r.t the ECEF frame,
            // expressed in the NED frame. This is called the transort rate
            Vector3d w_en_n = transport_rate(p_b, v_eb_n);
            Matrix3d W_en_n = skew(w_en_n);

            // Determine the acceleration due to gravity expressed in the NED frame
            Vector3d g_b_n = gravitational_acceleration(p_b);

            // Determine IMU measurements
            w_ib_b_out.col(i-1) = w_eb_b.col(i-1) + C_b_n.transpose()*(w_en_n + w_ie_n);
            f_ib_b_out.col(i-1) = a_eb_b.col(i-1) - C_b_n.transpose()*(g_b_n - (W_en_n + 2.0*W_ie_n)*v_eb_n);

            // // TEMP: Disturbances
            // w_ib_b_out.col(i-1) += w_dist.col(i-1);
            // f_ib_b_out.col(i-1) += f_dist.col(i-1);

            // Iterate navigation
            Vector3d w = w_ib_b_out.col(i-1);
            Vector3d f = f_ib_b_out.col(i-1);
            f_ins(C_b_n, v_eb_n, p_b, w, f, dt);

            Matrix3d C_n_b = C_b_n.transpose();
            theta_n_b_out.col(i) = avl::matrix_to_euler(C_n_b);
            v_eb_n_out.col(i) = v_eb_n;
            p_b_out.col(i) = p_b;

        }

        ManeuverOutput out;
        out.N = N;
        out.dt = dt;
        out.t = t;
        out.w_ib_b = w_ib_b_out;
        out.f_ib_b = f_ib_b_out;
        out.theta_n_b = theta_n_b_out;
        out.v_eb_n = v_eb_n_out;
        out.p_b = p_b_out;
        return out;

    }

    //--------------------------------------------------------------------------
    // Name:        coast
    // Description: Generate a coast maneuver.
    // Arguments:   ???
    // Returns:     Angular velocity and linear acceleration for the maneuver.
    //--------------------------------------------------------------------------
    Maneuver coast(double t)
    {

        // Calculate number of iterations based on time and dt
        int N = t / dt;

        // Re-calculate t based on number of iterations
        t = N*dt;

        // Generate angular velocity and linear acceleration vectors
        Maneuver m;
        m.w_eb_b = MatrixXd::Zero(3,N);
        m.a_eb_b = MatrixXd::Zero(3,N);

        return m;

    }

    //--------------------------------------------------------------------------
    // Name:        accelerate
    // Description: Generate a turn maneuver.
    // Arguments:   ???
    // Returns:     Angular velocity and linear acceleration for the maneuver.
    //--------------------------------------------------------------------------
    Maneuver accelerate(double t, double vel)
    {

        // Calculate number of iterations based on time and dt
        int N = t / dt;

        // Re-calculate t based on number of iterations
        t = N*dt;

        // Calculate angular velocity and linear acceleration for maneuver
        Vector3d w = Vector3d::Zero();
        Vector3d a = {vel/t, 0.0, 0.0};

        // Generate angular velocity and linear acceleration vectors
        Maneuver m;
        m.w_eb_b = w.replicate(1,N);
        m.a_eb_b = a.replicate(1,N);

        return m;

    }

    //--------------------------------------------------------------------------
    // Name:        turn
    // Description: Generate a turn maneuver.
    // Arguments:   ???
    // Returns:     Angular velocity and linear acceleration for the maneuver.
    //--------------------------------------------------------------------------
    Maneuver turn(double angle, double radius, double vel)
    {

        double arc_len = abs(radius*angle);
        double t = arc_len/vel;
        double dir = (angle < 0.0) ? -1.0 : 1.0;

        // Calculate number of iterations based on time and dt
        int N = t / dt;

        // Re-calculate t based on number of iterations
        t = N*dt;

        // Calculate angular velocity and linear acceleration for maneuver
        Vector3d w = {0.0, 0.0, angle/t};
        Vector3d a = {0.0, dir*vel*vel/radius, 0.0};

        // Generate angular velocity and linear acceleration vectors
        Maneuver m;
        m.w_eb_b = w.replicate(1,N);
        m.a_eb_b = a.replicate(1,N);

        return m;

    }


private:

    // Time step in sec
    double dt;

};

#endif // MANEUVER_GENERATOR_H
