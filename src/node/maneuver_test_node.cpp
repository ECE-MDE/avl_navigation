//==============================================================================
// Autonomous Vehicle Library
//
// Description: Test.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: dNone
//==============================================================================

// Node base class
#include <avl_core/node.h>

#include <avl_navigation/algorithm/maneuver_generator.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ManeuverTestNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ManeuverTestNode constructor
    //--------------------------------------------------------------------------
    ManeuverTestNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:



private:

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {

        Vector3d theta_n_b0 = {0.0, 0.0, avl::deg_to_rad(0.0)};
        Vector3d v_eb_n0 = {0.0, 0.0, 0.0};
        Vector3d p_b0 = {0.646783695325038, -1.40698626818740, 0.0};

        ManeuverGenerator gen(0.01);
        std::vector<Maneuver> lawnmower =
        {
            gen.accelerate(30.0, 1.5),
            gen.coast(200.0),
            gen.turn(M_PI/2.0, 15.0, 1.5),
            gen.coast(13.333),
            gen.turn(M_PI/2.0, 15.0, 1.5),
            gen.coast(200.0),
            gen.turn(-M_PI/2.0, 15.0, 1.5),
            gen.coast(13.333),
            gen.turn(-M_PI/2.0, 15.0, 1.5),
            gen.coast(200.0)
        };

        std::vector<Maneuver> accel =
        {
            gen.accelerate(30.0, 1.5),
            gen.coast(200.0),
        };

        std::vector<Maneuver> nomove = { gen.coast(1000.0) };
        std::vector<Maneuver> test =
        {
            gen.accelerate(30.0, 1.5),
            gen.turn(M_PI*2.0, 15.0, 1.5)
        };

        ManeuverOutput out = gen.generate(lawnmower, theta_n_b0, v_eb_n0, p_b0);

        int N = out.t.size();

        add_data_header("[t] t");
        add_data_header("[t] sec");
        add_data_header("[imu] wx wy wz fx fy fz");
        add_data_header("[imu] rad/s rad/s rad/s m/s^2 m/s^2 m/s^2");
        add_data_header("[x] roll pitch yaw vN vE vD lat lon alt");
        add_data_header("[x] rad rad rad m/s m/s m/s rad rad m");

        for (int i = 0; i < N; i++)
        {

            log_data("[t] %.4f", out.t(i));

            log_data("[imu] %.9f %.9f %.9f %.9f %.9f %.9f",
                out.w_ib_b.col(i)(0),
                out.w_ib_b.col(i)(1),
                out.w_ib_b.col(i)(2),
                out.f_ib_b.col(i)(0),
                out.f_ib_b.col(i)(1),
                out.f_ib_b.col(i)(2));

            log_data("[x] %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
                out.theta_n_b.col(i)(0),
                out.theta_n_b.col(i)(1),
                out.theta_n_b.col(i)(2),
                out.v_eb_n.col(i)(0),
                out.v_eb_n.col(i)(1),
                out.v_eb_n.col(i)(2),
                out.p_b.col(i)(0),
                out.p_b.col(i)(1),
                out.p_b.col(i)(2));

        }

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    ManeuverTestNode node(argc, argv);
    node.start();
    return 0;
}
