//==============================================================================
// Autonomous Vehicle Library
//
// Description: Listens for micro heartbeat packets received over the acoustic
//              channel and translates them into range measurements.
//
// Servers:     None
//
// Clients:     device/whoi/downlink_rx (avl_devices/WhoiDataMsg)
//
// Publishers:  nav/range (avl_navigation/RangeMsg)
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/vector.h>
#include <avl_core/util/string.h>
#include <avl_core/util/math.h>

// ROS messages
#include <avl_msgs/WhoiDataMsg.h>
#include <avl_msgs/RangeMsg.h>
using namespace avl_msgs;

// AVL Packets
#include <avl_core/protocol/avl.h>
using namespace avl;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class AcousticRangingNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        AcousticRangingNode constructor
    //--------------------------------------------------------------------------
    AcousticRangingNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Subscriber for WHOI Micromodem data
    ros::Subscriber whoi_rx_sub;

    // Publisher for range messages
    ros::Publisher range_pub;

    // If set to true, OWTT range calculations will use only the digits after
    // the decimal point in the message time of arrival to calculate range. If
    // false, the entire timestamp will be used (requires that the computer and
    // WHOI micromodem are both time-synced to GPS). Setting from config file
    bool use_fractional_seconds;

    // Speed of sound in m/s from the config file
    double speed_of_sound;

private:

    //--------------------------------------------------------------------------
    // Name:        whoi_rx_msg_callback
    // Description: Called when a WHOI data message is received.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void whoi_data_msg_callback(const WhoiDataMsg& message)
    {

        // Look for micro heartbeat packets and parse them
        if (message.data.at(0) == MICRO_HEARTBEAT_PACKET)
        {

            log_debug("received micro heartbeat packet");

            // Construct a micro heartbeat packet from the bytes
            HeartbeatPacket packet =
                MicroHeartbeatPacket::to_heartbeat_packet(message.data);
            PacketHeader header = packet.get_header();
            int source_id = static_cast<int>(header.source_id);
            Heartbeat heartbeat = packet.get_heartbeat();

            // Get the time of departure and and time of arrival
            double time_of_departure = message.time_of_departure;
            double time_of_arrival =   message.time_of_arrival;

            // Calculate the time of flight based on whether we're using
            // only the fractional part of the time of arrival or the
            // full timestamp
            double time_of_flight;
            if (use_fractional_seconds)
            {

                // Separate the time of arrival into an integer part and
                // a fractional part
                double fract_part, int_part;
                fract_part = modf(time_of_arrival, &int_part);

                // The total time of flight is the fractional part of
                // the time of arrival since the message was transmitted
                // exactly on the second. This is valid for times of flight
                // less than 1 second
                time_of_flight = fract_part;

            }
            else
            {
                time_of_flight = time_of_arrival - time_of_departure;
            }

            // Calculate range from speed of sound and time of flight
            double range = time_of_flight * speed_of_sound;

            // Format and publish a range message
            RangeMsg range_msg;
            range_msg.src_id = source_id;
            range_msg.lat = avl::deg_to_rad(heartbeat.nav_lat);
            range_msg.lon = avl::deg_to_rad(heartbeat.nav_lon);
            range_msg.alt = heartbeat.nav_alt;
            range_msg.range = range;
            range_pub.publish(range_msg);

            log_data("[range] %d %.9f %.9f %.2f %.6f %.6f %.6f %.4f",
                source_id, range_msg.lat, range_msg.lon, range_msg.alt,
                time_of_departure, time_of_arrival, time_of_flight,
                range_msg.range);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log the data headers
        add_data_header("[range] src_id lat lon alt tod toa tof range");
        add_data_header("[range] id deg deg m sec sec sec m");

        // Get the config file parameters
        speed_of_sound = get_param<double>("~speed_of_sound");
        use_fractional_seconds = get_param<bool>("~use_fractional_seconds");

        // Set up the WHOI rx subscriber
        whoi_rx_sub = node_handle->subscribe("device/whoi/downlink_rx", 1,
            &AcousticRangingNode::whoi_data_msg_callback, this);

        // Set up the range publisher
        range_pub = node_handle->advertise<RangeMsg>("nav/range", 1);

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

// Name and initialize the node
int main(int argc, char **argv)
{
    AcousticRangingNode node(argc, argv);
    node.start();
    return 0;
}
