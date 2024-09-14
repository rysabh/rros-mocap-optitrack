#ifndef MOCAPPUBLISHER_H
#define MOCAPPUBLISHER_H

#include <vector>
#include <NatNetTypes.h>

#include "rclcpp/rclcpp.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include "mocap_optitrack_interfaces/msg/marker_set_array.hpp"
#include "mocap_optitrack_interfaces/msg/motion_capture_data.hpp"


using namespace std;
using namespace std::chrono;

class MoCapPublisher: public rclcpp::Node
{
private:
    //Attributes
    rclcpp::Publisher<mocap_optitrack_interfaces::msg::MotionCaptureData>::SharedPtr publisher_motion;

    // rclcpp::Publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr publisher_;
    // rclcpp::Publisher<mocap_optitrack_interfaces::msg::MarkerSetArray>::SharedPtr publisher_set;
    rclcpp::TimerBase::SharedPtr timer_;
    high_resolution_clock::time_point t_start;

    //Methods
    void sendFakeMessage();
    

public:
    // Definition of the construtors
    MoCapPublisher();

    // Send methods
    void sendCombinedMessage(double cameraMidExposureSecsSinceEpoch, sMarker* markers_ptr, int nMarkers, sRigidBodyData* bodies_ptr, int nRigidBodies);
    void sendRigidBodyMessage(double cameraMidExposureSecsSinceEpoch, sRigidBodyData* bodies_ptr, int nRigidBodies);
    void sendMarkerSetMessage(double cameraMidExposureSecsSinceEpoch, sMarker* markers_ptr, int nMarkers);
    
    mocap_optitrack_interfaces::msg::MarkerSet createMarkerSetMessage(
        double cameraMidExposureSecsSinceEpoch, sMarker LabeledMarker);

    mocap_optitrack_interfaces::msg::RigidBody createRigidBodyMessage(
        double cameraMidExposureSecsSinceEpoch, sRigidBodyData bodies_ptr);

 
    // Getters
    std::string getServerAddress();
    int getConnectionType();
    std::string getMulticastAddress();
    uint16_t getServerCommandPort();
    uint16_t getServerDataPort();
    bool isRecordingRequested();
    std::string getTakeName();
    
    // Setters

};
 
#endif