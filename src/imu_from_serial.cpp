#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial/serial.h" // Make sure to install the 'serial' package

using std::placeholders::_1;

class SerialToRosNode : public rclcpp::Node
{
public:
    SerialToRosNode() : Node("serial_to_ros_node")
    {
        // Initialize serial port
        serial_port_.setPort("/dev/ttyACM0"); // Change port as required
        serial_port_.setBaudrate(115200); // Change baudrate as required
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port_.setTimeout(timeout);
        serial_port_.open();

        // Publisher for the ROS topic
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

        // Timer to read from serial port and publish data
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialToRosNode::serial_to_ros, this));
    }

private:
    void serial_to_ros()
    {
        if (serial_port_.available() > 0)
        {
            // Read data from serial port
            std::string serial_data = serial_port_.readline();

            // Process the received data
            // Assuming serial_data contains IMU data in a specific format, parse it accordingly
            sensor_msgs::msg::Imu imu_msg = parse_imu_data(serial_data);

            // Publish the IMU data to ROS topic
            publisher_->publish(imu_msg);

            // Log the published data
            RCLCPP_INFO(this->get_logger(), "Published IMU data");
        }
    }

    sensor_msgs::msg::Imu parse_imu_data(const std::string &serial_data)
    {
        // Assuming serial_data contains IMU data in a specific format, parse it accordingly
        // Fill the IMU message with the parsed data
        sensor_msgs::msg::Imu imu_msg;

        // Set the fields of the IMU message (e.g., orientation, angular velocity, linear acceleration)
        // For demonstration, set some example values

        istringstream iss(serial_data);
        float qi;
        float qj;
        float qk;
        float qr;
        float qa;
        getline(iss, qi, "\t");
        getline(iss, qj, "\t");
        getline(iss, qk, "\t");
        getline(iss, ql, "\t");
        getline(iss, qa, "\t");
        float la_x;
        float la_y;
        float la_z;
        uint8_t la_acc;
        getline(iss, la_x, "\t");
        getline(iss, la_y, "\t");
        getline(iss, la_z, "\t");
        getline(iss, la_acc, "\t");
        float g_x;
        float g_y;
        float g_z;
        getline(iss, gx, "\t");
        getline(iss, gy, "\t");
        getline(iss, gz, "\t");

        std::string rest;
        getline(iss, rest);            

        std::cout << rest;

        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;

        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;

        return imu_msg;
    }

    serial::Serial serial_port_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialToRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
