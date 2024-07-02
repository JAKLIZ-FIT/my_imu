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
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&SerialToRosNode::serial_to_ros, this));
    }

private:
    void serial_to_ros()
    {
        bool serial_available = false;
        try{
            serial_available = serial_port_.available();
        }
        catch(const serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial IO Exception: %s", e.what());
            while (1)
            {
                // Attempt to close and reopen the connection
                try {
                    serial_port_.close();
                    serial_port_.open();
                    RCLCPP_INFO(this->get_logger(), "Reconnected to IMU");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to reconnect: %s", e.what());
                    rclcpp::sleep_for(std::chrono::milliseconds(500));// Wait before retrying
                    continue;
                }
                break;
            }
        }
        
        if (serial_available > 0)
        {
            this->inactive_counter = 0;
            // Read data from serial port
            std::string serial_data;
            try{
                 serial_data = serial_port_.readline();
            }
            catch(const serial::IOException& e){
                //RCLCPP_INFO(this->get_logger(), "Serial connection broken");
                //serial_data= "broken serial connection";
                RCLCPP_ERROR(this->get_logger(), "Serial IO Exception: %s", e.what());
                while (1)
                {
                    // Attempt to close and reopen the connection
                    try {
                        serial_port_.close();
                        serial_port_.open();
                        RCLCPP_INFO(this->get_logger(), "Reconnected to IMU");
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to reconnect: %s", e.what());
                        // Wait before retrying
                        rclcpp::sleep_for(std::chrono::milliseconds(500));
                        continue;
                    }
                    break;                        
                }
                return;
            }

            if (isalpha(serial_data[0])){ 
                RCLCPP_INFO(this->get_logger(), "No-data IMU message");
                std::cout << serial_data << std::endl;
                return; 
            }

            // Process the received data
            // Assuming serial_data contains IMU data in a specific format, parse it accordingly
            sensor_msgs::msg::Imu imu_msg;
            
            bool imu_msg_valid = parse_imu_data(imu_msg, serial_data);

            // Publish the IMU data to ROS topic
            publisher_->publish(imu_msg);

            // Log the published data
            RCLCPP_INFO(this->get_logger(), "Published IMU data");
        }
        else{
            this->inactive_counter++;
            if (this->inactive_counter > 1000){
                RCLCPP_INFO(this->get_logger(), "No IMU data received in last 5 seconds");
                this->inactive_counter = 0;
            }
        }
    }

    bool parse_imu_data(sensor_msgs::msg::Imu &imu_msg, const std::string &serial_data)
    {
        // Assuming serial_data contains IMU data in a specific format, parse it accordingly
        // Fill the IMU message with the parsed data
        //sensor_msgs::msg::Imu imu_msg;

        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link"; // Use the specific IMU link here

        // Set the fields of the IMU message (e.g., orientation, angular velocity, linear acceleration)
        // For demonstration, set some example values

        std::vector<double> values;
        std::stringstream ss(serial_data);
        //int i = 0;
        double value;
        while (ss >> value){
            
            values.push_back(value);

            char nextchar = ss.peek();
            if (nextchar == '\t'){
                ss.ignore();
            }
            if (nextchar == ','){
                ss.ignore();
            }
            if (nextchar == '\n'){
                break;
            }
        }

        if (values.size() != 14){ // 5 values for quarternion, 4 values for lin accel, 3 for gyro, 2 debug
            RCLCPP_INFO(this->get_logger(), "Invalid input IMU data");
            std::cout << serial_data << std::endl;
            return false; // imu_msg_valid = false
        }

        // Set the fields of the IMU message (e.g., orientation, angular velocity, linear acceleration)
        // For demonstration, set some example values

        imu_msg.orientation.x = values[0];
        imu_msg.orientation.y = values[1];
        imu_msg.orientation.z = values[2];
        imu_msg.orientation.w = values[3];
        // values[4] // == quarternion error in radians
        imu_msg.linear_acceleration.x = values[5];
        imu_msg.linear_acceleration.y = values[6];
        imu_msg.linear_acceleration.z = values[7];
        // values[8] // == linear acceleration accuracy
        imu_msg.angular_velocity.x = values[9];
        imu_msg.angular_velocity.y = values[10];
        imu_msg.angular_velocity.z = values[11];
        //std::cout << "Quarternion error=" << values[4] << "\n"; // TODO create a Debug parameter
        return true; // imu_msg_valid = true
    }


    int inactive_counter = 0;
    serial::Serial serial_port_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<SerialToRosNode>();
        rclcpp::spin(node);
    }
    catch(const serial::IOException& e)
    {
        std::cerr << "CRASHED IN MAIN\n";
        std::cerr << e.what() << '\n';
    }
    
    rclcpp::shutdown();
    return 0;
}
