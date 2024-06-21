// Copyright 2022 AI Racing Tech
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "razor_imu_ros2/razor_imu_node.hpp"

#include <algorithm>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace razor_imu_ros2 {
std::vector<uint8_t> str_to_bytes(const std::string &s) { return std::vector<uint8_t>(s.begin(), s.end()); }

void RazorImuNode::command(const std::string &command, const double &val, const uint32_t &delay_ms = 1000) {
    // RCLCPP_INFO(get_logger(), command + std::to_string(val));
    RCLCPP_INFO(get_logger(), "%s", (command + std::to_string(val)).c_str());
    driver_->port()->async_send(str_to_bytes(command + std::to_string(val) + '\r'));
    sleep(delay_ms / 1000.0);
    std::cout << "Command sent successfully" << std::endl;
}

void RazorImuNode::command(const std::string &command, const uint32_t &delay_ms = 1000) {
    RCLCPP_INFO(get_logger(), "%s", (command).c_str());
    driver_->port()->async_send(str_to_bytes(command + '\r'));
    // driver_->port()->send(str_to_bytes(command));
    sleep(delay_ms / 1000.0);
}

// Unfortunately conversion to Vector3 is not availble in Foxy
// TODO(haoru): use tf2::toMsg in Galactic onwards
geometry_msgs::msg::Vector3 toMsg(const tf2::Vector3 &in) {
    geometry_msgs::msg::Vector3 out;
    out.x = in.getX();
    out.y = in.getY();
    out.z = in.getZ();
    return out;
}

// ROS2 Foxy TF2 just doesn't work 99% of the time
// TODO(haoru): use tf2::fromMsg in Galactic onwards
void fromMsg(const geometry_msgs::msg::Vector3 &in, tf2::Vector3 &out) { out = tf2::Vector3(in.x, in.y, in.z); }

void RazorImuNode::sigintHandler(int sig) {
    (void)sig;
    RCLCPP_INFO(rclcpp::get_logger("razor_imu_node"), "Shutting down");
    rclcpp::shutdown();
    exit(0);
}

RazorImuNode::RazorImuNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("razor_imu_node", options),
      owned_ctx{new drivers::common::IoContext(2)},
      driver_{new drivers::serial_driver::SerialDriver(*owned_ctx)}

{
    // Handle SIGINT
    signal(SIGINT, RazorImuNode::sigintHandler);

    // Create pub
    m_imu_pub_ = create_publisher<Imu>("imu", rclcpp::QoS{10});

    m_imu_.header.frame_id = declare_parameter("frame_id", "imu");

    const auto avc = declare_parameter("angular_velocity_covariance", std::vector<double>{});

    std::copy(avc.begin(), avc.end(), m_imu_.angular_velocity_covariance.begin());
    const auto oc = declare_parameter("orientation_covariance", std::vector<double>{});
    std::copy(oc.begin(), oc.end(), m_imu_.orientation_covariance.begin());

    this->declare_parameter<bool>("enable_offset", true);
    m_enable_offset_ = this->get_parameter("enable_offset").as_bool();

    double rpy_offset[3]{0.0, 0.0, 0.0};
    if (m_enable_offset_) {
        static constexpr double DEG2RAD = M_PI / 180.0;

        this->declare_parameter<double>("roll_offset_deg", 0.0);
        // rpy_offset[0] = declare_parameter("roll_offset_deg").get<double>() *
        // DEG2RAD;
        rpy_offset[0] = this->get_parameter("roll_offset_deg").as_double() * DEG2RAD;

        this->declare_parameter<double>("pitch_offset_deg", 0.0);
        // rpy_offset[1] = declare_parameter("pitch_offset_deg").get<double>() *
        // DEG2RAD;
        rpy_offset[1] = this->get_parameter("pitch_offset_deg").as_double() * DEG2RAD;

        this->declare_parameter<double>("yaw_offset_deg", 0.0);
        // rpy_offset[2] = declare_parameter("yaw_offset_deg").get<double>() *
        // DEG2RAD;
        rpy_offset[2] = this->get_parameter("yaw_offset_deg").as_double() * DEG2RAD;

        m_q_offset_.setRPY(rpy_offset[0], rpy_offset[1], rpy_offset[2]);
    }
    this->declare_parameter<bool>("zero_gravity", false);
    // m_zero_gravity_ = declare_parameter("zero_gravity").get<bool>();
    m_zero_gravity_ = this->get_parameter("zero_gravity").as_bool();

    // Open serial port
    this->declare_parameter<std::string>("serial_port", "/dev/sensors/razor");
    const std::string serial_port = this->get_parameter("serial_port").as_string();

    this->declare_parameter<int>("baud_rate", 57600);
    // const uint32_t baud_rate = declare_parameter("baud_rate").get<uint32_t>();
    const uint32_t baud_rate = this->get_parameter("baud_rate").as_int();

    const auto fc = drivers::serial_driver::FlowControl::NONE;
    const auto pt = drivers::serial_driver::Parity::NONE;
    const auto sb = drivers::serial_driver::StopBits::ONE;
    try {
        driver_->init_port(serial_port, drivers::serial_driver::SerialPortConfig(baud_rate, fc, pt, sb));
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Could not open serial port: %s", e.what());
        throw;
    }

    driver_->port()->open();
    if (!driver_->port()->is_open()) {
        throw std::runtime_error("Could not open serial port: " + serial_port);
    }

    // Configure IMU
    // Stop output

    this->command("#o0");
    // Set output mode RPYAG
    this->command("#ox");

    // this->command("#f");

    this->declare_parameter<bool>("send_calibration", false);
    if (this->get_parameter("send_calibration").as_bool()) {
        // Set accel calibrations

        this->declare_parameter<double>("accel_x_min", -256.0);
        this->declare_parameter<double>("accel_x_max", 256.0);
        this->declare_parameter<double>("accel_y_min", -256.0);
        this->declare_parameter<double>("accel_y_max", 256.0);
        this->declare_parameter<double>("accel_z_min", -256.0);
        this->declare_parameter<double>("accel_z_max", 256.0);
        command("#caxm", this->get_parameter("accel_x_min").as_double());
        command("#caxM", this->get_parameter("accel_x_max").as_double());
        command("#caym", this->get_parameter("accel_y_min").as_double());
        command("#cayM", this->get_parameter("accel_y_max").as_double());
        command("#cazm", this->get_parameter("accel_z_min").as_double());
        command("#cazM", this->get_parameter("accel_z_max").as_double());

        // Set manetometer calibrations
        this->declare_parameter<bool>("calibration_magn_use_extended", false);

        if (this->get_parameter("calibration_magn_use_extended").as_bool()) {
            this->declare_parameter<std::vector<double>>("magn_ellipsoid_center", std::vector<double>{});
            const auto magn_ellipsoid_center = this->get_parameter("magn_ellipsoid_center").as_double_array();

            command("#ccx", magn_ellipsoid_center[0]);
            command("#ccy", magn_ellipsoid_center[1]);
            command("#ccz", magn_ellipsoid_center[2]);
            const auto magn_ellipsoid_transform = declare_parameter("magn_ellipsoid_transform", std::vector<double>{});
            command("#ctxX", magn_ellipsoid_transform[0]);
            command("#ctxY", magn_ellipsoid_transform[1]);
            command("#ctxZ", magn_ellipsoid_transform[2]);
            command("#ctyX", magn_ellipsoid_transform[3]);
            command("#ctyY", magn_ellipsoid_transform[4]);
            command("#ctyZ", magn_ellipsoid_transform[5]);
            command("#ctzX", magn_ellipsoid_transform[6]);
            command("#ctzY", magn_ellipsoid_transform[7]);
            command("#ctzZ", magn_ellipsoid_transform[8]);
        } else {
            this->declare_parameter<double>("magn_x_min", -600.0);
            this->declare_parameter<double>("magn_x_max", 600.0);
            this->declare_parameter<double>("magn_y_min", -600.0);
            this->declare_parameter<double>("magn_y_max", 600.0);
            this->declare_parameter<double>("magn_z_min", -600.0);
            this->declare_parameter<double>("magn_z_max", 600.0);
            command("#cmxm", this->get_parameter("magn_x_min").as_double());
            command("#cmxM", this->get_parameter("magn_x_max").as_double());
            command("#cmym", this->get_parameter("magn_y_min").as_double());
            command("#cmyM", this->get_parameter("magn_y_max").as_double());
            command("#cmzm", this->get_parameter("magn_z_min").as_double());
            command("#cmzM", this->get_parameter("magn_z_max").as_double());
        }

        // Set gyro calibrations
        this->declare_parameter<double>("gyro_average_offset_x", 0.0);
        this->declare_parameter<double>("gyro_average_offset_y", 0.0);
        this->declare_parameter<double>("gyro_average_offset_z", 0.0);
        command("#cgx", get_parameter("gyro_average_offset_x").as_double());
        command("#cgy", get_parameter("gyro_average_offset_y").as_double());
        command("#cgz", get_parameter("gyro_average_offset_z").as_double());
    }
    // Start outputting
    command("#o1");

    RCLCPP_INFO(get_logger(), "Start receiving");
    m_loop_thread_ = std::make_unique<std::thread>([this] { this->loop_thread(); });
}

void RazorImuNode::loop_thread() {
    std::vector<uint8_t> buff{};
    buff.reserve(128);
    uint8_t bad_lines = 0;
    std::vector<uint8_t> buff_temp{0};
    static const std::string header = "#YPRAG=";
    static constexpr double DEG2RAD = M_PI / 180.0;
    // sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
    static constexpr double ACCEL_FACTOR = 9.807 / 256.0;
    while (rclcpp::ok()) {
        driver_->port()->receive(buff_temp);
        buff.insert(buff.end(), buff_temp.begin(), buff_temp.end());
        if ('\n' == buff.back()) {
            std::string line = std::string(buff.begin(), buff.end());
            if (!std::regex_search(line, std::regex("(#YPRAG=)([+-]?\\d*(\\.)\\d*(,))"
                                                    "{8}[+-]?\\d*(\\.)\\d*\\r\\n"))) {
                ++bad_lines;
                static constexpr uint8_t MAX_BAD_LINES = 10;
                if (bad_lines > MAX_BAD_LINES) {
                    throw std::runtime_error("Too many bad receives.");
                }
                buff.clear();
                continue;
            } else {
                bad_lines = 0;
            }
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream ss(line.substr(header.length()));
            Imu msg = m_imu_;
            msg.header.stamp = now();

            double yrp[3] = {0.0, 0.0, 0.0};
            ss >> yrp[0];
            ss >> yrp[1];
            ss >> yrp[2];
            tf2::Quaternion q;
            q.setRPY(yrp[1] * DEG2RAD, (180.0 + yrp[2]) * DEG2RAD, -yrp[0] * DEG2RAD);
            if (m_enable_offset_) {
                q = (m_q_offset_ * q).normalize();
            }
            msg.orientation = tf2::toMsg(q);

            ss >> msg.linear_acceleration.y;
            ss >> msg.linear_acceleration.x;
            ss >> msg.linear_acceleration.z;
            msg.linear_acceleration.x *= ACCEL_FACTOR;
            msg.linear_acceleration.y *= ACCEL_FACTOR * -1.0;
            msg.linear_acceleration.z *= ACCEL_FACTOR;
            tf2::Vector3 v_l;
            razor_imu_ros2::fromMsg(msg.linear_acceleration, v_l);

            ss >> msg.angular_velocity.y;
            ss >> msg.angular_velocity.x;
            ss >> msg.angular_velocity.z;
            msg.angular_velocity.y *= -1.0;
            msg.angular_velocity.z *= -1.0;
            tf2::Vector3 v_a;
            razor_imu_ros2::fromMsg(msg.angular_velocity, v_a);
            if (m_enable_offset_) {
                msg.linear_acceleration = razor_imu_ros2::toMsg(tf2::quatRotate(m_q_offset_, v_l));
                msg.angular_velocity = razor_imu_ros2::toMsg(tf2::quatRotate(m_q_offset_, v_a));
            }

            if (m_zero_gravity_) {
                static constexpr double GRAVITY = 9.807;
                msg.linear_acceleration.z += GRAVITY;
            }
            m_imu_pub_->publish(msg);

            buff.clear();
        }
    }
    driver_->port()->close();
}
}  // namespace razor_imu_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(razor_imu_ros2::RazorImuNode)
