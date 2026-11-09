#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "tf_lisense_pkg/serial_sender.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class OdomListener : public rclcpp::Node
{
public:
    OdomListener() : Node("odom_listener_node")
    {
        m_serial_device = this->declare_parameter<std::string>("serial_device", "/dev/ttyCH340");
        m_serial_baudrate = this->declare_parameter<int>("serial_baudrate", 115200);
        m_reconnect_interval_ms = this->declare_parameter<int>("serial_reconnect_interval_ms", 500);
        if (m_reconnect_interval_ms <= 0)
        {
            RCLCPP_WARN(this->get_logger(),
                "serial_reconnect_interval_ms must be positive; using 500 ms.");
            m_reconnect_interval_ms = 500;
        }

        // 低通滤波系数（0~1），值越大平滑性越好，但滞后越大
        m_alpha = this->declare_parameter<double>("filter_alpha", 0.3);
        m_tag_timeout_ms = this->declare_parameter<int>("tag_timeout_ms", 500);
        m_odom_timeout_ms = this->declare_parameter<int>("odom_timeout_ms", 500);
        const auto cmd_vel_topic = this->declare_parameter<std::string>(
            "cmd_vel_topic", "/cmd_vel_nav");

        connectSerialPort();
        m_reconnect_timer = this->create_wall_timer(
            std::chrono::milliseconds(m_reconnect_interval_ms),
            std::bind(&OdomListener::reconnectSerialPort, this));

        m_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio/imu/odom", 10,
            std::bind(&OdomListener::callback, this, std::placeholders::_1));

        m_tag_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tag_pose_compensated", 10,
            std::bind(&OdomListener::tagPoseCallback, this, std::placeholders::_1));
        m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&OdomListener::cmdVelCallback, this, std::placeholders::_1));

        //   m_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        //        "/fastlio2/lio_odom", 10,
        //        std::bind(&OdomListener::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Subscribing to /lio/imu/odom, /tag_pose_compensated and %s; serial frame: 22 bytes",
            cmd_vel_topic.c_str());
    }

private:
    bool connectSerialPort()
    {
        if (m_serial.isOpen())
        {
            return true;
        }

        if (!m_serial.open(m_serial_device, m_serial_baudrate))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Failed to open serial port %s @ %d; retrying every %d ms.",
                m_serial_device.c_str(), m_serial_baudrate, m_reconnect_interval_ms);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port %s @ %d opened successfully.",
            m_serial_device.c_str(), m_serial_baudrate);
        return true;
    }

    void reconnectSerialPort()
    {
        if (!m_serial.isOpen())
        {
            connectSerialPort();
        }
    }

    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        const auto &v = msg->twist.twist.linear;
        const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        if (msg->header.frame_id.empty() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.z) || !std::isfinite(v.x) || !std::isfinite(v.y) ||
            !std::isfinite(v.z) || !std::isfinite(norm) || norm < 1e-9)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Ignoring invalid odometry or empty odometry frame");
            return;
        }
        if (m_odom_frame != msg->header.frame_id)
        {
            m_first_received = false;
            m_odom_frame = msg->header.frame_id;
        }
        
        // 四元数 -> 偏航角（绕 Z 轴，弧度）
        const double yaw = quaternionToYaw(q.w, q.x, q.y, q.z);

        if(printIndex_ >= printThrehold_) {
            RCLCPP_INFO(this->get_logger(),
            "raw pos=(%.3f, %.3f, %.3f) | yaw=%.2f deg | vel=(%.3f, %.3f, %.3f)",
            p.x, p.y, p.z, yaw * 180.0 / kPi, v.x, v.y, v.z);
        }

        // 一阶低通滤波
        if (!m_first_received)
        {
            // 首次数据直接初始化
            m_filtered_x = p.x;
            m_filtered_y = p.y;
            m_filtered_z = p.z;
            m_filtered_yaw = yaw;
            m_first_received = true;
        }
        else
        {
            m_filtered_x = m_alpha * m_filtered_x + (1.0 - m_alpha) * p.x;
            m_filtered_y = m_alpha * m_filtered_y + (1.0 - m_alpha) * p.y;
            m_filtered_z = m_alpha * m_filtered_z + (1.0 - m_alpha) * p.z;
            m_filtered_yaw = m_alpha * m_filtered_yaw + (1.0 - m_alpha) * yaw;
        }

        if (printIndex_ >= printThrehold_)
        {
            RCLCPP_INFO(this->get_logger(),
                "filtered pos=(%.3f, %.3f, %.3f) | yaw=%.2f deg | DWB cmd=(%.3f, %.3f, %.3f) m/s,m/s,rad/s",
                m_filtered_x, m_filtered_y, m_filtered_z, m_filtered_yaw * 180.0 / kPi,
                m_dwb_vx, m_dwb_vy, m_dwb_wz);
            printIndex_ = 0;
        }
        m_serial_words = {
            toMmInt16(m_filtered_x), toMmInt16(m_filtered_y),
            toMmInt16(m_filtered_z), toMradInt16(m_filtered_yaw),
            toMmPerSecondInt16(v.x), toMmPerSecondInt16(v.y), toMmPerSecondInt16(v.z),
            toMmPerSecondInt16(m_dwb_vx), toMmPerSecondInt16(m_dwb_vy),
            toMradInt16(m_dwb_wz)};
        m_last_odom_time = std::chrono::steady_clock::now();
        sendCombinedFrame();
        printIndex_++;
    }

    void sendCombinedFrame()
    {
        const auto now = std::chrono::steady_clock::now();
        if (!m_first_received || now - m_last_odom_time >
            std::chrono::milliseconds(m_odom_timeout_ms))
        {
            return;  // Do not send a stale robot position.
        }
        // 0xAA | odom[7] | dwb_cmd[3] | 0xBB: 10 int16, 22 bytes.
        // Positions in mm, yaw/angular velocity in mrad(/s), linear velocities in mm/s.
        std::array<int16_t, 10> words = m_serial_words;
        if (!serial_comm::sendFrameInt16Array(m_serial, words.data(), words.size()))
        {
            m_serial.close();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Serial write failed; closed the port and will retry connection every %d ms.",
                m_reconnect_interval_ms);
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Ignoring DWB cmd_vel containing non-finite velocities");
            return;
        }
        m_dwb_vx = msg->linear.x;
        m_dwb_vy = msg->linear.y;
        m_dwb_wz = msg->angular.z;
    }

    void tagPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // /tag_pose_compensated: [id, x, y, z, roll, pitch, yaw]
        if (msg->data.size() != 7)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Invalid /tag_pose_compensated message: expected 7 values, got %zu.",
                msg->data.size());
            return;
        }
        const double id = msg->data[0];
        const double x = msg->data[1];
        const double y = msg->data[2];
        const double z = msg->data[3];
        const double yaw_deg = msg->data[6];
        if (!std::isfinite(id) || !std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(z) || !std::isfinite(yaw_deg))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Ignoring /tag_pose_compensated message containing non-finite values.");
            return;
        }
        m_tag_id = toInt16Saturated(id);
        m_tag_x = x;
        m_tag_y = y;
        m_tag_z = z;
        m_tag_yaw_deg = yaw_deg;
        m_last_tag_time = this->now();
        m_has_tag = true;
        RCLCPP_INFO(this->get_logger(), "节点拿到相机数据");
    }

    static double quaternionToYaw(double w, double x, double y, double z)
    {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    // 米 -> 毫米并转为 int16（量程约 ±32.767 m，超出做饱和处理）
    static int16_t toMmInt16(double meters)
    {
        double mm = meters * 1000.0;
        if (mm > 32767.0) { mm = 32767.0; }
        if (mm < -32768.0) { mm = -32768.0; }
        return static_cast<int16_t>(std::lround(mm));
    }

    // 弧度 -> 毫弧度并转为 int16（下位机 data_deal[3]*0.001*180/PI 还原为度）
    static int16_t toMradInt16(double radians)
    {
        double mrad = radians * 1000.0;
        if (mrad > 32767.0) { mrad = 32767.0; }
        if (mrad < -32768.0) { mrad = -32768.0; }
        return static_cast<int16_t>(std::lround(mrad));
    }

    // 米/秒 -> 毫米/秒并转为 int16（量程约 ±32.767 m/s，超出做饱和处理）
    static int16_t toMmPerSecondInt16(double meters_per_second)
    {
        double mm_per_second = meters_per_second * 1000.0;
        if (mm_per_second > 32767.0) { mm_per_second = 32767.0; }
        if (mm_per_second < -32768.0) { mm_per_second = -32768.0; }
        return static_cast<int16_t>(std::lround(mm_per_second));
    }

    // 角度（度） -> 0.01 度并转为 int16。
    static int16_t toCentidegInt16(double degrees)
    {
        double centideg = degrees * 100.0;
        if (centideg > 32767.0) { centideg = 32767.0; }
        if (centideg < -32768.0) { centideg = -32768.0; }
        return static_cast<int16_t>(std::lround(centideg));
    }

    static int16_t toInt16Saturated(double value)
    {
        if (value > 32767.0) { value = 32767.0; }
        if (value < -32768.0) { value = -32768.0; }
        return static_cast<int16_t>(std::lround(value));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
    std::string m_odom_frame;
    std::array<int16_t, 10> m_serial_words{};
    double m_dwb_vx = 0.0;
    double m_dwb_vy = 0.0;
    double m_dwb_wz = 0.0;
    int m_odom_timeout_ms = 500;
    std::chrono::steady_clock::time_point m_last_odom_time{};
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_tag_sub;
    rclcpp::TimerBase::SharedPtr m_reconnect_timer;
    serial_comm::SerialPort m_serial;
    std::string m_serial_device;
    int m_serial_baudrate = 115200;
    int m_reconnect_interval_ms = 1000;

    // 滤波器参数与状态
    double m_alpha;
    double m_filtered_x = 0.0;
    double m_filtered_y = 0.0;
    double m_filtered_z = 0.0;
    double m_filtered_yaw = 0.0;
    bool m_first_received = false;
    int printThrehold_ = 10;
    int printIndex_ = 0;

    int m_tag_timeout_ms = 500;
    bool m_has_tag = false;
    rclcpp::Time m_last_tag_time;
    int16_t m_tag_id = -1;
    double m_tag_x = 0.0;
    double m_tag_y = 0.0;
    double m_tag_z = 0.0;
    double m_tag_yaw_deg = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomListener>());
    rclcpp::shutdown();
    return 0;
}
