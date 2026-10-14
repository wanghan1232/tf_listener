#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
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
        m_path_timeout_ms = this->declare_parameter<int>("path_timeout_ms", 500);
        m_odom_timeout_ms = this->declare_parameter<int>("odom_timeout_ms", 500);
        const auto path_topic = this->declare_parameter<std::string>(
            "path_topic", "/local_plan_sampled");
        if (m_path_timeout_ms <= 0 || m_odom_timeout_ms <= 0)
        {
            throw std::invalid_argument("path_timeout_ms and odom_timeout_ms must be positive");
        }
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

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
        m_path_sub = this->create_subscription<nav_msgs::msg::Path>(
            path_topic, rclcpp::QoS(1),
            std::bind(&OdomListener::pathCallback, this, std::placeholders::_1));
        m_path_timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&OdomListener::expirePath, this));

        //   m_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        //        "/fastlio2/lio_odom", 10,
        //        std::bind(&OdomListener::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Subscribing to /lio/imu/odom, /tag_pose_compensated and %s; serial frame: 48 bytes",
            path_topic.c_str());
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
            m_path_count = 0;
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

        if(printIndex_ >= printThrehold_) {
            RCLCPP_INFO(this->get_logger(),
            "filtered pos=(%.3f, %.3f, %.3f) | yaw=%.2f deg",
            m_filtered_x, m_filtered_y, m_filtered_z, m_filtered_yaw * 180.0 / kPi);
            printIndex_ = 0;
        }
        m_odom_words = {
            toMmInt16(m_filtered_x), toMmInt16(m_filtered_y),
            toMmInt16(m_filtered_z), toMradInt16(m_filtered_yaw),
            toMmPerSecondInt16(v.x), toMmPerSecondInt16(v.y), toMmPerSecondInt16(v.z)};
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
            return;  // Do not repeatedly send a stale robot position on path updates.
        }
        if (now - m_last_path_time > std::chrono::milliseconds(m_path_timeout_ms))
        {
            m_path_count = 0;
        }
        // 0xAA | odom[7] | N | path[5][x,y,yaw] | 0xBB
        // 23 个 int16，大端，共 48 字节。位置 mm，yaw 毫弧度，速度 mm/s。
        // N=0 表示无有效路径；未使用的路径槽全部填零。
        std::array<int16_t, 23> words{};
        std::copy(m_odom_words.begin(), m_odom_words.end(), words.begin());
        words[7] = m_path_count;
        std::copy_n(m_path_words.begin(), static_cast<std::size_t>(m_path_count) * 3,
            words.begin() + 8);
        if (!serial_comm::sendFrameInt16Array(m_serial, words.data(), words.size()))
        {
            m_serial.close();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Serial write failed; closed the port and will retry connection every %d ms.",
                m_reconnect_interval_ms);
        }
    }

    void expirePath()
    {
        if (m_path_count > 0 && std::chrono::steady_clock::now() - m_last_path_time >
            std::chrono::milliseconds(m_path_timeout_ms))
        {
            m_path_count = 0;
            sendCombinedFrame();
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Clear first so an invalid update never leaves the previous path active.
        m_path_count = 0;
        m_path_words.fill(0);
        if (msg->poses.empty())
        {
            sendCombinedFrame();
            return;
        }
        if (!m_first_received || msg->header.frame_id.empty() || msg->poses.size() > 5)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Path rejected: need odometry first, a frame and at most 5 points");
            sendCombinedFrame();
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        const bool needs_transform = msg->header.frame_id != m_odom_frame;
        if (needs_transform)
        {
            try
            {
                // A zero stamp means latest TF; otherwise use the path publication time.
                transform = m_tf_buffer->lookupTransform(m_odom_frame, msg->header.frame_id,
                    rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()));
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Cannot transform path to odometry frame: %s", ex.what());
                sendCombinedFrame();
                return;
            }
        }

        std::array<int16_t, 15> path_words{};
        for (std::size_t i = 0; i < msg->poses.size(); ++i)
        {
            auto pose = msg->poses[i];
            auto &q = pose.pose.orientation;
            const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
            if ((!pose.header.frame_id.empty() && pose.header.frame_id != msg->header.frame_id) ||
                !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
                !std::isfinite(pose.pose.position.z) || !std::isfinite(norm) || norm < 1e-9)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Path rejected: invalid coordinates, quaternion or point frame");
                sendCombinedFrame();
                return;
            }
            q.x /= norm;
            q.y /= norm;
            q.z /= norm;
            q.w /= norm;
            if (needs_transform)
            {
                geometry_msgs::msg::PoseStamped converted;
                tf2::doTransform(pose, converted, transform);
                pose = converted;
            }
            const double x = pose.pose.position.x;
            const double y = pose.pose.position.y;
            const auto &orientation = pose.pose.orientation;
            const double yaw = quaternionToYaw(
                orientation.w, orientation.x, orientation.y, orientation.z);
            // Unlike telemetry, never silently clip a path into a different route.
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
                x < -32.768 || x > 32.767 || y < -32.768 || y > 32.767)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Path rejected: point outside int16 millimetre range");
                sendCombinedFrame();
                return;
            }
            path_words[i * 3] = toMmInt16(x);
            path_words[i * 3 + 1] = toMmInt16(y);
            path_words[i * 3 + 2] = toMradInt16(yaw);
        }
        m_path_words = path_words;
        m_path_count = static_cast<int16_t>(msg->poses.size());
        m_last_path_time = std::chrono::steady_clock::now();
        sendCombinedFrame();  // Send every received path update immediately.
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
        RCLCPP_INFO(this->get_logger(),"节点拿到相机数据");
    }

    // 四元数 -> 偏航角（弧度），标准转换公式
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
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
    rclcpp::TimerBase::SharedPtr m_path_timer;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::string m_odom_frame;
    std::array<int16_t, 7> m_odom_words{};
    std::array<int16_t, 15> m_path_words{};
    int16_t m_path_count = 0;
    int m_path_timeout_ms = 500;
    int m_odom_timeout_ms = 500;
    std::chrono::steady_clock::time_point m_last_path_time{};
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
