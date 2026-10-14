#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace serial_comm
{

constexpr uint8_t FRAME_HEADER = 0xaa;   // 包头
constexpr uint8_t FRAME_TAIL   = 0xbb;   // 包尾

// 把整数波特率映射为 termios 的 speed_t
inline speed_t baudToSpeed(int baudrate)
{
    switch (baudrate)
    {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600; 
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

// 串口基础封装（Linux termios，8N1，原始模式）
class SerialPort
{
public:
    SerialPort() = default;

    ~SerialPort()
    {
        close();
    }

    // 禁止拷贝，避免句柄被复制后重复关闭
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baudrate)
    {
        close();

        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0)
        {
            return false;
        }

        // 清除 O_NDELAY，改为阻塞模式
        fcntl(fd_, F_SETFL, 0);

        termios options{};
        if (tcgetattr(fd_, &options) != 0)
        {
            close();
            return false;
        }

        const speed_t speed = baudToSpeed(baudrate);
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;        // 8 数据位
        options.c_cflag &= ~PARENB;    // 无校验位
        options.c_cflag &= ~CSTOPB;    // 1 停止位
        options.c_cflag &= ~CRTSCTS;   // 关闭硬件流控

        options.c_iflag &= ~(IXON | IXOFF | IXANY);   // 关闭软件流控
        options.c_iflag &= ~(ICRNL | INLCR);          // 不做换行转换
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 原始模式
        options.c_oflag &= ~OPOST;                    // 原始输出

        options.c_cc[VMIN]  = 0;
        options.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &options) != 0)
        {
            close();
            return false;
        }

        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool isOpen() const { return fd_ >= 0; }

    ssize_t write(const uint8_t* data, size_t len)
    {
        return ::write(fd_, data, len);
    }

private:
    int fd_ = -1;
};

// 把一个 16 位有符号数（INT16S）按大端序写入缓冲区：高字节在前、低字节在后。
// 对应下位机解析：data_deal[j] = (高字节 << 8) | 低字节
inline void appendInt16BE(std::vector<uint8_t>& buf, int16_t value)
{
    const uint16_t u = static_cast<uint16_t>(value);
    buf.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));  // 高字节
    buf.push_back(static_cast<uint8_t>(u & 0xFF));         // 低字节
}

// =====================================================================
// 模板发送接口：自动加上包头 0xAA 与包尾 0xBB
//
// 中间数据为 16 位有符号数（INT16S），按大端序打包，与下位机一致：
//   帧格式： 0xAA | word0_hi word0_lo word1_hi word1_lo ... | 0xBB
//
// 用法示例（以后可自由填写中间数据以及数量）：
//   sendFrameInt16(port, x, y, z);          // 任意数量
//   sendFrameInt16Array(port, arr, n);      // 发送 n 个 word
// =====================================================================

// 底层：发送一段原始字节（自动加包头 0xAA、包尾 0xBB）
inline bool sendFrame(SerialPort& port, const uint8_t* data, size_t count)
{
    if (!port.isOpen())
    {
        return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(count + 2);
    frame.push_back(FRAME_HEADER);
    frame.insert(frame.end(), data, data + count);
    frame.push_back(FRAME_TAIL);

    const ssize_t written = port.write(frame.data(), frame.size());
    return written == static_cast<ssize_t>(frame.size());
}

// 变参模板：发送任意数量的 16 位有符号数（大端，自动加包头包尾）
template <typename... Args>
bool sendFrameInt16(SerialPort& port, const Args&... args)
{
    std::vector<uint8_t> payload;
    payload.reserve(sizeof...(args) * 2);
    (appendInt16BE(payload, static_cast<int16_t>(args)), ...);
    return sendFrame(port, payload.data(), payload.size());
}

// 发送 count 个 16 位有符号数（大端，自动加包头包尾）
template <typename T>
bool sendFrameInt16Array(SerialPort& port, const T* data, size_t count)
{
    std::vector<uint8_t> payload;
    payload.reserve(count * 2);
    for (size_t i = 0; i < count; ++i)
    {
        appendInt16BE(payload, static_cast<int16_t>(data[i]));
    }
    return sendFrame(port, payload.data(), payload.size());
}

}  // namespace serial_comm
