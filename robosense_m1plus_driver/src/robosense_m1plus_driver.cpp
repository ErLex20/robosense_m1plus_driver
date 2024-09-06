// Robosense M1 Plus Driver

// Alexandru Cretu <alexandru.cretu@alumni.uniroma2.eu>
// Intelligent Systems Lab <isl.torvergata@gmail.com>

// 6 September, 2024

// This is free software.
// You can redistribute it and/or modify this file under the
// terms of the GNU General Public License as published by the Free Software
// Foundation; either version 3 of the License, or (at your option) any later
// version.

// This file is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along with
// this file; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include "robosense_m1plus_interfaces/msg/difop.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <cstring>

#define BUFFER_SIZE         2048

// MSOP
#define MSOP_PORT           6699
#define MSOP_LENGTH         1210

#define HEADER_MSOP         32
#define NUM_CHANNELS        5
#define SIZE_CHANNELS       9
#define NUM_BLOCKS          25
#define SIZE_BLOCKS         47

#define OFFSET_SEC          10
#define LENGTH_SEC          6
#define OFFSET_MSEC         16
#define LENGTH_MSEC         4
#define OFFSET_RADIUS       2
#define OFFSET_ELEVATION    4
#define OFFSET_AZIMUTH      6
#define OFFSET_INTENSITY    8
#define OFFSET_CENTRAL      32768
#define RESOLUTION_METERS   0.005f
#define RESOLUTION_DEGREES  0.01f

// Point cloud
#define MAX_POINTS          78750
#define HEIGHT              126
#define WIDTH               625
#define POINT_STEP          28

#define POINTS_PER_BLOCK    25
#define POINTS_PER_MESSAGE  125
#define POINTS_PER_RING     625

// DIFOP
#define DIFOP_PORT          7788
#define DIFOP_LENGTH        256

#define HEADER_DIFOP        8
#define OFFSET_FREQUENCY    8
#define OFFSET_SOURCE_IP    9
#define LENGTH_SOURCE_IP    4
#define OFFSET_DEST_IP      13
#define LENGTH_DEST_IP      4
#define OFFSET_MAC_ADDR     17
#define LENGTH_MAC_ADDR     6
#define OFFSET_MSOP_PORT    24
#define LENGTH_MSOP_PORT    2
#define OFFSET_DIFOP_PORT   26
#define LENGTH_DIFOP_PORT   2
#define OFFSET_PL_PN        28
#define LENGTH_PL_PN        5
#define OFFSET_PS_PN        33
#define LENGTH_PS_PN        5
#define OFFSET_RETURN_MODE  54
#define OFFSET_SYNC_MODE    55
#define OFFSET_SYNC_STATUS  56
#define OFFSET_TIME_STATUS  57
#define LENGTH_TIME_STATUS  10
#define OFFSET_BATTERY_VOLT 67
#define LENGTH_BATTERY_VOLT 2
#define OFFSET_FAULT_STATUS 136

class robosense_m1plus_driver : public rclcpp::Node
{
public:
    robosense_m1plus_driver()
    : Node("robosense_m1plus_driver"),
      stop_thread_(false),
      ready_for_processing_(false),
      processing_done_(false),
      buffer_1_ready_(true),
      buffer_2_ready_(false)
      {

        // Declare parameters
        this->declare_parameter<std::string>("ros_frame_id", "rslidar");
        this->declare_parameter<std::string>("ros_send_point_cloud_topic", "/rslidar_points");
        this->declare_parameter<std::string>("ros_send_difop_topic", "/difop");
        this->declare_parameter<float>("start_x", 0.0);
        this->declare_parameter<float>("start_y", 0.0);
        this->declare_parameter<float>("start_z", 0.0);
        this->declare_parameter<bool>("publish_difop", false);
        this->declare_parameter<bool>("use_lidar_timer", false);

        // Load parameters
        this->get_parameter("ros_frame_id", frame_id_);
        this->get_parameter("ros_send_point_cloud_topic", point_cloud_topic_);
        this->get_parameter("ros_send_difop_topic", difop_topic_);
        this->get_parameter("start_x", start_x_);
        this->get_parameter("start_y", start_y_);
        this->get_parameter("start_z", start_z_);
        this->get_parameter("publish_difop", publish_difop_);
        this->get_parameter("use_lidar_timer", use_lidar_timer_);

        // Create a UDP socket
        sockfd_msop_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_msop_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket MSOP");
            return;
        }

        // Configure socket address
        sockaddr_in servaddr_msop {};
        servaddr_msop.sin_family = AF_INET;
        servaddr_msop.sin_addr.s_addr = INADDR_ANY; // Accept packets from any IP
        servaddr_msop.sin_port = htons(MSOP_PORT); // MSOP port for the LiDAR

        // Bind socket to the address
        if (bind(sockfd_msop_, (struct sockaddr *)&servaddr_msop, sizeof(servaddr_msop)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket MSOP");
            close(sockfd_msop_);
            return;
        }

        // Create a publisher for the PointCloud2 message
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~" + point_cloud_topic_, 10);
        // Create a publisher for the DIFOP message
        difop_publisher_ = this->create_publisher<robosense_m1plus_interfaces::msg::DIFOP>("~" + difop_topic_, 10);

        // Initialize PointCloud2 message fields
        point_cloud_msg_.header.frame_id = frame_id_;
        point_cloud_msg_.fields.resize(6); // x, y, z, intensity, ring, time
        for (size_t i = 0; i < 4; ++i)
        {
            point_cloud_msg_.fields[i].name = i == 0 ? "x" : i == 1 ? "y" : i == 2 ? "z" : "intensity";
            point_cloud_msg_.fields[i].offset = i * 4;
            point_cloud_msg_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            point_cloud_msg_.fields[i].count = 1;
        }
        // Add ring field
        point_cloud_msg_.fields[4].name = "ring";
        point_cloud_msg_.fields[4].offset = 16; // Offset at 16 bytes after x, y, z, intensity
        point_cloud_msg_.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
        point_cloud_msg_.fields[4].count = 1;
        // Add time field
        point_cloud_msg_.fields[5].name = "time";
        point_cloud_msg_.fields[5].offset = 20; // Offset at 20 bytes after x, y, z, intensity, ring
        point_cloud_msg_.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
        point_cloud_msg_.fields[5].count = 1;
        // Set correct dimensions
        point_cloud_msg_.height = HEIGHT; // Number of rows
        point_cloud_msg_.width = WIDTH;   // Number of columns
        point_cloud_msg_.point_step = POINT_STEP; // Size of each point in bytes
        point_cloud_msg_.row_step = POINT_STEP * WIDTH; // Set the row_step (number of bytes in a row)
        point_cloud_msg_.data.resize(MAX_POINTS * POINT_STEP); // Preallocate the data buffer with a fixed size
        point_cloud_msg_.is_dense = true; // No NaN values

        // Create threads for receiving and processing data
        receive_thread_ = std::thread(&robosense_m1plus_driver::receiveLoop, this);
        process_thread_ = std::thread(&robosense_m1plus_driver::processLoop, this);

        RCLCPP_INFO(this->get_logger(), "LiDAR MSOP Receiver started");

        if (publish_difop_)
        {
            // Create a UDP socket
            sockfd_difop_ = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd_difop_ < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create socket DIFOP");
                return;
            }

            // Configure socket address
            sockaddr_in servaddr_difop {};
            servaddr_difop.sin_family = AF_INET;
            servaddr_difop.sin_addr.s_addr = INADDR_ANY; // Accept packets from any IP
            servaddr_difop.sin_port = htons(DIFOP_PORT); // DIFOP port for the LiDAR

            // Bind socket to the address
            if (bind(sockfd_difop_, (struct sockaddr *)&servaddr_difop, sizeof(servaddr_difop)) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to bind socket DIFOP");
                close(sockfd_difop_);
                return;
            }

            difop_thread_ = std::thread(&robosense_m1plus_driver::difopLoop, this);

            RCLCPP_INFO(this->get_logger(), "LiDAR DIFOP Receiver started");
        }
    }

    ~robosense_m1plus_driver()
    {
        stop_thread_ = true; // Signal the threads to stop
        queue_cv_.notify_all(); // Unblock the threads if they are waiting
        if (receive_thread_.joinable())
        {
            receive_thread_.join(); // Wait for receive thread to finish
        }
        if (process_thread_.joinable())
        {
            process_thread_.join(); // Wait for process thread to finish
        }
        if (difop_thread_.joinable())
        {
            difop_thread_.join(); // Wait for difop thread to finish
        }
        close(sockfd_msop_); // Close the socket MSOP
        close(sockfd_difop_); // Close the socket DIFOP
    }

private:
    void receiveLoop()
    {
        char buffer[BUFFER_SIZE];
        sockaddr_in cliaddr{};
        socklen_t len = sizeof(cliaddr);
        size_t packet_count = 1;

        while (!stop_thread_) {
            ssize_t n = recvfrom(sockfd_msop_, buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&cliaddr, &len);

            if (n == MSOP_LENGTH)
            {
                // Get the packet's sequence number from the header
                uint16_t num = (static_cast<uint8_t>(buffer[4]) << 8) |static_cast<uint8_t>(buffer[5]);

                // Start saving packets from the first one
                if (num == packet_count)
                {
                    {
                        // Lock for saving data
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        auto& queue = buffer_1_ready_ ? data_queue_1_ : data_queue_2_;
                        queue.emplace(buffer, buffer + n);
                    }

                    packet_count++;

                    // One scan is completed only when 630 packets are received
                    if (packet_count == (WIDTH - 1))
                    {
                        // Lock for processing
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        ready_for_processing_ = true;
                        queue_cv_.notify_one();
                        packet_count = 1;
                    }
                }
            }
            else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                RCLCPP_ERROR(this->get_logger(), "recvfrom MSOP failed: %s", strerror(errno));
            }
        }
    }

    void processLoop()
    {
        while (!stop_thread_)
        {
            // Lock for processing
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]
            {
                return ready_for_processing_ || stop_thread_;
            });

            if (stop_thread_ && data_queue_1_.empty() && data_queue_2_.empty())
            {
                break;
            }

            // Select queue
            auto& active_queue = buffer_1_ready_ ? data_queue_1_ : data_queue_2_;

            size_t points_processed = 0;
            size_t packet_count = 0;
            size_t message_ring_count = 0;
            size_t message_count = 0;

            // Process all available data in the selected queue
            while (!active_queue.empty())
            {
                // Pop packet from queue
                auto data = std::move(active_queue.front());
                active_queue.pop();

                const uint8_t* header_ptr = data.data();

                // Get the time from the pc clock
                double timestamp_header = static_cast<double>(this->get_clock()->now().seconds());
                // Get the time from the lidar clock
                if (use_lidar_timer_)
                {
                    uint64_t seconds = 0;
                    for (size_t i = 0; i < LENGTH_SEC; ++i)
                    {
                        seconds = (seconds << 8) | header_ptr[OFFSET_SEC + i];
                    }
                    uint32_t microseconds = 0;
                    for (size_t i = 0; i < LENGTH_MSEC; ++i)
                    {
                        microseconds = (microseconds << 8) | header_ptr[OFFSET_MSEC + i];
                    }
                    timestamp_header = static_cast<double>(seconds) + static_cast<double>(microseconds) * 1e-6;
                }

                // Process the packet
                for (size_t channel_idx = 0; channel_idx < NUM_CHANNELS; ++channel_idx)
                {
                    for (size_t block_idx = 0; block_idx < NUM_BLOCKS; ++block_idx)
                    {
                        const uint8_t* block_ptr = header_ptr + HEADER_MSOP + block_idx * SIZE_BLOCKS;
                        size_t offset = channel_idx * SIZE_CHANNELS;

                        uint8_t time_offset = block_ptr[offset];
                        double timestamp_point = timestamp_header + static_cast<double>(time_offset) * 1e-6;

                        float radius = ((block_ptr[offset + OFFSET_RADIUS] << 8) | block_ptr[offset + (OFFSET_RADIUS + 1)]) * RESOLUTION_METERS;

                        float elevation = (((block_ptr[offset + OFFSET_ELEVATION] << 8) | block_ptr[offset + (OFFSET_ELEVATION + 1)]) - OFFSET_CENTRAL) * RESOLUTION_DEGREES;

                        float azimuth = (((block_ptr[offset + OFFSET_AZIMUTH] << 8) | block_ptr[offset + (OFFSET_AZIMUTH + 1)]) - OFFSET_CENTRAL) * RESOLUTION_DEGREES;

                        float intensity = static_cast<float>(block_ptr[offset + OFFSET_INTENSITY]);

                        float x = start_x_ + radius * cosf(elevation * M_PI / 180.0f) * cosf(azimuth * M_PI / 180.0f);
                        float y = start_y_ + radius * cosf(elevation * M_PI / 180.0f) * sinf(azimuth * M_PI / 180.0f);
                        float z = start_z_ + radius * sinf(elevation * M_PI / 180.0f);

                        uint16_t ring = (HEIGHT - 1) - message_ring_count;

                        uint8_t* ptr = point_cloud_msg_.data.data() + (block_idx + message_count * POINTS_PER_BLOCK + channel_idx * POINTS_PER_MESSAGE + message_ring_count * POINTS_PER_RING) * point_cloud_msg_.point_step;
                        memcpy(ptr, &x, sizeof(float));
                        memcpy(ptr + 4, &y, sizeof(float));
                        memcpy(ptr + 8, &z, sizeof(float));
                        memcpy(ptr + 12, &intensity, sizeof(float));
                        memcpy(ptr + 16, &ring, sizeof(uint16_t));
                        memcpy(ptr + 20, &timestamp_point, sizeof(double));

                        points_processed++;
                    }
                }

                packet_count++;
                message_count++;

                if (packet_count % 5 == 0)
                {
                    message_count = 0;
                    message_ring_count++;
                }
            }

            // Set the header stamp
            point_cloud_msg_.header.stamp = this->get_clock()->now();
            // Publish the ROS2 message
            if (points_processed > 0)
            {
                point_cloud_publisher_->publish(point_cloud_msg_);
            }

            // Notify the receive thread that processing is done
            ready_for_processing_ = false;
            processing_done_ = true;
            queue_cv_.notify_one();
        }
    }

    void difopLoop()
    {
        char buffer[BUFFER_SIZE];
        sockaddr_in cliaddr {};
        socklen_t len = sizeof(cliaddr);

        while (!stop_thread_)
        {
            ssize_t n = recvfrom(sockfd_difop_, buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&cliaddr, &len);

            if (n == DIFOP_LENGTH)
            {
                // Create a new DIFOPMsg message
                auto message = robosense_m1plus_interfaces::msg::DIFOP();

                // Extract and assign data to the message
                memcpy(message.header.begin(), buffer, HEADER_DIFOP);

                message.frequency_setting = buffer[OFFSET_FREQUENCY];

                uint32_t source_ip;
                memcpy(&source_ip, buffer + OFFSET_SOURCE_IP, LENGTH_SOURCE_IP);
                source_ip = ntohl(source_ip);
                message.source_ip = source_ip;

                uint32_t destination_ip;
                memcpy(&destination_ip, buffer + OFFSET_DEST_IP, LENGTH_DEST_IP);
                destination_ip = ntohl(destination_ip);
                message.destination_ip = destination_ip;

                std::copy(buffer + OFFSET_MAC_ADDR, buffer + OFFSET_MAC_ADDR + LENGTH_MAC_ADDR, message.mac_address.begin());

                uint16_t msop_port;
                memcpy(&msop_port, buffer + OFFSET_MSOP_PORT, LENGTH_MSOP_PORT);
                msop_port = ntohs(msop_port);
                message.msop_port = msop_port;

                uint16_t difop_port;
                memcpy(&difop_port, buffer + OFFSET_DIFOP_PORT, LENGTH_DIFOP_PORT);
                difop_port = ntohs(difop_port);
                message.difop_port = difop_port;

                std::copy(buffer + OFFSET_PL_PN, buffer + OFFSET_PL_PN + LENGTH_PL_PN, message.main_board_pl_pn.begin());
                std::copy(buffer + OFFSET_PS_PN, buffer + OFFSET_PS_PN + LENGTH_PS_PN, message.main_board_ps_pn.begin());

                message.return_mode = buffer[OFFSET_RETURN_MODE];
                message.timesync_mode = buffer[OFFSET_SYNC_MODE];
                message.timesync_status = buffer[OFFSET_SYNC_STATUS];

                std::copy(buffer + OFFSET_TIME_STATUS, buffer + OFFSET_TIME_STATUS + LENGTH_TIME_STATUS, message.time_status.begin());

                uint16_t battery_volt;
                memcpy(&battery_volt, buffer + OFFSET_BATTERY_VOLT, LENGTH_BATTERY_VOLT);
                battery_volt = ntohs(battery_volt);
                message.battery_volt = battery_volt;

                message.lidar_fault_status = buffer[OFFSET_FAULT_STATUS];

                // Publish the ROS2 message
                difop_publisher_->publish(message);
            }
            else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                RCLCPP_ERROR(this->get_logger(), "recvfrom DIFOP failed: %s", strerror(errno));
            }
        }
    }

    std::string frame_id_;
    std::string point_cloud_topic_;
    std::string difop_topic_;
    float start_x_;
    float start_y_;
    float start_z_;
    bool publish_difop_;
    bool use_lidar_timer_;

    int sockfd_msop_;
    int sockfd_difop_;
    bool stop_thread_;
    bool ready_for_processing_;
    bool processing_done_;
    bool buffer_1_ready_;
    bool buffer_2_ready_;
    std::thread receive_thread_;
    std::thread process_thread_;
    std::thread difop_thread_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<robosense_m1plus_interfaces::msg::DIFOP>::SharedPtr difop_publisher_;
    std::queue<std::vector<uint8_t>> data_queue_1_;
    std::queue<std::vector<uint8_t>> data_queue_2_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    sensor_msgs::msg::PointCloud2 point_cloud_msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robosense_m1plus_driver>());
    rclcpp::shutdown();
    return 0;
}
