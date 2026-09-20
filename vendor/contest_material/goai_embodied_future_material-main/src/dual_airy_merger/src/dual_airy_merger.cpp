#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class DualAiryMerger : public rclcpp::Node
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2;

  DualAiryMerger() : Node("dual_airy_merger")
  {
    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    publisher_ = create_publisher<Cloud>("/LIDAR/POINTS_MERGED", qos);
    front_sub_ = create_subscription<Cloud>(
      "/rslidar_front/points", qos,
      [this](Cloud::UniquePtr msg) { receive(std::move(msg), true); });
    rear_sub_ = create_subscription<Cloud>(
      "/rslidar_rear/points", qos,
      [this](Cloud::UniquePtr msg) { receive(std::move(msg), false); });
    RCLCPP_INFO(get_logger(), "Merging CD1 clouds into /LIDAR/POINTS_MERGED");
  }

private:
  static int64_t stamp_ns(const Cloud & cloud)
  {
    return static_cast<int64_t>(cloud.header.stamp.sec) * 1000000000LL +
           static_cast<int64_t>(cloud.header.stamp.nanosec);
  }

  void receive(Cloud::UniquePtr msg, bool is_front)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_front) {
      front_queue_.push_back(std::move(msg));
      if (front_queue_.size() > 5) {
        front_queue_.pop_front();
      }
    } else {
      rear_queue_.push_back(std::move(msg));
      if (rear_queue_.size() > 5) {
        rear_queue_.pop_front();
      }
    }
    try_publish();
  }

  void try_publish()
  {
    constexpr int64_t max_skew_ns = 100000000LL;
    while (!front_queue_.empty() && !rear_queue_.empty()) {
      auto & front = front_queue_.front();
      auto & rear = rear_queue_.front();
      const auto front_ns = stamp_ns(*front);
      const auto rear_ns = stamp_ns(*rear);
      if (std::llabs(front_ns - rear_ns) <= max_skew_ns) {
        publish_pair(*front, *rear, front_ns, rear_ns);
        front_queue_.pop_front();
        rear_queue_.pop_front();
      } else if (front_ns < rear_ns) {
        front_queue_.pop_front();
      } else {
        rear_queue_.pop_front();
      }
    }
  }

  void publish_pair(const Cloud & front, const Cloud & rear, int64_t front_ns, int64_t rear_ns)
  {
    if (front.point_step != rear.point_step || front.fields != rear.fields) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Front/rear PointCloud2 layouts differ");
      return;
    }

    auto merged = std::make_unique<Cloud>();
    merged->header = front_ns >= rear_ns ? front.header : rear.header;
    merged->header.frame_id = "lidar_link";
    merged->height = 1;
    merged->width = front.width * front.height + rear.width * rear.height;
    merged->fields = front.fields;
    merged->is_bigendian = front.is_bigendian;
    merged->point_step = front.point_step;
    merged->data.reserve(front.data.size() + rear.data.size());
    merged->data.insert(merged->data.end(), front.data.begin(), front.data.end());
    merged->data.insert(merged->data.end(), rear.data.begin(), rear.data.end());
    merged->row_step = static_cast<uint32_t>(merged->data.size());
    merged->is_dense = front.is_dense && rear.is_dense;
    publisher_->publish(std::move(merged));
  }

  std::mutex mutex_;
  std::deque<Cloud::UniquePtr> front_queue_;
  std::deque<Cloud::UniquePtr> rear_queue_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr front_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr rear_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualAiryMerger>());
  rclcpp::shutdown();
  return 0;
}
