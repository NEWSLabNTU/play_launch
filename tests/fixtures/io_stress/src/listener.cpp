#include <chrono>
#include <fstream>
#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace std::chrono_literals;

// Thread-safe bounded queue for pipeline architecture with backpressure
template<typename T>
class ThreadSafeQueue {
public:
  explicit ThreadSafeQueue(size_t max_size = 100) : max_size_(max_size) {}

  // Push blocks if queue is full (backpressure)
  void push(T value) {
    std::unique_lock<std::mutex> lock(mutex_);

    // Block until queue has space or shutdown
    cv_push_.wait(lock, [this]{ return queue_.size() < max_size_ || shutdown_; });

    if (shutdown_) {
      return;  // Don't push if shutting down
    }

    queue_.push(std::move(value));
    cv_pop_.notify_one();  // Wake up consumer
  }

  // Pop with timeout
  bool pop(T& value, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!cv_pop_.wait_for(lock, timeout, [this]{ return !queue_.empty() || shutdown_; })) {
      return false;  // Timeout
    }

    if (shutdown_ && queue_.empty()) {
      return false;  // Shutdown with empty queue
    }

    value = std::move(queue_.front());
    queue_.pop();
    cv_push_.notify_one();  // Wake up producer
    return true;
  }

  void shutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    cv_pop_.notify_all();   // Wake up all consumers
    cv_push_.notify_all();  // Wake up all producers
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable cv_pop_;   // For consumers waiting on data
  std::condition_variable cv_push_;  // For producers waiting on space
  size_t max_size_;
  bool shutdown_ = false;
};

class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode()
  : Node("listener"),
    total_bytes_written_(0),
    total_messages_received_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Listener node starting");

    // Create subscription
    subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "io_stress", 10,
      std::bind(&ListenerNode::topic_callback, this, std::placeholders::_1));

    // Start background writer thread (consumer)
    writer_thread_ = std::thread(&ListenerNode::write_devnull_loop, this);

    // Statistics timer
    stats_timer_ = this->create_wall_timer(
      5s,
      std::bind(&ListenerNode::print_stats, this));
  }

  ~ListenerNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down listener...");

    // Signal shutdown
    queue_.shutdown();

    // Wait for writer thread
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }

    print_stats();
  }

private:
  // ROS subscription callback: receives message and pushes to queue (producer)
  void topic_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    total_messages_received_++;

    // Push to queue for background writing
    queue_.push(msg->data);
  }

  // Background thread: pops from queue and writes to /dev/null (consumer)
  void write_devnull_loop()
  {
    std::ofstream devnull("/dev/null", std::ios::binary);

    if (!devnull.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/null");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Writer thread started, writing to /dev/null");

    std::vector<uint8_t> buffer;

    while (rclcpp::ok()) {
      // Pop from queue with timeout
      if (queue_.pop(buffer, 100ms)) {
        // Write to /dev/null
        devnull.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());

        if (!devnull.good()) {
          RCLCPP_WARN(this->get_logger(), "Write to /dev/null failed");
          continue;
        }

        total_bytes_written_ += buffer.size();
      }
    }

    // Drain remaining queue on shutdown
    while (queue_.pop(buffer, 10ms)) {
      devnull.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
      total_bytes_written_ += buffer.size();
    }

    devnull.close();
    RCLCPP_INFO(this->get_logger(), "Writer thread exiting");
  }

  void print_stats()
  {
    RCLCPP_INFO(this->get_logger(),
      "Stats: %zu MB written, %zu messages received, queue size: %zu",
      total_bytes_written_.load() / (1024 * 1024),
      total_messages_received_.load(),
      queue_.size());
  }

  // ROS components
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Pipeline components
  ThreadSafeQueue<std::vector<uint8_t>> queue_;
  std::thread writer_thread_;

  // Statistics
  std::atomic<size_t> total_bytes_written_;
  std::atomic<size_t> total_messages_received_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
