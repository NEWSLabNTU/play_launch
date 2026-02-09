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

class TalkerNode : public rclcpp::Node
{
public:
  TalkerNode()
  : Node("talker"),
    total_bytes_read_(0),
    total_messages_published_(0)
  {
    // Declare and get block_size parameter
    this->declare_parameter("block_size", 4096);
    block_size_ = this->get_parameter("block_size").as_int();

    RCLCPP_INFO(this->get_logger(),
      "Talker node starting with block_size=%d bytes", block_size_);

    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "io_stress", 10);

    // Start background reader thread (producer)
    reader_thread_ = std::thread(&TalkerNode::read_urandom_loop, this);

    // Create timer to publish from queue (consumer)
    // High frequency to keep up with fast I/O
    timer_ = this->create_wall_timer(
      1ms,
      std::bind(&TalkerNode::publish_next_block, this));

    // Statistics timer
    stats_timer_ = this->create_wall_timer(
      5s,
      std::bind(&TalkerNode::print_stats, this));
  }

  ~TalkerNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down talker...");

    // Signal shutdown
    queue_.shutdown();

    // Wait for reader thread
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }

    print_stats();
  }

private:
  // Background thread: reads /dev/urandom and pushes to queue
  void read_urandom_loop()
  {
    std::ifstream urandom("/dev/urandom", std::ios::binary);

    if (!urandom.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/urandom");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Reader thread started, reading from /dev/urandom");

    while (rclcpp::ok()) {
      // Allocate buffer
      std::vector<uint8_t> buffer(block_size_);

      // Read block from /dev/urandom
      urandom.read(reinterpret_cast<char*>(buffer.data()), block_size_);

      if (urandom.gcount() != block_size_) {
        RCLCPP_WARN(this->get_logger(), "Short read from /dev/urandom");
        continue;
      }

      total_bytes_read_ += block_size_;

      // Push to queue (blocks if queue is full - natural backpressure)
      queue_.push(std::move(buffer));
    }

    urandom.close();
    RCLCPP_INFO(this->get_logger(), "Reader thread exiting");
  }

  // ROS timer callback: pops from queue and publishes
  void publish_next_block()
  {
    std::vector<uint8_t> buffer;

    // Try to pop with short timeout (non-blocking)
    if (queue_.pop(buffer, 1ms)) {
      // Create message
      auto msg = std_msgs::msg::UInt8MultiArray();
      msg.data = std::move(buffer);

      // Publish
      publisher_->publish(msg);
      total_messages_published_++;
    }
  }

  void print_stats()
  {
    RCLCPP_INFO(this->get_logger(),
      "Stats: %zu MB read, %zu messages published, queue size: %zu",
      total_bytes_read_.load() / (1024 * 1024),
      total_messages_published_.load(),
      queue_.size());
  }

  // ROS components
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Pipeline components
  ThreadSafeQueue<std::vector<uint8_t>> queue_;
  std::thread reader_thread_;

  // Parameters
  int block_size_;

  // Statistics
  std::atomic<size_t> total_bytes_read_;
  std::atomic<size_t> total_messages_published_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerNode>());
  rclcpp::shutdown();
  return 0;
}
