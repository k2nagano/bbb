#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <ctime>
#include <iomanip>

#define MAX_WIDTH 1920
#define MAX_HEIGHT 1080

struct SharedMemory {
  unsigned long serial_number;
  int width;
  int height;
  unsigned char data[MAX_HEIGHT * MAX_WIDTH * 3];  // 3 channels for RGB888 format
};

class VideoStreamNode : public rclcpp::Node {
public:
  VideoStreamNode() : Node("video_stream_node"), streaming_(true), recording_(false) {
    // ROS2 Services
    stream_service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_streaming", std::bind(&VideoStreamNode::handle_streaming, this, std::placeholders::_1, std::placeholders::_2));
    record_service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_recording", std::bind(&VideoStreamNode::handle_recording, this, std::placeholders::_1, std::placeholders::_2));
    capture_service_ = this->create_service<std_srvs::srv::Trigger>(
      "capture_frame", std::bind(&VideoStreamNode::handle_capture, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize shared memory
    shm_fd_ = shm_open("/video_shm", O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd_, sizeof(SharedMemory));
    shared_memory_ = (SharedMemory*) mmap(0, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shared_memory_ == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "Shared memory initialization failed.");
      exit(EXIT_FAILURE);
    }

    shared_memory_->serial_number = 0;
    shared_memory_->width = MAX_WIDTH;
    shared_memory_->height = MAX_HEIGHT;

    // GStreamer pipeline for H.264 UDP streaming
    std::string pipeline = "udpsrc port=5600 ! application/x-rtp, payload=96 ! "
                           "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
    cap_.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open video stream.");
      exit(EXIT_FAILURE);
    }

    // Timer to fetch frames at 30 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), std::bind(&VideoStreamNode::timer_callback, this));
  }

  ~VideoStreamNode() {
    munmap(shared_memory_, sizeof(SharedMemory));
    shm_unlink("/video_shm");
  }

private:
  void handle_streaming(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    streaming_ = request->data;
    response->success = true;
    response->message = streaming_ ? "Streaming started" : "Streaming stopped";
  }

  void handle_recording(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    recording_ = request->data;
    if (recording_) {
      // Create VideoWriter for recording
      std::string timestamp = get_current_time_string();
      std::string filename = output_directory_ + "/Video/" + timestamp + ".mp4";
      writer_.open(filename, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(MAX_WIDTH, MAX_HEIGHT));
    } else {
      writer_.release();
    }
    response->success = true;
    response->message = recording_ ? "Recording started" : "Recording stopped";
  }

  void handle_capture(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::string timestamp = get_current_time_string();
    std::string filename = output_directory_ + "/Photo/" + timestamp + ".jpg";
    cv::imwrite(filename, current_frame_);
    response->success = true;
    response->message = "Captured frame saved as " + filename;
  }

  void timer_callback() {
    if (!streaming_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    cap_ >> current_frame_;
    if (current_frame_.empty()) return;

    // Convert frame to RGB format (if not already in that format)
    cv::cvtColor(current_frame_, current_frame_, cv::COLOR_BGR2RGB);

    // Copy to shared memory in RGB888 format
    shared_memory_->serial_number++;
    shared_memory_->width = current_frame_.cols;
    shared_memory_->height = current_frame_.rows;
    memcpy(shared_memory_->data, current_frame_.data, current_frame_.total() * current_frame_.elemSize());

    // Record the frame if recording is active
    if (recording_ && writer_.isOpened()) {
      writer_ << current_frame_;
    }
  }

  std::string get_current_time_string() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream timestamp;
    timestamp << std::put_time(std::localtime(&now_time_t), "%Y%m%d-%H%M%S")
              << '.' << std::setw(3) << std::setfill('0') << now_ms.count();
    return timestamp.str();
  }

  int shm_fd_;
  SharedMemory* shared_memory_;
  std::mutex mutex_;
  bool streaming_;
  bool recording_;
  cv::Mat current_frame_;
  cv::VideoCapture cap_;
  cv::VideoWriter writer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stream_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  std::string output_directory_ = "/your/output/directory";
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoStreamNode>());
  rclcpp::shutdown();
  return 0;
}

