#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include "hik_camera/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using namespace std::chrono_literals;
class CameraNode final : public rclcpp::Node {
 public:
  CameraNode() : Node("hik_camera") {
    serial_=declare_parameter<std::string>("serial_number", "", descriptor("USB serial from MVS, restart to change", true));
    const auto topic=declare_parameter<std::string>("image_topic", "/image_raw", descriptor("Image topic, restart to change", true));
    frame_id_=declare_parameter<std::string>("frame_id", "camera_optical_frame", descriptor("Image frame, restart to change", true));
    const bool reliable=declare_parameter<bool>("reliable", true, descriptor("Reliable image QoS; false for best effort", true));
    desired_.exposure_us=declare_parameter<double>("exposure_time", 10000.0, descriptor("Manual exposure in microseconds; hardware range checked"));
    desired_.gain_db=declare_parameter<double>("gain", 0.0, descriptor("Manual gain in dB; hardware range checked"));
    desired_.frame_rate=declare_parameter<double>("frame_rate", 30.0, descriptor("Requested camera FPS; actual publish FPS appears in diagnostics"));
    desired_.pixel_format=declare_parameter<std::string>("pixel_format", "BayerRG8", descriptor("Camera pixel format; published Image always bgr8"));
    hik_camera::validate(desired_);
    if (serial_.empty()) throw std::runtime_error("Set serial_number to the serial shown by MVS");
    auto qos=rclcpp::QoS(rclcpp::KeepLast(2)).durability_volatile();
    if (reliable) qos.reliable(); else qos.best_effort();
    images_=create_publisher<sensor_msgs::msg::Image>(topic, qos);
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
    parameter_callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful=false;
      auto candidate=desired_;
      bool changed=false;
      try {
        for (const auto &p:parameters) {
          if (p.get_name()=="exposure_time") {candidate.exposure_us=p.as_double(); changed=true;}
          else if (p.get_name()=="gain") {candidate.gain_db=p.as_double(); changed=true;}
          else if (p.get_name()=="frame_rate") {candidate.frame_rate=p.as_double(); changed=true;}
          else if (p.get_name()=="pixel_format") {candidate.pixel_format=p.as_string(); changed=true;}
          else throw std::runtime_error("Only exposure_time, gain, frame_rate and pixel_format can change at runtime");
        }
        if (changed) {
          camera_.reconfigure(candidate, desired_);
          desired_=candidate;
          last_frame_=Steady::now();
          period_frames_=0; period_start_=last_frame_; publish_fps_=0;
          error_.clear();
          RCLCPP_INFO(get_logger(), "Accepted configuration; readbacks will appear in diagnostics");
        }
        result.successful=true;
      } catch (const std::exception &error) {result.reason=error.what();}
      return result;
    });
    timer_=create_wall_timer(1ms, [this] {tick();});
    status_timer_=create_wall_timer(1s, [this] {status();});
  }
 private:
  using Steady=std::chrono::steady_clock;
  static rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string &description, bool read_only=false) {
    rcl_interfaces::msg::ParameterDescriptor d; d.description=description; d.read_only=read_only; return d;
  }
  void tick() {
    const auto current=Steady::now();
    if (!camera_.is_open()) {
      if (current < next_connect_) return;
      next_connect_=current+2s;
      try {
        camera_.open(serial_, desired_);
        last_frame_=Steady::now(); period_start_=last_frame_; period_frames_=0; publish_fps_=0;
        error_.clear();
        RCLCPP_INFO(get_logger(), "Connected USB serial=%s; %s", serial_.c_str(), camera_.report().c_str());
      } catch (const std::exception &error) {
        camera_.close(); error_=error.what();
        RCLCPP_WARN(get_logger(), "%s; retry in 2s", error_.c_str());
      }
      return;
    }
    try {
      if (!camera_.connected()) throw std::runtime_error("Device disconnected");
      if (!camera_.grab(buffer_)) {
        const double allowance=std::max({5.0, desired_.exposure_us/1e6*3.0, 3.0/desired_.frame_rate});
        if (std::chrono::duration<double>(Steady::now()-last_frame_).count()>allowance)
          throw std::runtime_error("No frames beyond timeout; check USB and trigger configuration");
        return;
      }
      auto message=std::make_unique<sensor_msgs::msg::Image>();
      message->header.stamp=now(); // Host receipt time, not hardware exposure timestamp.
      message->header.frame_id=frame_id_;
      message->width=buffer_.width; message->height=buffer_.height;
      message->encoding="bgr8"; message->is_bigendian=0; message->step=buffer_.width*3;
      message->data=std::move(buffer_.bgr);
      images_->publish(std::move(message));
      last_frame_=Steady::now(); ++period_frames_; ++total_frames_; error_.clear();
    } catch (const std::exception &error) {
      error_=error.what(); camera_.close(); publish_fps_=0; period_frames_=0;
      next_connect_=Steady::now()+2s;
      RCLCPP_ERROR(get_logger(), "%s; reconnect will restore last accepted settings", error_.c_str());
    }
  }
  void status() {
    const auto current=Steady::now();
    const double elapsed=std::chrono::duration<double>(current-period_start_).count();
    publish_fps_=elapsed>0 ? period_frames_/elapsed : 0;
    period_start_=current; period_frames_=0;
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name="hik_camera/stream"; s.hardware_id=serial_;
    s.level=camera_.is_open() ? 0 : 2;
    s.message=camera_.is_open() ? "Connected; host publication statistics" : error_;
    const auto add=[&s](const std::string &key, const std::string &value) {
      diagnostic_msgs::msg::KeyValue kv; kv.key=key; kv.value=value; s.values.push_back(kv);
    };
    add("requested_fps", std::to_string(desired_.frame_rate));
    add("publish_fps", std::to_string(publish_fps_));
    add("total_published", std::to_string(total_frames_));
    add("exposure_us_requested", std::to_string(desired_.exposure_us));
    add("gain_db_requested", std::to_string(desired_.gain_db));
    add("camera_pixel_format", desired_.pixel_format);
    add("output_encoding", "bgr8");
    if (camera_.is_open()) {
      try {add("hardware_readback", camera_.report());}
      catch (const std::exception &e) {s.level=1; s.message=e.what();}
    }
    array.status.push_back(s); diagnostics_->publish(array);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "connected=%d requested_fps=%.2f publish_fps=%.2f total=%llu",
      camera_.is_open(), desired_.frame_rate, publish_fps_, static_cast<unsigned long long>(total_frames_));
  }
  hik_camera::Camera camera_;
  hik_camera::Settings desired_;
  hik_camera::Image buffer_;
  std::string serial_, frame_id_, error_{"Waiting for camera"};
  Steady::time_point next_connect_{}, last_frame_{}, period_start_{Steady::now()};
  uint64_t total_frames_{0}, period_frames_{0}; double publish_fps_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  bool initialized=false;
  int result=0;
  try {
    hik_camera::check(MV_CC_Initialize(), "SDK Initialize"); initialized=true;
    { auto node=std::make_shared<CameraNode>(); rclcpp::spin(node); }
  } catch (const std::exception &e) {std::fprintf(stderr, "Fatal: %s\n", e.what()); result=1;}
  if (initialized) MV_CC_Finalize();
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
