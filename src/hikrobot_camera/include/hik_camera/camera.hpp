#pragma once
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "MvCameraControl.h"

namespace hik_camera {
struct Settings {
  double exposure_us{10000.0};
  double gain_db{0.0};
  double frame_rate{30.0};
  std::string pixel_format{"BayerRG8"};
};
inline void validate(const Settings &s) {
  if (!std::isfinite(s.exposure_us) || s.exposure_us <= 0 ||
      !std::isfinite(s.gain_db) || s.gain_db < 0 ||
      !std::isfinite(s.frame_rate) || s.frame_rate <= 0)
    throw std::runtime_error("Exposure/frame_rate must be finite and positive; gain must be finite and nonnegative");
  if (s.pixel_format.empty()) throw std::runtime_error("Empty pixel_format");
}
inline void check(int code, const std::string &operation) {
  if (code == MV_OK) return;
  std::ostringstream out;
  out << operation << " failed: SDK 0x" << std::hex << std::uppercase
      << static_cast<uint32_t>(code);
  throw std::runtime_error(out.str());
}
template<typename T, size_t N> std::string text(const T (&value)[N]) {
  const char *p = reinterpret_cast<const char *>(value);
  size_t n = 0;
  while (n < N && p[n]) ++n;
  return std::string(p, n);
}
struct Image {
  uint32_t width{}, height{}, frame_number{};
  std::vector<uint8_t> bgr;
};

// All calls are serialized by the node's single-threaded executor.
class Camera {
 public:
  Camera() = default;
  Camera(const Camera &) = delete;
  Camera &operator=(const Camera &) = delete;
  ~Camera() { close(); }
  bool is_open() const { return handle_ != nullptr && opened_; }
  bool connected() const { return is_open() && MV_CC_IsDeviceConnected(handle_); }
  void close() noexcept {
    if (handle_) {
      if (grabbing_) MV_CC_StopGrabbing(handle_);
      if (opened_) MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
    }
    handle_ = nullptr; opened_ = false; grabbing_ = false;
  }
  void open(const std::string &serial, const Settings &s) {
    close();
    validate(s);
    if (serial.empty()) throw std::runtime_error("serial_number must not be empty");
    MV_CC_DEVICE_INFO_LIST devices{};
    check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "EnumDevices USB");
    MV_CC_DEVICE_INFO *chosen = nullptr;
    for (unsigned int i=0; i<devices.nDeviceNum; ++i) {
      auto *device=devices.pDeviceInfo[i];
      if (device && device->nTLayerType == MV_USB_DEVICE &&
          text(device->SpecialInfo.stUsb3VInfo.chSerialNumber) == serial) chosen=device;
    }
    if (!chosen) throw std::runtime_error("USB camera serial " + serial + " not found; close MVS and check cable/permissions");
    try {
      check(MV_CC_CreateHandle(&handle_, chosen), "CreateHandle");
      check(MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0), "OpenDevice");
      opened_=true;
      check(MV_CC_SetImageNodeNum(handle_, 4), "SetImageNodeNum");
      apply(s);
      start();
    } catch (...) { close(); throw; }
  }
  std::vector<std::string> formats() const {
    MVCC_ENUMVALUE list{};
    check(MV_CC_GetEnumValue(handle_, "PixelFormat", &list), "Get PixelFormat");
    if (list.nSupportedNum > sizeof(list.nSupportValue)/sizeof(list.nSupportValue[0]))
      throw std::runtime_error("Invalid PixelFormat enum length");
    std::vector<std::string> result;
    for (unsigned int i=0; i<list.nSupportedNum; ++i) {
      MVCC_ENUMENTRY entry{}; entry.nValue=list.nSupportValue[i];
      check(MV_CC_GetEnumEntrySymbolic(handle_, "PixelFormat", &entry), "Get PixelFormat symbolic");
      result.push_back(text(entry.chSymbolic));
    }
    return result;
  }
  std::string report() const {
    std::ostringstream out;
    for (const char *key : {"ExposureTime", "Gain", "AcquisitionFrameRate"}) {
      MVCC_FLOATVALUE value{};
      check(MV_CC_GetFloatValue(handle_, key, &value), std::string("Read ")+key);
      out << key << "=" << value.fCurValue << " range=[" << value.fMin << "," << value.fMax << "] ";
    }
    MVCC_ENUMVALUE value{};
    check(MV_CC_GetEnumValue(handle_, "PixelFormat", &value), "Read PixelFormat");
    MVCC_ENUMENTRY entry{}; entry.nValue=value.nCurValue;
    check(MV_CC_GetEnumEntrySymbolic(handle_, "PixelFormat", &entry), "Read PixelFormat name");
    out << "PixelFormat=" << text(entry.chSymbolic) << " supported=";
    for (const auto &name : formats()) out << name << ",";
    return out.str();
  }
  // Reject a failed change and restore the last accepted configuration.
  // If rollback itself fails, close the device and let the node reconnect.
  void reconfigure(const Settings &next, const Settings &previous) {
    validate(next);
    if (!connected()) throw std::runtime_error("Camera disconnected; parameter update rejected");
    try {
      stop(); apply(next); start();
      if (next.pixel_format != previous.pixel_format) {
        // Confirm that the SDK can convert a real frame before committing a format.
        Image probe;
        const auto begin=std::chrono::steady_clock::now();
        const double limit=std::max({3.0, next.exposure_us/1e6*2.0+1.0, 2.0/next.frame_rate+1.0});
        while (!grab(probe)) {
          if (!connected() || std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>limit)
            throw std::runtime_error("No convertible frame after pixel format change");
        }
      }
    } catch (const std::exception &error) {
      const std::string reason=error.what();
      try { stop(); apply(previous); start(); }
      catch (const std::exception &rollback) {
        close();
        throw std::runtime_error(reason + "; rollback failed, reconnect required: " + rollback.what());
      }
      throw std::runtime_error(reason + "; previous settings restored");
    }
  }
  bool grab(Image &image) {
    MV_FRAME_OUT frame{};
    const int code=MV_CC_GetImageBuffer(handle_, &frame, 20);
    if (code == MV_E_NODATA) return false;
    check(code, "GetImageBuffer");
    // Release the SDK buffer even when conversion/allocation throws.
    struct Release {
      void *handle; MV_FRAME_OUT *frame;
      ~Release() { MV_CC_FreeImageBuffer(handle, frame); }
    } release{handle_, &frame};
    image.width=frame.stFrameInfo.nExtendWidth ? frame.stFrameInfo.nExtendWidth : frame.stFrameInfo.nWidth;
    image.height=frame.stFrameInfo.nExtendHeight ? frame.stFrameInfo.nExtendHeight : frame.stFrameInfo.nHeight;
    image.frame_number=frame.stFrameInfo.nFrameNum;
    const uint64_t bytes=uint64_t(image.width)*image.height*3;
    if (!image.width || !image.height || bytes > 512ULL*1024*1024 || !frame.pBufAddr)
      throw std::runtime_error("Invalid or unexpectedly large frame");
    image.bgr.resize(static_cast<size_t>(bytes));
    MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
    conversion.nWidth=image.width; conversion.nHeight=image.height;
    conversion.enSrcPixelType=frame.stFrameInfo.enPixelType;
    conversion.pSrcData=frame.pBufAddr;
    conversion.nSrcDataLen=frame.stFrameInfo.nFrameLen;
    conversion.enDstPixelType=PixelType_Gvsp_BGR8_Packed;
    conversion.pDstBuffer=image.bgr.data();
    conversion.nDstBufferSize=static_cast<unsigned int>(bytes);
    check(MV_CC_ConvertPixelTypeEx(handle_, &conversion), "ConvertPixelTypeEx to BGR8");
    if (conversion.nDstLen != bytes) throw std::runtime_error("Unexpected converted image length");
    return true;
  }
 private:
  void start() { check(MV_CC_StartGrabbing(handle_), "StartGrabbing"); grabbing_=true; }
  void stop() {
    if (grabbing_) { check(MV_CC_StopGrabbing(handle_), "StopGrabbing"); grabbing_=false; }
  }
  void set_enum(const char *key, const char *value) {
    check(MV_CC_SetEnumValueByString(handle_, key, value), std::string("Set ")+key+"="+value);
  }
  void set_float(const char *key, double requested) {
    MVCC_FLOATVALUE limits{};
    check(MV_CC_GetFloatValue(handle_, key, &limits), std::string("Get limits ")+key);
    if (requested < limits.fMin || requested > limits.fMax) {
      std::ostringstream message;
      message << key << "=" << requested << " outside [" << limits.fMin << "," << limits.fMax << "]";
      throw std::runtime_error(message.str());
    }
    check(MV_CC_SetFloatValue(handle_, key, static_cast<float>(requested)), std::string("Set ")+key);
    MVCC_FLOATVALUE actual{};
    check(MV_CC_GetFloatValue(handle_, key, &actual), std::string("Read back ")+key);
    // Device quantization is accepted within 0.1%; actual readbacks are reported.
    if (std::abs(actual.fCurValue-requested) > std::max(0.001, std::abs(requested)*0.001))
      throw std::runtime_error(std::string(key)+" readback differs from request");
  }
  void apply(const Settings &s) {
    set_enum("AcquisitionMode", "Continuous");
    // Disable both common trigger selectors when the device exposes them.
    for (const char *selector : {"FrameBurstStart", "FrameStart"}) {
      if (MV_CC_SetEnumValueByString(handle_, "TriggerSelector", selector) == MV_OK)
        set_enum("TriggerMode", "Off");
    }
    set_enum("TriggerMode", "Off");
    set_enum("ExposureAuto", "Off");
    set_enum("GainAuto", "Off");
    const auto supported=formats();
    if (std::find(supported.begin(), supported.end(), s.pixel_format) == supported.end())
      throw std::runtime_error("Unsupported PixelFormat: " + s.pixel_format);
    set_enum("PixelFormat", s.pixel_format.c_str());
    check(MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true), "Enable frame rate");
    set_float("ExposureTime", s.exposure_us);
    set_float("Gain", s.gain_db);
    set_float("AcquisitionFrameRate", s.frame_rate);
  }
  void *handle_{nullptr};
  bool opened_{false}, grabbing_{false};
};
}  // namespace hik_camera
