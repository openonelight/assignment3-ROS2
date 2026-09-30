// Fake SDK: verifies error handling against the real SDK declarations.
// No device or vendor binary is used. This does NOT test actual USB streaming.
#include <iostream>
#include <map>
#include "hik_camera/camera.hpp"

namespace {
bool online=true, started=false, conversion_fail=false, start_fail=false;
bool stop_fail=false;
int fail_gain_writes=0, releases=0, destroyed=0;
std::map<std::string,float> values{{"ExposureTime",10000.0F}, {"Gain",0.0F}, {"AcquisitionFrameRate",30.0F}};
std::string format="BayerRG8";
MV_CC_DEVICE_INFO device{};
void require(bool condition, const char *message) {if (!condition) throw std::runtime_error(message);}
template<class F> void rejects(F function) {
  bool rejected=false;
  try {function();} catch (const std::exception &) {rejected=true;}
  require(rejected, "Expected rejection");
}
}
extern "C" {
int __stdcall MV_CC_EnumDevices(unsigned int, MV_CC_DEVICE_INFO_LIST *list) {
  device.nTLayerType=MV_USB_DEVICE;
  std::strcpy(reinterpret_cast<char *>(device.SpecialInfo.stUsb3VInfo.chSerialNumber), "TEST_SERIAL");
  list->nDeviceNum=online?1:0; list->pDeviceInfo[0]=&device; return MV_OK;
}
int __stdcall MV_CC_CreateHandle(void **h, const MV_CC_DEVICE_INFO *) {*h=&device; return MV_OK;}
int __stdcall MV_CC_OpenDevice(void *, unsigned int, unsigned short) {return MV_OK;}
int __stdcall MV_CC_CloseDevice(void *) {return MV_OK;}
int __stdcall MV_CC_DestroyHandle(void *) {++destroyed; return MV_OK;}
bool __stdcall MV_CC_IsDeviceConnected(void *) {return online;}
int __stdcall MV_CC_SetImageNodeNum(void *, unsigned int) {return MV_OK;}
int __stdcall MV_CC_StartGrabbing(void *) {if(start_fail){start_fail=false;return MV_E_PARAMETER;} started=true;return MV_OK;}
int __stdcall MV_CC_StopGrabbing(void *) {if(stop_fail){stop_fail=false;return MV_E_PARAMETER;} started=false;return MV_OK;}
int __stdcall MV_CC_SetEnumValueByString(void *, const char *key, const char *v) {
  if(std::string(key)=="PixelFormat") format=v; return MV_OK;
}
int __stdcall MV_CC_SetBoolValue(void *, const char *, bool) {return MV_OK;}
int __stdcall MV_CC_GetEnumValue(void *, const char *, MVCC_ENUMVALUE *v) {
  v->nSupportedNum=2;v->nSupportValue[0]=1;v->nSupportValue[1]=2;
  v->nCurValue=format=="BayerRG8"?1:2;return MV_OK;
}
int __stdcall MV_CC_GetEnumEntrySymbolic(void *, const char *, MVCC_ENUMENTRY *v) {
  std::strcpy(v->chSymbolic, v->nValue==1?"BayerRG8":"Mono8");return MV_OK;
}
int __stdcall MV_CC_GetFloatValue(void *, const char *key, MVCC_FLOATVALUE *v) {
  v->fCurValue=values.at(key);v->fMin=std::string(key)=="Gain"?0.0F:1.0F;
  v->fMax=std::string(key)=="Gain"?17.0F:(std::string(key)=="ExposureTime"?1000000.0F:250.0F);return MV_OK;
}
int __stdcall MV_CC_SetFloatValue(void *, const char *key, float v) {
  if(std::string(key)=="Gain" && fail_gain_writes>0){--fail_gain_writes;return MV_E_PARAMETER;}
  values[key]=v;return MV_OK;
}
int __stdcall MV_CC_GetImageBuffer(void *, MV_FRAME_OUT *f, unsigned int) {
  static unsigned char pixels[4]={10,20,30,40};
  f->pBufAddr=pixels;f->stFrameInfo.nWidth=2;f->stFrameInfo.nHeight=2;
  f->stFrameInfo.nFrameLen=4;f->stFrameInfo.enPixelType=PixelType_Gvsp_BayerRG8;return MV_OK;
}
int __stdcall MV_CC_FreeImageBuffer(void *, MV_FRAME_OUT *) {++releases;return MV_OK;}
int __stdcall MV_CC_ConvertPixelTypeEx(void *, MV_CC_PIXEL_CONVERT_PARAM_EX *c) {
  if(conversion_fail)return MV_E_PARAMETER;
  c->nDstLen=c->nWidth*c->nHeight*3;
  std::memset(c->pDstBuffer, 100, c->nDstLen);return MV_OK;
}
}
int main() {
  try {
    hik_camera::Camera c; hik_camera::Settings original;
    rejects([&]{c.open("WRONG_SERIAL",original);});
    c.open("TEST_SERIAL",original);
    require(c.is_open() && started,"Open failed");
    auto candidate=original;candidate.exposure_us=20000;candidate.gain_db=2;
    fail_gain_writes=1;
    rejects([&]{c.reconfigure(candidate,original);});
    require(values["ExposureTime"]==10000 && values["Gain"]==0 && started,"Partial write not rolled back");
    candidate.pixel_format="InvalidFormat";
    rejects([&]{c.reconfigure(candidate,original);});
    require(format=="BayerRG8" && started,"Pixel format rollback failed");
    candidate=original;candidate.gain_db=999;
    rejects([&]{c.reconfigure(candidate,original);});
    candidate=original;candidate.frame_rate=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{c.reconfigure(candidate,original);});
    candidate=original;candidate.exposure_us=20000;candidate.pixel_format="Mono8";
    c.reconfigure(candidate,original);
    require(values["ExposureTime"]==20000 && format=="Mono8","Valid setting failed");
    start_fail=true;
    rejects([&]{c.reconfigure(original,candidate);});
    require(started && values["ExposureTime"]==20000,"Start failure not rolled back");
    stop_fail=true;
    rejects([&]{c.reconfigure(original,candidate);});
    require(started && values["ExposureTime"]==20000,"Stop failure not rolled back");
    const int releases_before=releases;
    hik_camera::Image image; c.grab(image);
    require(image.bgr.size()==12 && releases==releases_before+1,"Frame layout/release failed");
    conversion_fail=true;
    rejects([&]{c.grab(image);});
    require(releases==releases_before+2,"SDK buffer leaked on conversion failure");
    rejects([&]{c.reconfigure(original,candidate);});
    require(format=="Mono8" && started,"Conversion failure should roll back format");
    conversion_fail=false;online=false;
    rejects([&]{c.reconfigure(original,candidate);});
    online=true;fail_gain_writes=2;
    rejects([&]{c.reconfigure(original,candidate);});
    require(!c.is_open(),"Rollback failure must close device");
    c.open("TEST_SERIAL",candidate);
    require(values["ExposureTime"]==20000 && format=="Mono8","Reconnect did not reapply settings");
    std::cout << "PASS: serial selection, bounds, nonfinite, partial rollback, stop/start failure, conversion buffer release, offline reject, rollback failure close, reconnect restore\n";
    return 0;
  } catch (const std::exception &e) {std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
