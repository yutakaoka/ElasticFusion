#include "RealSenseInterface.h"
#include <functional>
#include <cstring>

#ifdef WITH_REALSENSE

const rs2_format kStereoFormat = RS2_FORMAT_Z16;
const int kStereoFps = 30;
const rs2_format kColorFormat = RS2_FORMAT_RGB8;

RealSenseInterface::RealSenseInterface(int inWidth,int inHeight,int inFps)
  : width(inWidth),
  height(inHeight),
  fps(inFps),
    // dev(nullptr),
  initSuccessful(true)
{
  rs2::device_list devices = ctx.query_devices();
  if (devices.size() == 0) {
    std::cerr << "No device connected, please connect a RealSense device" << std::endl;
    //To help with the boilerplate code of waiting for a device to connect
    //The SDK provides the rs2::device_hub class
    rs2::device_hub device_hub(ctx);
    //Using the device_hub we can block the program until a device connects
    dev = device_hub.wait_for_device();
  } else if (devices.size() == 1) {
    dev = devices[0];
  } else {
    std::cerr << "Connected more than 2 sensors. We assume only one sensor." << std::endl;
    errorText = "No device connected.";
    initSuccessful = false;
    return;
  }

  auto sensors = dev.query_sensors();
  rs2::stream_profile stereo_profile;
  rs2::stream_profile color_profile;
  for (auto&& sensor : sensors) {
    std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
    if (module_name == "Stereo Module") {
      stereo = sensor;
      auto profiles = stereo.get_stream_profiles();
      for (auto& profile : profiles) {
	auto video_profile = profile.as<rs2::video_stream_profile>();
	if (video_profile.format() == kStereoFormat &&
	    video_profile.width() == width &&
	    video_profile.height() == height &&
	    video_profile.fps() == fps) {
	  stereo_profile = profile;
	  std::cout << module_name << " ["
		    << video_profile.format() << " "
		    << video_profile.width() << " "
		    << video_profile.height() << " "
		    << video_profile.fps() << "]" << std::endl;
	}
      }
    } else if (module_name == "RGB Camera") {
      color = sensor;
      auto profiles = color.get_stream_profiles();
      for (auto& profile : profiles) {
	auto video_profile = profile.as<rs2::video_stream_profile>();
	if (video_profile.format() == kColorFormat &&
	    video_profile.width() == width &&
	    video_profile.height() == height &&
	    video_profile.fps() == fps) {
	  color_profile = profile;
	  std::cout << module_name << " ["
		    << video_profile.format() << " "
		    << video_profile.width() << " "
		    << video_profile.height() << " "
		    << video_profile.fps() << "]" << std::endl;
	}
      }
    }
  }

  stereo.open(stereo_profile);
  color.open(color_profile);

  latestDepthIndex.assign(-1);
  latestRgbIndex.assign(-1);

  for(int i = 0; i < numBuffers; i++)
  {
    uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
    rgbBuffers[i] = std::pair<uint8_t *,int64_t>(newImage,0);
  }

  for(int i = 0; i < numBuffers; i++)
  {
    uint8_t * newDepth = (uint8_t *)calloc(width * height * 2,sizeof(uint8_t));
    uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
    frameBuffers[i] = std::pair<std::pair<uint8_t *,uint8_t *>,int64_t>(std::pair<uint8_t *,uint8_t *>(newDepth,newImage),0);
  }

  auto color_callback = [this](rs2::frame frame) {
    std::cout << "color" << std::endl;
    lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;
    auto image = frame.as<rs2::video_frame>();
    std::memcpy(rgbBuffers[bufferIndex].first, frame.get_data(),
		image.get_width() * image.get_height() * 3);

    rgbBuffers[bufferIndex].second = lastRgbTime;

    latestRgbIndex++;
  };

  auto stereo_callback = [this](rs2::frame frame) {
    std::cout << "stereo" << std::endl;
    lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

    auto image = frame.as<rs2::video_frame>();
    // The multiplication by 2 is here because the depth is actually uint16_t
    std::memcpy(frameBuffers[bufferIndex].first.first,frame.get_data(),
        image.get_width() * image.get_height() * 2);

    frameBuffers[bufferIndex].second = lastDepthTime;

    int lastImageVal = latestRgbIndex.getValue();

    if(lastImageVal == -1)
      {
        return;
      }

    lastImageVal %= numBuffers;

    std::memcpy(frameBuffers[bufferIndex].first.second,rgbBuffers[lastImageVal].first,
		image.get_width() * image.get_height() * 3);
    
    latestDepthIndex++;
  };

  color.start(color_callback);
  stereo.start(stereo_callback);

  // setAutoExposure(true);
  // setAutoWhiteBalance(true);

  // rgbCallback = new RGBCallback(lastRgbTime,
  //   latestRgbIndex,
  //   rgbBuffers);

  // depthCallback = new DepthCallback(lastDepthTime,
  //   latestDepthIndex,
  //   latestRgbIndex,
  //   rgbBuffers,
  //   frameBuffers);

  // dev->set_frame_callback(rs::stream::depth,*depthCallback);
  // dev->set_frame_callback(rs::stream::color,*rgbCallback);

  // dev->start();
}

RealSenseInterface::~RealSenseInterface()
{
  if(initSuccessful)
  {
    // dev->stop();
    stereo.stop();
    color.stop();
    stereo.close();
    color.stop();

    for(int i = 0; i < numBuffers; i++)
    {
      free(rgbBuffers[i].first);
    }

    for(int i = 0; i < numBuffers; i++)
    {
      free(frameBuffers[i].first.first);
      free(frameBuffers[i].first.second);
    }

  }
}

void RealSenseInterface::setAutoExposure(bool value)
{
  // dev->set_option(rs::option::color_enable_auto_exposure,value);
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
  // dev->set_option(rs::option::color_enable_auto_white_balance,value);
}

bool RealSenseInterface::getAutoExposure()
{
  // return dev->get_option(rs::option::color_enable_auto_exposure);
  return true;
}

bool RealSenseInterface::getAutoWhiteBalance()
{
  // return dev->get_option(rs::option::color_enable_auto_white_balance);
  return true;
}
#else

RealSenseInterface::RealSenseInterface(int inWidth,int inHeight,int inFps)
  : width(inWidth),
  height(inHeight),
  fps(inFps),
  initSuccessful(false)
{
  errorText = "Compiled without Intel RealSense library";
}

RealSenseInterface::~RealSenseInterface()
{
}

void RealSenseInterface::setAutoExposure(bool value)
{
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
}

bool RealSenseInterface::getAutoExposure()
{
  return false;
}

bool RealSenseInterface::getAutoWhiteBalance()
{
  return false;
}
#endif
