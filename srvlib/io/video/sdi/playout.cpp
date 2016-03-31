#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __linux__
#include <unistd.h>
#endif
#include <fcntl.h>
#include <sstream>

#include <srvlib/io/video/sdi/playout.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <cinder/app/App.h>
#include <cinder/gl/Context.h>

using namespace srvlib;
using namespace sdi;

PlayoutDevice::PlayoutDevice(const size_t device_index, BMDConfig &config, const BMDDisplayMode target_display_mode, const size_t frame_width, const size_t frame_height): device_index_(device_index), config_(config), output_(NULL), frame_width_(frame_width), frame_height_(frame_height) {

  GetStreamFromDevice();

  EnableVideoAndStartDisplay(&config, target_display_mode);

}



PlayoutDevice::~PlayoutDevice(){

  //deleting input releases delegate
  if(output_ != NULL){
    try{
      output_->Release();
    }
    catch (...){

    }
    output_ = NULL;
  }
    
}



bool PlayoutDevice::GetStreamFromDevice(){

  IDeckLinkIterator* device_iterator;
  HRESULT result;
#ifdef __linux__
  device_iterator = CreateDeckLinkIteratorInstance();
#elif defined _WIN32
  result = CoCreateInstance(CLSID_CDeckLinkIterator, NULL, CLSCTX_ALL, IID_IDeckLinkIterator, (void**)&device_iterator);
  if (result != S_OK) return false;
#endif

  IDeckLink *device = nullptr;
  int idx = (int)device_index_;

  while ((result = device_iterator->Next(&device)) == S_OK){

    if (idx == 0)
      break;
    --idx;

    device->Release();

  }

  if (result != S_OK || device == nullptr){
    
    std::cerr <<  "Unable to get DeckLink device" <<  device_index_ << "\n";
    return false;

  }else{

    result = device->QueryInterface(IID_IDeckLinkOutput, (void**)&output_);

    if (result != S_OK)
      return false;

  }

  return true;

}
  
void PlayoutDevice::SetupDisplay(const size_t width, const size_t height){

  frame_width_ = width;
  frame_height_ = height;

}


bool PlayoutDevice::EnableVideoAndStartDisplay(BMDConfig *config, const BMDDisplayMode target_display_mode ){

  int result;

  IDeckLinkDisplayModeIterator*	display_mode_iterator = nullptr;
  //BMDDisplayMode  displayMode = bmdModePAL; // mode to use for capture and playout
  IDeckLinkDisplayMode *display_mode = nullptr;

  if (output_->GetDisplayModeIterator(&display_mode_iterator) != S_OK){
    std::cerr << "Cannot get Display Mode Iterator.", "DeckLink error.\n";
    return false;
  }
  
  while (display_mode_iterator->Next(&display_mode) == S_OK){
    if (display_mode->GetDisplayMode() == target_display_mode)
      break;
	  
    display_mode->Release();
    display_mode = nullptr;

  }

  display_mode_iterator->Release();

  if(display_mode == nullptr){
    std::cerr << "Error, could not set display mode!\n";
    return false;
  }

  
  result = output_->EnableVideoOutput(display_mode->GetDisplayMode(), config->m_outputFlags);

  if (result != S_OK){
    std::cerr << "Failed to enable video output. Is another application using the card?\n";
    return false;
  }

  //create some initial frames
  auto f_output = output_->CreateVideoFrame(frame_width_, frame_height_, frame_width_ * 4, bmdFormat8BitBGRA, bmdFrameFlagFlipVertical, &frame_);

  if (f_output != S_OK){
    std::cerr << "Failed to create video frame.\n";
    return false;
  }

  return true;

}


void PlayoutDevice::DrawFrameToDisplay() {

  auto res = output_->DisplayVideoFrameSync(frame_);

  if(!SUCCEEDED(res)){
    ci::app::console() << "Failed to display frame.\n";
  }

}
