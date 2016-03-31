/* -LICENSE-START-
** Copyright (c) 2013 Blackmagic Design
**
** Permission is hereby granted, free of charge, to any person or organization
** obtaining a copy of the software and accompanying documentation covered by
** this license (the "Software") to use, reproduce, display, distribute,
** execute, and transmit the Software, and to prepare derivative works of the
** Software, and to permit third-parties to whom the Software is furnished to
** do so, all subject to the following:
**
** The copyright notices in the Software and this entire statement, including
** the above license grant, this restriction and the following disclaimer,
** must be included in all copies of the Software, in whole or in part, and
** all derivative works of the Software, unless such copies or derivative
** works are solely in the form of machine-executable object code generated by
** a source language processor.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
** SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
** FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
** ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
** DEALINGS IN THE SOFTWARE.
** -LICENSE-END-
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __linux__
#include <unistd.h>
#elif defined _WIN32
#include <stdio.h>
#endif
#include <fcntl.h>

#include <srvlib/io/video/sdi/capture.h>

using namespace srvlib;
using namespace sdi;

CaptureDevice::CaptureDevice(int index, std::shared_ptr<FrameQueue> frame_queue, BMDConfig &config): stream_running_(false), delegate_(nullptr), device_index_(index), config_(config), display_mode_(nullptr), input_(nullptr) {

  if(!GetStreamFromDevice(&config)){
    std::cerr << "Error getting stream from device!\n";
  }

  SetupDelegate(frame_queue);

  if(!EnableVideoAndStartStream(&config)){
    std::cerr << "Error starting the stream!\n";
  }

}

void CaptureDevice::Destroy() {

  if(stream_running_){
    StopStream();
    stream_running_ = false;
  }

  if(display_mode_ != nullptr){
    display_mode_->Release();
    display_mode_ = nullptr;
  }

  //deleting input releases delegate
  if(input_ != nullptr){
    input_->Release();
    input_ = nullptr;
  }



}

CaptureDevice::~CaptureDevice(){

  Destroy();
    
}



bool CaptureDevice::GetStreamFromDevice(BMDConfig *config){

  IDeckLinkIterator* device_iterator;
  HRESULT result;
#ifdef __linux__
  device_iterator = CreateDeckLinkIteratorInstance();
#elif defined _WIN32
  result = CoCreateInstance(CLSID_CDeckLinkIterator, NULL, CLSCTX_ALL, IID_IDeckLinkIterator, (void**)&device_iterator);
  if (result != S_OK) return false;
#endif

  IDeckLink *device = nullptr;
  int idx = device_index_;

  while ((result = device_iterator->Next(&device)) == S_OK){

    if (idx == 0)
      break;
    --idx;

    device->Release();

  }

  if (result != S_OK || device == nullptr){
    
    fprintf(stderr, "Unable to get DeckLink device %u\n", device_index_);
    return false;

  }else{

    result = device->QueryInterface(IID_IDeckLinkInput, (void**)&input_);

    if (result != S_OK)
      return false;

  }


  device_iterator->Release();

  GetDisplayModeFromDevice(device, config);

  device->Release();

  return true;

}

size_t CaptureDevice::GetWidth() const {

  return display_mode_->GetWidth();

}

size_t CaptureDevice::GetHeight() const {

  return display_mode_->GetHeight();

}

void CaptureDevice::SetupDelegate(std::shared_ptr<FrameQueue> frame_queue){

  delegate_ = new DeckLinkCaptureDelegate(frame_queue);
  delegate_->SetDeckLinkInput(input_);
  input_->SetCallback(delegate_);  
 
}

bool CaptureDevice::EnableVideoAndStartStream(BMDConfig *config){

  int result;

  result = input_->EnableVideoInput(display_mode_->GetDisplayMode(), config->m_pixelFormat, config->m_inputFlags);
  if (result != S_OK){
    fprintf(stderr, "Failed to enable video input. Is another application using the card?\n");
    return false;
  }

  return StartStream();

}

bool CaptureDevice::StartStream(){

  int result;
  result = input_->StartStreams();
  if(result != S_OK){
    return false;
  }

  stream_running_ = true;

  return true;

}


void CaptureDevice::StopStream(){

  try{
    input_->StopStreams();
  }
  catch (...){
  }
  try{
    input_->DisableVideoInput();
  }
  catch (...)
  {
  }

  stream_running_ = false;

}

bool CaptureDevice::GetDisplayModeFromDevice(IDeckLink *device, BMDConfig *config){

  IDeckLinkDisplayModeIterator* displayModeIterator = NULL;
  int display_mode_index = 0;

  int result;

  //auto-detect the display mode
  if (config->m_displayModeIndex == -1){
    IDeckLinkAttributes *attributes;
    bool format_detection_supported;
    // Check the card supports format detection
    result = device->QueryInterface(IID_IDeckLinkAttributes, (void**)&attributes);
    if (result == S_OK){
#ifdef __linux__
      result = attributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &format_detection_supported);
#elif defined _WIN32
      BOOL format_detection_supported_;
      result = attributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &format_detection_supported_);
      format_detection_supported = format_detection_supported_;
#endif
      if (result != S_OK || !format_detection_supported){
	fprintf(stderr, "Format detection is not supported on this device\n");
	return false;
      }
    }

    config->m_inputFlags |= bmdVideoInputEnableFormatDetection;

    // Format detection still needs a valid mode to start with
    display_mode_index = 0;

  }
  else{

    display_mode_index = config->m_displayModeIndex;

  }

  //get display mode from device
  result = input_->GetDisplayModeIterator(&displayModeIterator);
  if (result != S_OK)
    return false;

  int idx = display_mode_index;
  while ((result = displayModeIterator->Next(&display_mode_)) == S_OK){
    if (idx == 0)
      break;
    --idx;

    display_mode_->Release();
  }

  if (result != S_OK || display_mode_ == nullptr) {
    fprintf(stderr, "Unable to get display mode %d\n", config->m_displayModeIndex);
    return false;
  }

  // Get display mode name
  char *display_mode_name = nullptr;
#ifdef __linux__
  result = display_mode_->GetName((const char**)&display_mode_name);
#elif defined _WIN32
  display_mode_name = new char[100];
  int wslen = MultiByteToWideChar(CP_ACP, 0, (const char*)display_mode_name, strlen((const char*)display_mode_name), 0, 0);
  BSTR bstr = SysAllocStringLen(0, wslen);
  MultiByteToWideChar(CP_ACP, 0, (const char*)display_mode_name, strlen((const char*)display_mode_name), bstr, wslen);
  result = display_mode_->GetName(&bstr);
  SysFreeString(bstr);
  delete[] display_mode_name;
  display_mode_name = 0x0;
#endif
  
  if (result != S_OK){
    display_mode_name = (char *)malloc(32);
#ifdef __linux__
    snprintf(display_mode_name, 32, "[index %d]", config->m_displayModeIndex);
#elif _WIN32
    _snprintf(display_mode_name, 32, "[index %d]", config->m_displayModeIndex);
#endif
  }
  if(display_mode_name)
    delete display_mode_name;

  // Check display mode is supported with given options
  BMDDisplayModeSupport	display_mode_supported;
  result = input_->DoesSupportVideoMode(display_mode_->GetDisplayMode(), config->m_pixelFormat, bmdVideoInputFlagDefault, &display_mode_supported, NULL);
  if (result != S_OK)
    return false;

  if (display_mode_supported == bmdDisplayModeNotSupported){
    fprintf(stderr, "The display mode %s is not supported with the selected pixel format\n", display_mode_name);
    return false;
  }

  return true;  

}
 

