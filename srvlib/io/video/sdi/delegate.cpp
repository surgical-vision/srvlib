
#ifdef __linux__
#include <pthread.h>
#include <unistd.h>
#elif defined _WIN32
#include <thread>
#endif
#include <csignal>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <srvlib/io/video/sdi/delegate.h>
#include <srvlib/io/video/sdi/frame_queue.h>
#include <srvlib/io/video/sdi/playout.h>
#include <thread>

using namespace srvlib;
using namespace sdi;

DeckLinkCaptureDelegate::DeckLinkCaptureDelegate(std::shared_ptr<FrameQueue> frame_queue) : ref_count_(0), deck_link_input_(NULL), frame_queue_(frame_queue) {

#ifdef __linux__
  pthread_mutex_init(&mutex_, NULL);
#endif

}

DeckLinkCaptureDelegate::~DeckLinkCaptureDelegate(){

#ifdef __linux__
  pthread_mutex_destroy(&mutex_);
#endif
}

ULONG DeckLinkCaptureDelegate::AddRef(void){

#ifdef __linux__
  pthread_mutex_lock(&mutex_);
  ref_count_++;
  pthread_mutex_unlock(&mutex_);
#elif _WIN32
  mutex_.lock();
  ref_count_++;
  mutex_.unlock();
#endif
  return (ULONG)ref_count_;

}

ULONG DeckLinkCaptureDelegate::Release(void){

#ifdef __linux__
  pthread_mutex_lock(&mutex_);
  ref_count_--;
  pthread_mutex_unlock(&mutex_);
#elif _WIN32
  mutex_.lock();
  ref_count_--;
  mutex_.unlock();
#endif
  
  if (ref_count_ == 0){
    delete this;
    return 0;
  }
  
  return (ULONG)ref_count_;

}
 
HRESULT DeckLinkCaptureDelegate::VideoInputFrameArrived(IDeckLinkVideoInputFrame* video_frame, IDeckLinkAudioInputPacket* audioFrame){

  if (video_frame){
      
    if (video_frame->GetFlags() & bmdFrameHasNoInputSource){
	
      //pass

    }else{

      video_frame->AddRef(); // decrease ref count handled by consumer thread
      frame_queue_->push(video_frame);
      
    }
  }
  
  return S_OK;
  
}

HRESULT DeckLinkCaptureDelegate::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events, IDeckLinkDisplayMode *mode, BMDDetectedVideoInputFormatFlags){

  // This only gets called if bmdVideoInputEnableFormatDetection was set
  // when enabling video input
  HRESULT result;
  char*	displayModeName = NULL;

  if (!(events & bmdVideoInputDisplayModeChanged))
    return S_OK;

#ifdef __linux__
  result = mode->GetName((const char**)&displayModeName);
#elif defined _WIN32
  displayModeName = new char[100];
  int wslen = MultiByteToWideChar(CP_ACP, 0, (const char*)displayModeName, strlen((const char*)displayModeName), 0, 0);
  BSTR bstr = SysAllocStringLen(0, wslen);
  MultiByteToWideChar(CP_ACP, 0, (const char*)displayModeName, strlen((const char*)displayModeName), bstr, wslen);
  result = mode->GetName(&bstr);
  SysFreeString(bstr);
  delete displayModeName;
  displayModeName = 0x0;
#endif
  
  if (displayModeName)
    free(displayModeName);

  if (deck_link_input_){
    deck_link_input_->StopStreams();
    
    result = deck_link_input_->EnableVideoInput(mode->GetDisplayMode(), bmdFormat8BitYUV, bmdVideoInputFlagDefault);
    if (result != S_OK){
      fprintf(stderr, "Failed to switch video mode\n");
      goto bail;
    }
    
    deck_link_input_->StartStreams();
  }

 bail:
  return S_OK;
}
