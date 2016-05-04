#include <srvlib/io/video/sdi/sdi.hpp>
#include <cinder/gl/draw.h>
#include <srvlib/view/renderer.hpp>

#include <cinder/CinderOpenCV.h>
#include <cinder/app/App.h>

using namespace srvlib;
using namespace srvlib::sdi;

SDICameraIO::SDICameraIO() {

  frame_queue_.reset(new FrameQueue);

  const char *frag_shader = "#version 150\n"
    "uniform sampler2D tex0;\n" // We will use two 2D textures.
    "in vec4 vTexCoord0;\n"     // We get the interpolated texture coordinate from the vertex shader.   
    "out vec4 oColor;\n" // The output of our fragment shader is an RGBA color.
    "void main(void)\n"
    "{\n"
    "float tx, ty, Y, Cb, Cr, r, g, b;\n"
    "tx = vTexCoord0.x;\n"
    "ty = vTexCoord0.y;\n"
    "int true_width = textureSize(tex0, 0).x * 2;\n"
    // For U0 Y0 V0 Y1 macropixel, lookup Y0 or Y1 based on whether
    // the original texture x coord is even or odd.	
    "if (fract(floor(tx * true_width + 0.5) / 2.0) > 0.0)\n"
    "  Y = texture2D(tex0, vec2(tx, ty)).a;\n"		// odd so choose Y1
    "else"
    "  Y = texture2D(tex0, vec2(tx, ty)).g;\n"          // even so choose Y0
    "Cb = texture2D(tex0, vec2(tx, ty)).b;\n"
    "Cr = texture2D(tex0, vec2(tx, ty)).r;\n"
    // Y: Undo 1/256 texture value scaling and scale [16..235] to [0..1] range
    // C: Undo 1/256 texture value scaling and scale [16..240] to [-0.5 .. + 0.5] range
    "Y = (Y * 256.0 - 16.0) / 219.0;\n"
    "Cb = (Cb * 256.0 - 16.0) / 224.0 - 0.5;\n"
    "Cr = (Cr * 256.0 - 16.0) / 224.0 - 0.5;\n"
    // Convert to RGB using Rec.709 conversion matrix (see eq 26.7 in Poynton 2003)
    "r = Y + 1.5748 * Cr;\n"
    "g = Y - 0.1873 * Cb - 0.4681 * Cr;\n"
    "b = Y + 1.8556 * Cb;\n"
    // Set alpha to 0.7 for partial transparency when GL_BLEND is enabled
    "oColor = vec4(r, g, b, 1.0);\n"
    "}\n";

  const char *vert_shader = "#version 150\n"
    "uniform mat4 ciModelViewProjection;\n"  // Cinder will automatically send the default matrices and attributes to our shader.
    "in vec4 ciPosition;\n"
    "in vec4 ciTexCoord0;\n"
    "out vec4 vTexCoord0;\n" // We will pass the texture coordinate to the fragment shader as well.

    "void main(void)\n"
    "{\n"
    "vTexCoord0 = ciTexCoord0;\n" // Pass the (1st) texture coordinate of the vertex to the rasterizer.
    // Transform the vertex from object space to '2D space' 
    // and pass it to the rasterizer.
    "gl_Position = ciModelViewProjection * ciPosition;\n"
    "}\n";

  ycrcb_shader_ = ci::gl::GlslProg::create(vert_shader, frag_shader);

}

SDICameraIO::~SDICameraIO(){


}

void SDICameraIO::SetupInput(const size_t device_index, BMDConfig &config, const size_t width, const size_t height){

  capture_.reset(new CaptureDevice(device_index, frame_queue_, config));
  
  fbo_ = ci::gl::Fbo::create(width, height);

}

void SDICameraIO::SetupOutput(const size_t device_index, BMDConfig &config, const BMDDisplayMode target_display_mode, const size_t frame_width, const size_t frame_height, const size_t offset){

  playout_.reset(new PlayoutDevice(device_index, config, target_display_mode, frame_width, frame_height));
  if (offset != 0)
    playout_->SetOffsetToStream(offset);

}

void SDICameraIO::SetOffsetToPlayout(const size_t offset_val){

  playout_->SetOffsetToStream(offset_val);

}

std::shared_ptr<VideoFrame> SDICameraIO::Read(){

  std::shared_ptr<VideoFrame> f(new VideoFrame);

  IDeckLinkVideoInputFrame *video_frame = frame_queue_->pop();

  if (video_frame == nullptr) {
    f->is_valid = false;
    return f;
  }

  void *video_pixels;
  video_frame->GetBytes(&video_pixels);

  size_t width = video_frame->GetWidth();
  size_t height = video_frame->GetHeight();
  long textureSize = video_frame->GetRowBytes() * video_frame->GetHeight();

  ci::gl::Texture::Format format;
  format.dataType(GL_UNSIGNED_INT_8_8_8_8_REV);

  f->gpu_frame = ci::gl::Texture2d::create(video_pixels, GL_BGRA, width / 2, height, format);
  f->is_valid = true;

  //cv::Mat f = toOcv(texture->createSource());

  video_frame->Release();
  return f;

}


void SDICameraIO::Display(std::shared_ptr<VideoFrame> frame){

  IDeckLinkMutableVideoFrame *output_frame = playout_->GetFrame();

  void *data;
  output_frame->GetBytes(&data);

  long rowbytes = output_frame->GetRowBytes();
  long width = output_frame->GetWidth();
  long height = output_frame->GetHeight();
  long memSize = rowbytes * height;

  fbo_->bindFramebuffer();

  ci::gl::clear(ci::Color(0.0, 0.5, 0.1), true);

  DrawOnWindow(frame, fbo_->getBounds(), false);

  //if (rendering_){
  //  draw_psm_on_window(psm1, is_left);
  //  draw_psm_on_window(psm2, is_left);
 // }

  //if (rendering_model_){
  //  draw_model_on_window(mesh_, gl_mesh_pose_, is_left);
 // }


  //load the frame buffer data into the frame memory on the decklink card
  glReadPixels(0, 0, fbo_->getWidth(), fbo_->getHeight(), GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, data);

  //frame->bind();
  //glGetTextureImageEXT(0, GL_TEXTURE_2D, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, data);
  //frame->unbind();

  fbo_->unbindFramebuffer();

  playout_->DrawFrameToDisplay();

}



void SDICameraIO::DrawOnWindow(std::shared_ptr<VideoFrame> frame, const ci::Rectf bounds, bool clear){

  if (clear)  
    ci::gl::clear(ci::Color(0.5, 0.1, 0.0), true);

  renderer::DrawTexture(frame->gpu_frame, glm::ivec2(frame->GetWidth(), frame->GetHeight()), glm::ivec2(bounds.getWidth(), bounds.getHeight()), ycrcb_shader_);

 

}


void SDICameraIO::Read(std::shared_ptr<VideoFrame> left, std::shared_ptr<VideoFrame> right){



}