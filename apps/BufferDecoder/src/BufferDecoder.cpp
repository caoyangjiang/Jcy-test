// // Copyright @ 2016 Caoyang Jiang

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/file.h>
}

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
// class TileDecoder
// {
//  public:
//   TileDecoder()
//   {
//   }

//   ~TileDecoder()
//   {
//   }

//   bool Initialize()
//   {
//   }

//   bool StartThread()
//   {
//     while (!eof_ && !shutdown_)
//     {

//     }
//   }

//  private:
//   bool shutdown_ = true;

// }

struct buffer_data
{
  uint8_t *ptr;
  size_t size;  ///< size left in the buffer
};

struct Multi_buffer
{
  struct buffer_data buffers[10];
  int usebufferid = 0;
};

int decode_frame(AVCodecContext *avctx,
                 AVFrame *frame,
                 int *frame_count,
                 AVPacket *pkt)
{
  int len, got_frame;
  len = avcodec_decode_video2(avctx, frame, &got_frame, pkt);
  if (len < 0)
  {
    std::cout << "Error while decoding frame" << *frame_count << std::endl;
    return len;
  }
  std::cout << "Packet size: " << pkt->size << " " << len << std::endl;
  if (got_frame)
  {
    std::cout << "Got a frame" << frame->width << " " << frame->height << " "
              << frame->linesize[0] << std::endl;

    (*frame_count)++;
  }

  if (pkt->data)
  {
    pkt->size -= len;
    pkt->data += len;
  }
  return 0;
}

static int read_packet(void *opaque, uint8_t *buf, int buf_size)
{
  // struct Multi_buffer *mb = (struct Multi_buffer *)opaque;
  // struct buffer_data *bd  = &mb->buffers[mb->usebufferid];

  struct buffer_data *bd = (struct buffer_data *)opaque;

  buf_size = FFMIN(buf_size, bd->size);
  printf("buf_size: %d, ptr:%p size:%zu\n", buf_size, bd->ptr, bd->size);
  // printf("use buffer: %d\n", mb->usebufferid);
  /*copy internal buffer data to buf */
  memcpy(buf, bd->ptr, buf_size);
  bd->ptr += buf_size;
  bd->size -= buf_size;

  // if (bd->size == 0) mb->usebufferid++;
  return buf_size;
}
int main(int argc, char *argv[])
{
  avcodec_register_all();
  av_register_all();
  AVPacket avpkt;
  AVFrame *frame;
  AVFormatContext *fmt_ctx  = NULL;
  AVIOContext *avio_ctx     = NULL;
  AVCodec *avcodec          = NULL;
  AVCodecContext *codec_ctx = NULL;
  uint8_t *buffer = NULL, *avio_ctx_buffer = NULL;
  size_t buffer_size, avio_ctx_buffer_size = 4096;
  char *input_filename  = NULL;
  int ret               = 0;
  struct buffer_data bd = {0};
  struct Multi_buffer mbuffer;
  input_filename = argv[1];

  std::ifstream ifs(argv[2], std::ifstream::in | std::ifstream::binary);
  std::ifstream ifs2(argv[3], std::ifstream::in | std::ifstream::binary);
  uint8_t inbuf[170726 + AV_INPUT_BUFFER_PADDING_SIZE];
  uint8_t inbuf2[3900349 + AV_INPUT_BUFFER_PADDING_SIZE];
  size_t filesize = 0;

  memset(inbuf + 170726, 0, AV_INPUT_BUFFER_PADDING_SIZE);
  memset(inbuf2 + 3900349, 0, AV_INPUT_BUFFER_PADDING_SIZE);

  ifs.seekg(0, ifs.end);
  filesize = ifs.tellg();
  ifs.seekg(0, ifs.beg);
  ifs.read(reinterpret_cast<char *>(inbuf), 170726);

  ifs2.seekg(0, ifs2.end);
  filesize = ifs2.tellg();
  ifs2.seekg(0, ifs2.beg);
  ifs2.read(reinterpret_cast<char *>(inbuf2), 3900349);

  /* slurp file content into buffer */
  ret = av_file_map(input_filename, &buffer, &buffer_size, 0, NULL);
  if (ret < 0) goto end;
  /* fill opaque structure used by the AVIOContext read callback */
  bd.ptr  = buffer;
  bd.size = buffer_size;

  mbuffer.buffers[0].ptr  = buffer;
  mbuffer.buffers[0].size = buffer_size;
  mbuffer.buffers[1].ptr  = inbuf2;
  mbuffer.buffers[1].size = 3900349;
  mbuffer.buffers[2].ptr  = inbuf;
  mbuffer.buffers[2].size = 170726;

  std::cout << "Buffer size " << buffer_size << std::endl;
  if (!(fmt_ctx = avformat_alloc_context()))
  {
    ret = AVERROR(ENOMEM);
    goto end;
  }
  avio_ctx_buffer =
      reinterpret_cast<uint8_t *>(av_malloc(avio_ctx_buffer_size));
  if (!avio_ctx_buffer)
  {
    ret = AVERROR(ENOMEM);
    goto end;
  }
  avio_ctx = avio_alloc_context(
      avio_ctx_buffer, avio_ctx_buffer_size, 0, &bd, &read_packet, NULL, NULL);
  if (!avio_ctx)
  {
    ret = AVERROR(ENOMEM);
    goto end;
  }

  fmt_ctx->pb = avio_ctx;

  ret = avformat_open_input(&fmt_ctx, NULL, NULL, NULL);
  if (ret < 0)
  {
    fprintf(stderr, "Could not open input\n");
    goto end;
  }

  fmt_ctx->streams[0]->codec->pix_fmt = AV_PIX_FMT_YUV420P;
  std::cout << fmt_ctx->nb_streams << " " << fmt_ctx->bit_rate << " "
            << fmt_ctx->video_codec_id << std::endl;
  ret = avformat_find_stream_info(fmt_ctx, NULL);
  if (ret < 0)
  {
    fprintf(stderr, "Could not find stream information\n");
    goto end;
  }

  av_dump_format(fmt_ctx, 0, input_filename, 0);

  avcodec = avcodec_find_decoder(fmt_ctx->streams[0]->codec->codec_id);

  if (avcodec_open2(fmt_ctx->streams[0]->codec, avcodec, NULL) < 0)
  {
    std::cout << "Could not open codec." << std::endl;
  }
end:

  int frame_count = 0;
  av_init_packet(&avpkt);
  frame = av_frame_alloc();
  while (true)
  {
    if (av_read_frame(fmt_ctx, &avpkt) < 0)
    {
      av_packet_unref(&avpkt);
      avpkt.data    = NULL;
      avpkt.size    = 0;
      int tempcount = 0;
      // flush
      do
      {
        tempcount = frame_count;
        decode_frame(fmt_ctx->streams[0]->codec, frame, &frame_count, &avpkt);
      } while (frame_count != tempcount);
      break;
    }

    std::cout << avpkt.size << std::endl;
    // printf("0x%x, 0x%x, 0x%x, 0x%x\n",
    //        avpkt.data[0],
    //        avpkt.data[1],
    //        avpkt.data[2],
    //        avpkt.data[3]);
    if (decode_frame(fmt_ctx->streams[0]->codec, frame, &frame_count, &avpkt) <
        0)
    {
      std::cout << "decode_frame failed" << std::endl;
      break;
    }
  }

  bd.ptr  = inbuf;
  bd.size = 170726;
  while (av_read_frame(fmt_ctx, &avpkt) < 0)
  {
  }
  av_packet_unref(&avpkt);
  //}
  std::cout << frame_count << std::endl;
  ifs.close();
  av_frame_free(&frame);
  avformat_close_input(&fmt_ctx);
  avformat_free_context(fmt_ctx);
  /* note: the internal buffer could have changed, and be !=
  avio_ctx_buffer */
  if (avio_ctx)
  {
    av_freep(&avio_ctx->buffer);
    av_freep(&avio_ctx);
  }
  av_file_unmap(buffer, buffer_size);
  if (ret < 0)
  {
    std::cout << "Error occurred: " << std::endl;
    return 1;
  }
  return 0;
}

// #include <chrono>
// #include <cstring>
// #include <deque>
// #include <fstream>
// #include <iostream>
// #include <memory>
// #include <string>

// extern "C" {
// #include <libavcodec/avcodec.h>
// #include <libavformat/avformat.h>
// #include <libavutil/channel_layout.h>
// #include <libavutil/common.h>
// #include <libavutil/imgutils.h>
// #include <libavutil/mathematics.h>
// #include <libavutil/opt.h>
// #include <libavutil/pixfmt.h>
// #include <libavutil/samplefmt.h>
// #include <math.h>
// }

// std::chrono::milliseconds dur;
// std::chrono::system_clock::time_point beg, end;
// std::chrono::duration<double, std::milli> fp_ms;
// double total = 0;
// int frmcount = 0;

// struct AVFrameDeleter
// {
//   void operator()(AVFrame* frame)
//   {
//     av_free(frame);
//   }
// };

// class DepthDecoder
// {
//  public:
//   DepthDecoder(std::string inputfilename,
//                std::string outputfilename,
//                int decfrmqsize)
//       : inputfilename_(inputfilename)
//       , outputfilename_(outputfilename)
//       , decfrmqsize_(decfrmqsize)
//       , formatctx_(NULL)
//       , codecctx_(NULL)
//       , frame_(NULL)
//       , optionsdict_(NULL)
//       , byteread_(0)
//       , isaframe_(0)
//       , decfrmcnt_(0)
//       , eof_(false)
//       , isinit_(true)
//       , isframealloc_(false)
//       , iscodecctxalloc_(false)
//       , isformatctxalloc_(false)
//   {
//     if (decfrmqsize > 6)  // limit the number of decoded frame
//       decfrmqsize = 6;
//   }

//   ~DepthDecoder()
//   {
//     // if ((isframealloc_ == true) && (frame_ != NULL)) av_free(frame_);

//     // avformat api says
//     //  "The codecs are not opened. The stream must be closed
//     // with avformat_close_input()."
//     if ((iscodecopen == false) && (isformatctxalloc_ == true) &&
//         (formatctx_ != NULL))
//       avformat_close_input(&formatctx_);

//     if ((iscodecctxalloc_ == true) && (codecctx_ != NULL))
//       avcodec_close(codecctx_);
//   }

//   bool Initialize();

//   /**
//    * @brief      Get next depth map normalized to 0-65535 (16 bit)
//    *
//    * @return     { description_of_the_return_value }
//    */
//   const char* GetNextFrame();
//   const char* GetFrameSize();

//   /**
//    * @brief      In the current implementation, 3 depth frames are grouped
//    into
//    *             yuv444 and treated as a single "frame". Decoding a H265
//    frame
//    *             leads to 3 decoded depth frames. This function actually
//    decodes
//    *             3 depth frames at one time.
//    */
//   bool FetchNextDataFrame();
//   void PushFramesIntoQueue();
//   void CleanPacket();

//  private:
//   std::string inputfilename_;
//   std::string outputfilename_;

//   // Decoded frame queue
//   // std::deque<std::shared_ptr<uint16_t[]>> decfrmq_;
//   std::deque<uint16_t*> decfrmq_;
//   std::deque<std::unique_ptr<AVFrame, AVFrameDeleter>> rawfrmq_;
//   int decfrmqsize_;

//   // Codec related variables
//   AVFormatContext* formatctx_;
//   AVCodecContext* codecctx_;
//   AVFrame* frame_;
//   AVDictionary* optionsdict_;
//   AVCodec* codec_;
//   AVPacket pkt_;
//   int byteread_;
//   int isaframe_;
//   int decfrmcnt_;
//   int width_;
//   int height_;

//   // Key element status;
//   bool eof_;
//   bool isinit_;
//   bool isframealloc_;
//   bool iscodecctxalloc_;
//   bool isformatctxalloc_;
//   bool iscodecopen;
// };

// bool DepthDecoder::Initialize()
// {
//   avcodec_register_all();
//   av_register_all();

//   // Open input video file
//   if (avformat_open_input(&formatctx_, inputfilename_.c_str(), NULL, NULL) !=
//   0)
//   {
//     std::cout << "avformat_open_input failed: " << inputfilename_ <<
//     std::endl;
//     return false;
//   }

//   isformatctxalloc_ = true;

//   // Analyze input video stream
//   if (avformat_find_stream_info(formatctx_, NULL) < 0)
//   {
//     std::cout << "Couldn't find stream information" << std::endl;
//     return false;
//   }

//   av_dump_format(
//       formatctx_, 0, inputfilename_.c_str(), 0);  // For debugging purpose

//   codecctx_ = formatctx_->streams[0]->codec;
//   width_    = codecctx_->width;
//   height_   = codecctx_->height;

//   // Get Codec context from parsed input file
//   if ((codecctx_ = formatctx_->streams[0]->codec)->codec_id !=
//   AV_CODEC_ID_H264)
//   {
//     std::cout << "Wrong codec" << std::endl;
//     return false;
//   }
//   // codecctx_->thread_count = 16;
//   // codecctx_->thread_type  = FF_THREAD_SLICE;
//   iscodecctxalloc_ = true;

//   // Find the decoder for the video stream (must be AV_CODEC_ID_H265)
//   if ((codec_ = avcodec_find_decoder(codecctx_->codec_id)) == NULL)
//   {
//     std::cout << "Unsupported codec." << std::endl;
//     return false;
//   }

//   std::cout << "Buffer size " << formatctx_->pb->buffer_size << std::endl;
//   if (avcodec_open2(codecctx_, codec_, &optionsdict_) < 0)
//   {
//     std::cout << "Could not open codec." << std::endl;
//     return false;  // Could not open codec
//   }

//   iscodecopen = true;

//   // if ((frame_ = av_frame_alloc()) == NULL)
//   // {
//   //   std::cout << "Allocate frame failed." << std::endl;
//   //   return false;
//   // }

//   isframealloc_ = true;
//   isinit_       = true;
//   return true;
// }

// const char* DepthDecoder::GetNextFrame()
// {
//   std::fstream outfile(
//       outputfilename_.c_str(),
//       std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
//   while (eof_ != true)
//   {
//     FetchNextDataFrame();

//     while (decfrmq_.size() != 0)
//     {
//       outfile.write(reinterpret_cast<const char*>(decfrmq_.front()),
//                     width_ * height_ * 2);
//       decfrmq_.pop_front();
//       decfrmcnt_++;

//       if (decfrmcnt_ % 3 == 0) rawfrmq_.pop_front();
//     }
//   }
//   outfile.close();
// }

// void DepthDecoder::CleanPacket()
// {
//   av_packet_unref(&pkt_);
// }

// void DepthDecoder::PushFramesIntoQueue()
// {
//   rawfrmq_.push_back(std::unique_ptr<AVFrame, AVFrameDeleter>(frame_));

//   for (int comp = 0; comp < 3; comp++)
//   {
//     decfrmq_.push_back(reinterpret_cast<uint16_t*>(frame_->data[comp]));

//     // char* thiscomp = reinterpret_cast<char*>(decfrmq_.back().get());
//     // std::memcpy(
//     //     thiscomp, frame_->data[comp], frame_->linesize[comp] *
//     //     frame_->height);
//     //
//     // for (int nrows = 0; nrows < frame_->height; nrows++)
//     // {
//     //   std::memcpy(thiscomp,
//     //               frame_->data[comp] + nrows * frame_->linesize[comp],
//     //               frame_->linesize[comp]);
//     //   thiscomp += frame_->linesize[comp];
//     // }
//   }
// }

// // To be made private
// bool DepthDecoder::FetchNextDataFrame()
// {
//   beg = std::chrono::high_resolution_clock::now();

//   // No way to tell if file ended or error occurred. So, assume error will
//   not
//   // occur
//   if (av_read_frame(formatctx_, &pkt_) < 0)
//   {
//     CleanPacket();
//     eof_ = true;
//     return true;
//   }

//   if ((frame_ = av_frame_alloc()) == NULL)
//   {
//     std::cout << "Allocate frame failed." << std::endl;
//     return false;
//   }

//   byteread_ = avcodec_decode_video2(codecctx_, frame_, &isaframe_, &pkt_);
//   frmcount++;

//   if (byteread_ < 0)
//   {
//     std::cout << "decoding frame failed" << std::endl;
//     CleanPacket();
//     return false;
//   }

//   int errorcnt = 0;  // This is a weired operation in ffmpeg.
//   while ((isaframe_ != 1) && (errorcnt < 2))
//   {
//     errorcnt++;
//     pkt_.data = NULL;  // Memory leak?
//     pkt_.size = 0;
//     byteread_ = avcodec_decode_video2(codecctx_, frame_, &isaframe_, &pkt_);
//     if (byteread_ < 0)
//     {
//       std::cout << "decoding frame failed" << std::endl;
//       // CleanPacket();
//       return false;
//     }
//   }

//   if (isaframe_)
//   {
//     PushFramesIntoQueue();
//     end   = std::chrono::high_resolution_clock::now();
//     fp_ms = end - beg;
//     total += fp_ms.count();
//     return true;
//   }
//   else
//   {
//     std::cout << "got_picture_ptr is zero" << std::endl;
//     return false;
//   }

//   return false;
// }

// int main(int argc, char* argv[])
// {
//   DepthDecoder newDecoder(argv[1], argv[2], 1);
//   if (newDecoder.Initialize() != true)
//     std::cout << "Initialization failed" << std::endl;

//   newDecoder.GetNextFrame();

//   total = total / 1000.0;

//   std::cout << "fps: " << 9 / total << std::endl;
//   std::cout << frmcount << std::endl;

//   return 0;
// }
