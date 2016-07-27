// Copyright @ 2016 Caoyang Jiang

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
  struct buffer_data *bd = (struct buffer_data *)opaque;
  buf_size               = FFMIN(buf_size, bd->size);
  printf("ptr:%p size:%zu\n", bd->ptr, bd->size);
  /* copy internal buffer data to buf */
  memcpy(buf, bd->ptr, buf_size);
  bd->ptr += buf_size;
  bd->size -= buf_size;
  return buf_size;
}
int main(int argc, char *argv[])
{
  avcodec_register_all();
  av_register_all();

  AVFormatContext *fmt_ctx  = NULL;
  AVIOContext *avio_ctx     = NULL;
  AVCodec *avcodec          = NULL;
  AVCodecContext *codec_ctx = NULL;
  uint8_t *buffer = NULL, *avio_ctx_buffer = NULL;
  size_t buffer_size, avio_ctx_buffer_size = 4096;
  char *input_filename  = NULL;
  int ret               = 0;
  struct buffer_data bd = {0};
  // if (argc != 2)
  // {
  //   fprintf(stderr,
  //           "usage: %s input_file\n"
  //           "API example program to show how to read from a custom buffer "
  //           "accessed through AVIOContext.\n",
  //           argv[0]);
  //   return 1;
  // }
  input_filename = argv[1];
  /* register codecs and formats and other lavf/lavc components*/

  /* slurp file content into buffer */
  ret = av_file_map(input_filename, &buffer, &buffer_size, 0, NULL);
  if (ret < 0) goto end;
  /* fill opaque structure used by the AVIOContext read callback */
  bd.ptr  = buffer;
  bd.size = buffer_size;
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
  ret         = avformat_open_input(&fmt_ctx, NULL, NULL, NULL);
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

  if (avcodec_open2(codec_ctx, avcodec, NULL) < 2)
  {
    std::cout << "Could not open codec." << std::endl;
  }
  av_dump_format(fmt_ctx, 0, input_filename, 0);
end:
  AVPacket avpkt;
  AVFrame *frame;
  int frame_count = 0;
  std::ifstream ifs(argv[2], std::ifstream::in | std::ifstream::binary);
  uint8_t inbuf[4096 + AV_INPUT_BUFFER_PADDING_SIZE];
  size_t filesize = 0;

  memset(inbuf + 4096, 0, AV_INPUT_BUFFER_PADDING_SIZE);
  av_init_packet(&avpkt);
  frame = av_frame_alloc();

  ifs.seekg(0, ifs.end);
  filesize = ifs.tellg();
  ifs.seekg(0, ifs.beg);

  ifs.read(reinterpret_cast<char *>(inbuf), 4096);

  std::cout << static_cast<uint32_t>(inbuf[2]) << " 123 "
            << static_cast<uint32_t>(inbuf[3]) << std::endl;
  avpkt.data = inbuf;
  avpkt.size = 4096;

  std::cout << fmt_ctx->streams[0]->codec->codec_id << std::endl;

  if ((avcodec = avcodec_find_decoder(AV_CODEC_ID_H264)) == NULL)
  {
    std::cout << "unsupported codec " << std::endl;
    return 1;
  }
  while (avpkt.size > 0)
  {
    if (decode_frame(codec_ctx, frame, &frame_count, &avpkt) < 0)
    {
      std::cout << "decode_frame failed" << std::endl;
      return 1;
    }
  }

  ifs.close();
  av_frame_free(&frame);
  avformat_close_input(&fmt_ctx);
  /* note: the internal buffer could have changed, and be != avio_ctx_buffer */
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