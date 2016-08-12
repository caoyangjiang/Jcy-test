extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
}

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

int main()
{
  avcodec_register_all();
  av_register_all();
  avformat_network_init();

  AVStream* video_st;
  AVCodec* codec;
  AVFormatContext* fmtctx;
  AVCodecContext* codectx;
  AVPacket pkt;
  std::ifstream ifs;
  std::unique_ptr<uint8_t[]> buffer;
  std::string host;
  int filesize;

  host   = "rtp://192.168.0.72:12345";
  fmtctx = avformat_alloc_context();
  if (!fmtctx)
  {
    std::cout << "avformat_alloc_context failed" << std::endl;
    return -1;
  }

  fmtctx->oformat = av_guess_format("rtp", NULL, NULL);

  strncpy(fmtctx->filename, host.c_str(), host.size());

  if (avio_open(&fmtctx->pb, fmtctx->filename, AVIO_FLAG_WRITE) < 0)
  {
    std::cout << "avio open failed" << std::endl;
    return -1;
  }

  codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!codec)
  {
    std::cout << "avcodec_find_encoder failed" << std::endl;
    return -1;
  }
  video_st = avformat_new_stream(fmtctx, codec);
  codectx  = video_st->codec;
  avcodec_get_context_defaults3(codectx, codec);
  codectx->codec_id = AV_CODEC_ID_H264;
  codectx->width    = 2560;
  codectx->height   = 1600;

  if (fmtctx->oformat->flags & AVFMT_GLOBALHEADER)
  {
    codectx->flags |= CODEC_FLAG_GLOBAL_HEADER;
  }
  avcodec_open2(video_st->codec, codec, NULL);
  if (avformat_write_header(fmtctx, NULL) < 0)
  {
    std::cout << "avformat_write_header failed" << std::endl;
    return -1;
  }

  av_init_packet(&pkt);

  ifs.open("/home/hypevr/Desktop/IFrameBS.h264",
           std::ifstream::in | std::ifstream::binary);
  ifs.seekg(0, ifs.end);
  filesize = ifs.tellg();
  ifs.seekg(0, ifs.beg);
  buffer = std::unique_ptr<uint8_t[]>(new uint8_t[filesize]);
  ifs.read(reinterpret_cast<char*>(buffer.get()), filesize);
  ifs.close();
  char sdp[2048];
  av_sdp_create(&fmtctx, 1, sdp, sizeof(sdp));
  printf("%s\n", sdp);
  fflush(stdout);
  while (1)
  {
    pkt.data = buffer.get();
    pkt.size = filesize;
    if (av_interleaved_write_frame(fmtctx, &pkt) < 0)
    {
      std::cout << "av_interleaved_write_frame failed" << std::endl;
      return -1;
    }
  }

  return 0;
}