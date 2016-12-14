// Copyright @ 2016 Caoyang Jiang

#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
}

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
  std::unique_ptr<uint8_t[]> buffer, buffer2;
  std::string host;
  int filesize, filesize2;
  std::chrono::system_clock::time_point beg, end;
  std::chrono::duration<double, std::milli> dur;
  double totaltime = 0;
  int framecount   = 0;

  host   = "rtp://192.168.0.72:5004";  // VLC must use 5004
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

  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec)
  {
    std::cout << "avcodec_find_encoder failed" << std::endl;
    return -1;
  }
  video_st = avformat_new_stream(fmtctx, NULL);
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

  ifs.open("/home/hypevr/Desktop/PFrameBS.h264",
           std::ifstream::in | std::ifstream::binary);
  ifs.seekg(0, ifs.end);
  filesize2 = ifs.tellg();
  ifs.seekg(0, ifs.beg);
  buffer2 = std::unique_ptr<uint8_t[]>(new uint8_t[filesize2]);
  ifs.read(reinterpret_cast<char*>(buffer2.get()), filesize2);
  ifs.close();

  char sdp[2048];
  av_sdp_create(&fmtctx, 1, sdp, sizeof(sdp));
  printf("%s\n", sdp);
  fflush(stdout);

  while (1)
  {
    beg = std::chrono::high_resolution_clock::now();

    pkt.data = buffer.get();
    pkt.size = filesize;
    if (av_interleaved_write_frame(fmtctx, &pkt) < 0)
    {
      std::cout << "av_interleaved_write_frame failed" << std::endl;
      return -1;
    }

    usleep(1000);

    pkt.data = buffer2.get();
    pkt.size = filesize2;
    if (av_interleaved_write_frame(fmtctx, &pkt) < 0)
    {
      std::cout << "av_interleaved_write_frame failed" << std::endl;
      return -1;
    }

    usleep(1000);

    end = std::chrono::high_resolution_clock::now();
    dur = end - beg;
    totaltime += dur.count();
    framecount += 2;
    std::cout << "FPS: " << 1000.0 * framecount / totaltime << '\r'
              << std::flush;
  }

  return 0;
}