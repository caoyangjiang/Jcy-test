// Copyright @ 2016 Caoyang Jiang

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <stdint.h>
#include <x264.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
JCY_WINDOWS_ENABLE_ALL_WARNING

class VideoEncoder
{
 public:
  bool Initialize(int width, int height, int gopsize, int framerate)
  {
    width_      = width;
    height_     = height;
    framerate_  = framerate;
    gopsize_    = gopsize;
    lplanesize_ = width * height;
    cplanesize_ = width * height / 4;
    uplaneos_   = width * height;
    vplaneos_   = width * height * 5 / 4;
    bs_.reserve(1000000);

    x264_param_default_preset(&param_, "medium", "zerolatency");
    param_.i_csp               = X264_CSP_I420;
    param_.i_width             = width;
    param_.i_height            = height;
    param_.i_bframe            = 0;
    param_.i_fps_num           = static_cast<uint32_t>(framerate);
    param_.i_fps_den           = 1;
    param_.b_vfr_input         = 0;
    param_.i_keyint_max        = 100;
    param_.i_frame_reference   = 8;
    param_.b_full_recon        = 1;
    param_.b_cabac             = 1;
    param_.b_deblocking_filter = 0;

    // param_.i_level_idc          = 51;
    param_.rc.i_rc_method   = X264_RC_CQP;
    param_.rc.i_qp_constant = 24;
    // param.rc.i_qp_min          = 24;
    // param.rc.i_qp_max          = 24;
    // param.rc.i_bitrate         = 1500;

    // Muxing params
    param_.b_annexb = 1;
    param_.b_aud    = 1;

    // Analyze
    param_.analyse.i_weighted_pred = 0;
    param_.analyse.b_fast_pskip    = 1;

    x264_param_apply_profile(&param_, "high");
    x264_picture_alloc(&picin_, X264_CSP_I420, width, height);

    encoder_ = x264_encoder_open(&param_);

    if (!encoder_)
    {
      std::cout << "Create encoder failed." << std::endl;
      return false;
    }

    return true;
  }

  void Destroy()
  {
    x264_picture_clean(&picin_);

    if (encoder_)
    {
      x264_encoder_close(encoder_);
      encoder_ = nullptr;
    }
  }

  bool EncodeAFrame(const uint8_t* buffer)
  {
    std::memcpy(picin_.img.plane[0], buffer, static_cast<size_t>(lplanesize_));
    std::memcpy(picin_.img.plane[1],
                buffer + uplaneos_,
                static_cast<size_t>(cplanesize_));
    std::memcpy(picin_.img.plane[2],
                buffer + vplaneos_,
                static_cast<size_t>(cplanesize_));

    int nalcnt   = 0;
    int nalbytes = 0;

    if ((framecnt_ % gopsize_) == 0)
    {
      picin_.i_type = X264_TYPE_IDR;
    }
    else
    {
      picin_.i_type = X264_TYPE_P;
    }

    nalbytes = x264_encoder_encode(encoder_, &nal_, &nalcnt, &picin_, &picout_);
    if (nalbytes < 0)
    {
      std::cout << "Encode frame failed on " << framecnt_ << std::endl;
      return false;
    }

    bs_.clear();

    for (int inal = 0; inal < nalcnt; inal++)
    {
      for (int b = 0; b < nal_[inal].i_payload; b++)
      {
        bs_.push_back(nal_[inal].p_payload[b]);
      }
    }

    framecnt_++;
    return true;
  }

  void GetBitStream(const uint8_t*& bitstream, size_t& bssize)
  {
    bitstream = bs_.data();
    bssize    = bs_.size();
  }

 private:
  int width_      = 0;
  int height_     = 0;
  int framerate_  = 0;
  int gopsize_    = 0;
  int framecnt_   = 0;
  int lplanesize_ = 0;
  int cplanesize_ = 0;
  int uplaneos_   = 0;
  int vplaneos_   = 0;

  x264_param_t param_;
  x264_t* encoder_;
  x264_picture_t picin_;
  x264_picture_t picout_;
  x264_picture_t picout;
  x264_nal_t* nal_;
  std::vector<uint8_t> bs_;
};

int main(int argc, char* argv[])
{
  VideoEncoder encoder;
  std::ifstream ifs;
  std::ofstream ofs;
  int framecnt = 0;
  int width    = 2048;
  int height   = 4096;
  std::unique_ptr<uint8_t[]> buf =
      std::make_unique<uint8_t[]>(static_cast<size_t>(width * height * 3 / 2));

  if (argc < 3)
  {
    std::cout << "SimpleX264 Input.yuv #frame" << std::endl;
    return -1;
  }

  framecnt = std::atoi(argv[2]);

  if (!encoder.Initialize(width, height, 100, 45))
  {
    return -1;
  }

  ifs.open(argv[1], std::ios::in | std::ios::binary);
  ofs.open("output.h264", std::ios::out | std::ios::binary | std::ios::trunc);

  for (int i = 0; i < framecnt; i++)
  {
    size_t size;
    const uint8_t* bs;
    ifs.read(reinterpret_cast<char*>(buf.get()), width * height * 3 / 2);
    encoder.EncodeAFrame(buf.get());
    encoder.GetBitStream(bs, size);
    ofs.write(reinterpret_cast<const char*>(bs),
              static_cast<std::streamsize>(size));
  }

  ofs.close();
  ifs.close();
  return 0;
}

// int main(int argc, char* argv[])
// {
//   x264_param_t param;
//   x264_t* h = NULL;
//   x264_picture_t pic_in;
//   x264_picture_t pic_out;
//   x264_nal_t* nal;
//   uint8_t* data    = NULL;
//   int widthXheight = width * height;
//   int frame_size   = width * height * 1.5;
//   int read_sum = 0, write_sum = 0;
//   int frames = 0;
//   int i, rnum, i_size;
//   x264_nal_t* pNals = NULL;

//   x264_param_default(&param);
//   param.i_width              = width;
//   param.i_height             = height;
//   param.i_bframe             = 3;
//   param.i_fps_num            = 25;
//   param.i_fps_den            = 1;
//   param.b_vfr_input          = 0;
//   param.i_keyint_max         = 250;
//   param.rc.i_bitrate         = 1500;
//   param.i_scenecut_threshold = 40;
//   param.i_level_idc          = 51;

//   //  param.rc.i_qp_min = 20;
//   //  param.rc.i_qp_max = 24;
//   //  param.rc.i_rc_method = 0;

//   x264_param_apply_profile(&param, "high");

//   h = x264_encoder_open(&param);

//   printf("param:%s\n", x264_param2string(&param, 1));

//   x264_picture_init(&pic_in);
//   x264_picture_alloc(&pic_in, X264_CSP_YV12, width, height);
//   pic_in.img.i_csp   = X264_CSP_YV12;
//   pic_in.img.i_plane = 3;

//   data = (uint8_t*)malloc(0x400000);

//   FILE* fpr  = fopen(MFILE ".yuv", "rb");
//   FILE* fpw1 = fopen(MFILE ".szhu.h264", "wb");
//   //    FILE* fpw2 = fopen(MFILE".h264", "wb");

//   if (!fpr || !fpw1)
//   {
//     printf("file open failed\n");
//     return -1;
//   }

//   while (!feof(fpr))
//   {
//     rnum = fread(data, 1, frame_size, fpr);
//     if (rnum != frame_size)
//     {
//       printf("read file failed\n");
//       break;
//     }
//     memcpy(pic_in.img.plane[0], data, widthXheight);
//     memcpy(pic_in.img.plane[1], data + widthXheight, widthXheight >> 2);
//     memcpy(pic_in.img.plane[2],
//            data + widthXheight + (widthXheight >> 2),
//            widthXheight >> 2);
//     read_sum += rnum;
//     frames++;
//     //        printf("read frames=%d %.2fMB write:%.2fMB\n", frames, read_sum
//     *
//     //        1.0 / 0x100000, write_sum * 1.0 / 0x100000);
//     int i_nal;
//     int i_frame_size = 0;

//     if (0 && frames % 12 == 0)
//     {
//       pic_in.i_type = X264_TYPE_I;
//     }
//     else
//     {
//       pic_in.i_type = X264_TYPE_AUTO;
//     }
//     i_frame_size = x264_encoder_encode(h, &nal, &i_nal, &pic_in, &pic_out);

//     if (i_frame_size <= 0)
//     {
//       // printf("\t!!!FAILED encode frame \n");
//     }
//     else
//     {
//       fwrite(nal[0].p_payload, 1, i_frame_size, fpw1);
//       //            printf("\t+++i_frame_size=%d\n", i_frame_size);
//       write_sum += i_frame_size;
//     }
//   }

//   free(data);
//   x264_picture_clean(&pic_in);
//   x264_picture_clean(&pic_out);
//   if (h)
//   {
//     x264_encoder_close(h);
//     h = NULL;
//   }
//   fclose(fpw1);
//   //    fclose(fpw2);
//   fclose(fpr);
//   printf("h=0x%X", h);
//   return 0;
// }
