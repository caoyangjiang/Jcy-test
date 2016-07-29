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
#include <utility>
#include <vector>

class NetworkBytes
{
 public:
  NetworkBytes()
  {
  }
  ~NetworkBytes()
  {
  }

  bool Load(std::unique_ptr<uint8_t[]>& bytes, int length)
  {
    if ((length <= 0) || (bytes == nullptr)) return false;

    bytes_   = std::move(bytes);
    length_  = length;
    readpos_ = 0;
    return true;
  }

  int NumOfRemainingBytes() const
  {
    return length_ - readpos_;
  }

  bool ReadUInt8(uint8_t& val)
  {
    if (length_ < (readpos_ + 1)) return false;

    val = bytes_[readpos_];
    readpos_++;

    return true;
  }

  bool ReadUInt16(uint16_t& val)
  {
    std::array<uint16_t, 2> pos = {0, 0};

    if (length_ < (readpos_ + 2)) return false;

    pos[1] = static_cast<uint16_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint16_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt24(uint32_t& val)
  {
    std::array<uint32_t, 3> pos = {0, 0, 0};

    if (length_ < (readpos_ + 3)) return false;

    pos[2] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[2] << 16) | (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt32(uint32_t& val)
  {
    std::array<uint32_t, 4> pos = {0, 0, 0, 0};

    if (length_ < (readpos_ + 4)) return false;

    pos[3] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[2] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[3] << 24) | (pos[2] << 16) | (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt64(uint64_t& val)
  {
    std::array<uint64_t, 8> pos = {0, 0, 0, 0, 0, 0, 0, 0};

    if (length_ < (readpos_ + 8)) return false;

    pos[7] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[6] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[5] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[4] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[3] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[2] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[7] << 56) | (pos[6] << 48) | (pos[5] << 40) | (pos[4] << 32) |
          (pos[3] << 24) | (pos[2] << 16) | (pos[1] << 8) | pos[0];

    return true;
  }

  bool SeekBack(int bytes)
  {
    if (readpos_ >= bytes)
    {
      readpos_ = readpos_ - bytes;
      return true;
    }

    readpos_ = 0;
    return false;
  }

  bool SeekForward(int bytes)
  {
    if (NumOfRemainingBytes() >= bytes)
    {
      readpos_ = readpos_ + bytes;
      return true;
    }

    readpos_ = length_;
    return false;
  }

  void SeekBeg()
  {
    readpos_ = 0;
  }

  void SeekEnd()
  {
    readpos_ = length_;
  }

  const uint8_t* GetRawPtr() const
  {
    return bytes_.get();
  }

  const uint8_t* GetCurrRawPtr() const
  {
    return bytes_.get() + readpos_;
  }

 private:
  std::unique_ptr<uint8_t[]> bytes_;
  int length_  = 0;
  int readpos_ = 0;
};

class Box
{
 public:
  Box()
  {
  }

  ~Box()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    ntbytes.ReadUInt32(size_);
    ntbytes.ReadUInt32(boxtype_);

    if (size_ == 1)
    {
      std::cout << "Large size is not handled." << std::endl;
      return -1;
    }

    return 8;
  }

  uint32_t GetSize()
  {
    return size_;
  }

  uint32_t GetType()
  {
    return boxtype_;
  }

 protected:
  uint32_t size_    = 0;
  uint32_t boxtype_ = 0;
};

class FullBox : protected Box
{
 public:
  FullBox()
  {
  }
  ~FullBox()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int sz = Box::Parse(ntbytes);
    ntbytes.ReadUInt8(version_);
    ntbytes.ReadUInt24(flags_);

    return sz + 4;
  }

  uint8_t GetVersion() const
  {
    return version_;
  }

  uint32_t GetFlags() const
  {
    return flags_;
  }

 protected:
  uint8_t version_ = 0;
  uint32_t flags_  = 0;
};

class TFDT : protected FullBox
{
 public:
  TFDT()
  {
  }
  ~TFDT()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int usedbytes = FullBox::Parse(ntbytes);
    if (version_ == 0)
    {
      ntbytes.ReadUInt32(basedmediadecodetime_);
      usedbytes += 4;
    }
    else
    {
      std::cout << "Version > 0 not handled" << std::endl;
      return 0;
    }

    return usedbytes;
  }

  uint32_t GetBaseMediaDecodeTime() const
  {
    return basedmediadecodetime_;
  }

  static const uint32_t TYPE_ = 0x74666474;

 private:
  uint32_t basedmediadecodetime_ = 0;
};

class TRUN : protected FullBox
{
 public:
  TRUN()
  {
  }
  ~TRUN()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    uint32_t uint32tmp;
    int32_t int32tmp;
    int byteused = FullBox::Parse(ntbytes);

    ntbytes.ReadUInt32(samplecount_);
    byteused += 4;

    if (flags_ & 0x00000001)
    {
      ntbytes.ReadUInt32(uint32tmp);
      int32tmp = static_cast<int32_t>(uint32tmp);
      dataoffset_.push_back(int32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000004)
    {
      ntbytes.ReadUInt32(uint32tmp);
      firstsampleflags_.push_back(uint32tmp);
      byteused += 4;
    }

    for (uint32_t count = 0; count < samplecount_; count++)
    {
      if (flags_ & 0x00000100)
      {
        ntbytes.ReadUInt32(uint32tmp);
        sampleduration_.push_back(uint32tmp);
        byteused += 4;
      }

      if (flags_ & 0x00000200)
      {
        ntbytes.ReadUInt32(uint32tmp);
        samplesize_.push_back(uint32tmp);
        byteused += 4;
      }

      if (flags_ & 0x00000400)
      {
        ntbytes.ReadUInt32(uint32tmp);
        sampleflags_.push_back(uint32tmp);
        byteused += 4;
      }

      if (version_ == 0)
      {
        if (flags_ & 0x00000800)
        {
          ntbytes.ReadUInt32(uint32tmp);
          samplecompositiontimeoffset_.push_back(uint32tmp);
          byteused += 4;
        }
      }
      else
      {
        std::cout << "Version > 0 not handled" << std::endl;
        return 0;
      }
    }

    return byteused;
  }

  bool IsBaseDataOffSetAvailable() const
  {
    return !dataoffset_.empty();
  }

  bool IsFirstSampleFlagsAvailable() const
  {
    return !firstsampleflags_.empty();
  }

  bool IsSampleDurationAvailable() const
  {
    return !sampleduration_.empty();
  }

  bool IsSampleSizeAvailable() const
  {
    return !samplesize_.empty();
  }

  bool IsSampleFlagsAvailable() const
  {
    return !sampleflags_.empty();
  }

  bool IsSampleCompositionTimeOffsetAvailable() const
  {
    return !samplecompositiontimeoffset_.empty();
  }

  uint32_t GetSampleCount() const
  {
    return samplecount_;
  }

  int32_t GetBaseDataOffset() const
  {
    return dataoffset_.at(0);
  }

  uint32_t GetFirstSampleFlags() const
  {
    return firstsampleflags_.at(0);
  }

  const std::vector<uint32_t>& GetSampleDuration() const
  {
    return sampleduration_;
  }

  const std::vector<uint32_t>& GetSampleSize() const
  {
    return samplesize_;
  }

  const std::vector<uint32_t>& GetSampleFlags() const
  {
    return sampleflags_;
  }

  const std::vector<uint32_t>& GetSampleCompositionTimeOffset() const
  {
    return samplecompositiontimeoffset_;
  }

  static const uint32_t TYPE_ = 0x7472756e;

 private:
  uint32_t samplecount_ = 0;
  std::vector<int32_t> dataoffset_;
  std::vector<uint32_t> firstsampleflags_;
  std::vector<uint32_t> sampleduration_;
  std::vector<uint32_t> samplesize_;
  std::vector<uint32_t> sampleflags_;
  std::vector<uint32_t> samplecompositiontimeoffset_;
};

class TFHD : protected FullBox
{
 public:
  TFHD()
  {
  }
  ~TFHD()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int byteused = FullBox::Parse(ntbytes);
    uint32_t uint32tmp;
    uint64_t uint64tmp;

    ntbytes.ReadUInt32(trackid_);
    byteused += 4;

    if (flags_ & 0x00000001)
    {
      ntbytes.ReadUInt64(uint64tmp);
      basedataoffset_.push_back(uint64tmp);
      byteused += 8;
    }

    if (flags_ & 0x00000002)
    {
      ntbytes.ReadUInt32(uint32tmp);
      sampledescriptionindex_.push_back(uint32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000008)
    {
      ntbytes.ReadUInt32(uint32tmp);
      defaultsampleduration_.push_back(uint32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000010)
    {
      ntbytes.ReadUInt32(uint32tmp);
      defaultsamplesize_.push_back(uint32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000020)
    {
      ntbytes.ReadUInt32(uint32tmp);
      defaultsampleflags_.push_back(uint32tmp);
      byteused += 4;
    }

    return byteused;
  }

  bool IsBaseDataOffSetAvailable() const
  {
    return !basedataoffset_.empty();
  }

  bool IsSampleDescriptionIndexAvailable() const
  {
    return sampledescriptionindex_.empty();
  }

  bool IsSampleDurationAvailable() const
  {
    return !defaultsampleduration_.empty();
  }

  bool IsSampleSizeAvailable() const
  {
    return !defaultsamplesize_.empty();
  }

  bool IsSampleFlagsAvailable() const
  {
    return !defaultsampleflags_.empty();
  }

  uint32_t GetTrackId() const
  {
    return trackid_;
  }

  uint64_t GetBaseDataOffset() const
  {
    return basedataoffset_.at(0);
  }

  uint32_t GetSampleDescriptionIndex() const
  {
    return sampledescriptionindex_.at(0);
  }

  uint32_t GetSampleDuration() const
  {
    return defaultsampleduration_.at(0);
  }

  uint32_t GetSampleSize() const
  {
    return defaultsamplesize_.at(0);
  }

  uint32_t GetSampleFlags() const
  {
    return defaultsampleflags_.at(0);
  }

  static const uint32_t TYPE_ = 0x74666864;

 private:
  uint32_t trackid_;
  std::vector<uint64_t> basedataoffset_;
  std::vector<uint32_t> sampledescriptionindex_;
  std::vector<uint32_t> defaultsampleduration_;
  std::vector<uint32_t> defaultsamplesize_;
  std::vector<uint32_t> defaultsampleflags_;
};

class TRAF : protected Box
{
 public:
  TRAF()
  {
  }
  ~TRAF()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    Box box;
    int byteused = Box::Parse(ntbytes);

    while (byteused < static_cast<int>(size_))
    {
      box.Parse(ntbytes);
      ntbytes.SeekBack(8);

      if (box.GetType() == TFHD::TYPE_)
      {
        byteused += tfhd_.Parse(ntbytes);
      }
      else if (box.GetType() == TFDT::TYPE_)
      {
        TFDT tfdt;
        byteused += tfdt.Parse(ntbytes);
        tfdt_.push_back(tfdt);
      }
      else if (box.GetType() == TRUN::TYPE_)
      {
        TRUN trun;
        byteused += trun.Parse(ntbytes);
        trun_.push_back(trun);
      }
      else
      {
        std::cout << "Unknow  box inside TRAF " << std::endl;
        return 0;
      }
    }

    if (byteused != static_cast<int>(size_))
    {
      std::cout << "Parse TRAF bytes failed" << std::endl;
      return 0;
    }

    return byteused;
  }

  bool IsTFDTAvailable() const
  {
    return !tfdt_.empty();
  }

  bool IsTRUNAvailable() const
  {
    return !trun_.empty();
  }

  const TFHD& GetTFHD() const
  {
    return tfhd_;
  }

  const TFDT& GetTFDT() const
  {
    return tfdt_.at(0);
  }

  const std::vector<TRUN>& GetTRUN() const
  {
    return trun_;
  }

  static const uint32_t TYPE_ = 0x74726166;

 private:
  TFHD tfhd_;
  std::vector<TFDT> tfdt_;  // size=0: none, otherwise size=1;
  std::vector<TRUN> trun_;  // size=0: none, otherwise size>0;
};

/**
 * @brief      Movie Fragment Header Box (in Movie Fragment Box)
 */
class MFHD : protected FullBox
{
 public:
  MFHD()
  {
  }
  ~MFHD()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int sz = FullBox::Parse(ntbytes);
    ntbytes.ReadUInt32(sequencenumber_);
    return sz + 4;
  }

  uint32_t GetSequenceNumber() const
  {
    return sequencenumber_;
  }

  static const uint32_t TYPE_ = 0x6d666864;

 private:
  uint32_t sequencenumber_;
};

/**
 * @brief      Moview Fragment Box
 */
class MOOF : protected Box
{
 public:
  MOOF()
  {
  }
  ~MOOF()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int byteused = Box::Parse(ntbytes);

    while (byteused < static_cast<int>(size_))
    {
      Box box;
      box.Parse(ntbytes);
      ntbytes.SeekBack(8);

      if (box.GetType() == MFHD::TYPE_)
      {
        int sz = mfhd_.Parse(ntbytes);

        if (sz == 0) return 0;

        byteused += sz;
      }
      else if (box.GetType() == TRAF::TYPE_)
      {
        TRAF traf;
        int sz = traf.Parse(ntbytes);
        if (sz == 0) return 0;
        byteused += sz;
        traf_.push_back(traf);
      }
      else
      {
        std::cout << "Unknow box inside MOOF" << std::endl;
        return 0;
      }
    }

    if (byteused != static_cast<int>(size_))
    {
      std::cout << "Parse MOOF bytes failed" << std::endl;
      std::cout << byteused << " " << size_ << std::endl;
      return 0;
    }

    return size_;
  }

  const MFHD& GetMFHD() const
  {
    return mfhd_;
  }

  const std::vector<TRAF>& GetTRAF() const
  {
    return traf_;
  }

  static const uint32_t TYPE_ = 0x6d6f6f66;

 private:
  MFHD mfhd_;
  std::vector<TRAF> traf_;
};

class MoofParser
{
 public:
  MoofParser()
  {
  }

  ~MoofParser()
  {
  }

  bool Parse(std::unique_ptr<uint8_t[]>& buffer, int buffersize)
  {
    ntbytes_.Load(buffer, buffersize);

    if (moof_.Parse(ntbytes_) == 0) return false;

    ExtractFrames();

    return true;
  }

  bool GetNextFrame(uint8_t*& packet, int& size)
  {
    if (nextavailablesampleid_ < totalsample_)
    {
      packet = samples_[nextavailablesampleid_].first;
      size   = samples_[nextavailablesampleid_].second;
      nextavailablesampleid_++;
      return true;
    }

    return false;
  }

  bool IsEnd() const
  {
    return nextavailablesampleid_ == totalsample_;
  }

  int GetTotalSample() const
  {
    return totalsample_;
  }

  int GetNextSampleId() const
  {
    return nextavailablesampleid_;
  }

  void ResetSampleId()
  {
    nextavailablesampleid_ = 0;
  }

 private:
  void ExtractFrames()
  {
    const TRUN& trun = moof_.GetTRAF().at(0).GetTRUN().at(0);
    int offset       = 0;

    // Seek to mdat
    if (moof_.GetTRAF().at(0).GetTFHD().IsBaseDataOffSetAvailable())
    {
      std::cout << "TFHD base data off set not handled" << std::endl;
    }

    if (trun.IsBaseDataOffSetAvailable())
    {
      offset = static_cast<int>(trun.GetBaseDataOffset());
    }

    ntbytes_.SeekBeg();
    ntbytes_.SeekForward(offset);
    uint8_t* basedata = const_cast<uint8_t*>(ntbytes_.GetCurrRawPtr());

    // Separate frames
    for (uint32_t i = 0; i < trun.GetSampleCount(); i++)
    {
      std::pair<uint8_t*, uint32_t> sample;

      sample.first  = basedata;
      sample.second = trun.GetSampleSize().at(i);
      basedata += sample.second;
      samples_.push_back(sample);
    }

    totalsample_           = static_cast<int>(trun.GetSampleCount());
    nextavailablesampleid_ = 0;
  }

 private:
  int totalsample_           = 0;
  int nextavailablesampleid_ = 0;
  MOOF moof_;
  std::vector<std::pair<uint8_t*, uint32_t>> samples_;
  NetworkBytes ntbytes_;
};

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
  uint8_t* ptr;
  size_t size;  ///< size left in the buffer
};

struct Multi_buffer
{
  struct buffer_data buffers[10];
  int usebufferid = 0;
};

int decode_frame(AVCodecContext* avctx,
                 AVFrame* frame,
                 int* frame_count,
                 AVPacket* pkt)
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

static int read_packet(void* opaque, uint8_t* buf, int buf_size)
{
  struct buffer_data* bd = (struct buffer_data*)opaque;

  buf_size = FFMIN(buf_size, bd->size);
  printf("buf_size: %d, ptr:%p size:%zu\n", buf_size, bd->ptr, bd->size);
  /*copy internal buffer data to buf */
  memcpy(buf, bd->ptr, buf_size);
  bd->ptr += buf_size;
  bd->size -= buf_size;

  return buf_size;
}

int main(int argc, char* argv[])
{
  avcodec_register_all();
  av_register_all();
  AVPacket avpkt;
  AVFrame* frame;
  AVFormatContext* fmt_ctx  = NULL;
  AVIOContext* avio_ctx     = NULL;
  AVCodec* avcodec          = NULL;
  AVCodecContext* codec_ctx = NULL;
  uint8_t *buffer = NULL, *avio_ctx_buffer = NULL;
  size_t buffer_size, avio_ctx_buffer_size = 4096;
  char* input_filename  = NULL;
  int ret               = 0;
  struct buffer_data bd = {0};
  MoofParser parser[2];

  input_filename = argv[1];

  std::ifstream ifs(argv[2], std::ifstream::in | std::ifstream::binary);
  std::ifstream ifs2(argv[3], std::ifstream::in | std::ifstream::binary);
  std::unique_ptr<uint8_t[]> inbuf;
  std::unique_ptr<uint8_t[]> inbuf2;
  size_t filesize = 0;

  ifs.seekg(0, ifs.end);
  filesize = ifs.tellg();
  ifs.seekg(0, ifs.beg);
  inbuf = std::unique_ptr<uint8_t[]>(new uint8_t[filesize]);
  ifs.read(reinterpret_cast<char*>(inbuf.get()), filesize);
  parser[0].Parse(inbuf, filesize);

  ifs2.seekg(0, ifs2.end);
  filesize = ifs2.tellg();
  ifs2.seekg(0, ifs2.beg);
  inbuf2 = std::unique_ptr<uint8_t[]>(new uint8_t[filesize]);
  ifs2.read(reinterpret_cast<char*>(inbuf2.get()), filesize);
  parser[1].Parse(inbuf2, filesize);

  /* slurp file content into buffer */
  ret = av_file_map(input_filename, &buffer, &buffer_size, 0, NULL);
  if (ret < 0) goto end;
  /* fill opaque structure used by the AVIOContext read callback */
  bd.ptr  = buffer;
  bd.size = buffer_size;

  std::cout << "Buffer size " << buffer_size << std::endl;
  if (!(fmt_ctx = avformat_alloc_context()))
  {
    ret = AVERROR(ENOMEM);
    goto end;
  }
  avio_ctx_buffer = reinterpret_cast<uint8_t*>(av_malloc(avio_ctx_buffer_size));
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
    if (parser[0].IsEnd())
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
    else
    {
      parser[0].GetNextFrame(avpkt.data, avpkt.size);
    }
    // if (av_read_frame(fmt_ctx, &avpkt) < 0)
    // {
    //   av_packet_unref(&avpkt);
    //   avpkt.data    = NULL;
    //   avpkt.size    = 0;
    //   int tempcount = 0;
    //   // flush
    //   do
    //   {
    //     tempcount = frame_count;
    //     decode_frame(fmt_ctx->streams[0]->codec, frame, &frame_count,
    //     &avpkt);
    //   } while (frame_count != tempcount);
    //   break;
    // }

    std::cout << avpkt.size << std::endl;

    if (decode_frame(fmt_ctx->streams[0]->codec, frame, &frame_count, &avpkt) <
        0)
    {
      std::cout << "decode_frame failed" << std::endl;
      break;
    }
  }

  while (true)
  {
    if (parser[1].IsEnd())
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
    else
    {
      parser[1].GetNextFrame(avpkt.data, avpkt.size);
    }
    // if (av_read_frame(fmt_ctx, &avpkt) < 0)
    // {
    //   av_packet_unref(&avpkt);
    //   avpkt.data    = NULL;
    //   avpkt.size    = 0;
    //   int tempcount = 0;
    //   // flush
    //   do
    //   {
    //     tempcount = frame_count;
    //     decode_frame(fmt_ctx->streams[0]->codec, frame, &frame_count,
    //     &avpkt);
    //   } while (frame_count != tempcount);
    //   break;
    // }

    std::cout << avpkt.size << std::endl;

    if (decode_frame(fmt_ctx->streams[0]->codec, frame, &frame_count, &avpkt) <
        0)
    {
      std::cout << "decode_frame failed" << std::endl;
      break;
    }
  }

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
