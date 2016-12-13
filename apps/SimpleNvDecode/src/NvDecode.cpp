// Copyright @ 2016 Caoyang Jiang
#include <cuda.h>

#include <cuviddec.h>
#include <nvcuvid.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

CUcontext devctx;
CUvideoctxlock ctxlock;
void* gdevice;
std::unique_ptr<uint8_t[]> buffer;
size_t size;
int decodedframe = 0;
uint8_t* bufferptr;

// static int CUDAAPI ParseVideoData(void* pUserData,
//                                   CUVIDSOURCEDATAPACKET* pPacket)
// {
//   CUvideoparser videoparser = reinterpret_cast<CUvideoparser>(pUserData);

//   std::cout << "ParseVideoData" << std::endl;
//   pPacket->flags        = 0;
//   pPacket->payload_size = size;
//   if (pPacket->payload_size > 0)
//     pPacket->payload = reinterpret_cast<const unsigned char*>(bufferptr);
//   else
//   {
//     pPacket->payload = nullptr;
//     pPacket->flags   = 1;
//   }

//   pPacket->timestamp = 0;
//   size -= pPacket->payload_size;
//   bufferptr += pPacket->payload_size;

//   std::cout << pPacket->flags << std::endl;
//   std::cout << pPacket->payload_size << std::endl;
//   std::cout << reinterpret_cast<uint64_t>(pPacket->payload) << std::endl;
//   std::cout << pPacket->timestamp << std::endl;

//   if (cuvidParseVideoData(videoparser, pPacket) != CUDA_SUCCESS)
//   {
//     std::cout << "cuvidParseVideoData failed." << std::endl;
//     return 0;
//   }

//   return 1;
// }

// Call back function for handle video sequence
static int CUDAAPI CheckVideoSequence(void*, CUVIDEOFORMAT*)
{
  std::cout << "Check Video Sequence" << std::endl;
  return 1;
}

// Call back function for decoding frame
static int CUDAAPI DecodeFrame(void* pUserData, CUVIDPICPARAMS* pPicParams)
{
  CUvideodecoder videodecoder = reinterpret_cast<CUvideodecoder>(pUserData);

  std::cout << "DecodeFrame" << std::endl;
  std::cout << pPicParams->PicWidthInMbs * 16 << " "
            << pPicParams->FrameHeightInMbs * 16 << std::endl;
  if (CUDA_SUCCESS != cuvidDecodePicture(videodecoder, pPicParams))
  {
    std::cout << "cuvidDecodePicture failed." << std::endl;
    return 0;
  }

  return 1;
}

// Call back function for displaying frame (optional)
static int CUDAAPI HandlePictureDisplay(void* pUserData,
                                        CUVIDPARSERDISPINFO* pPicParams)
{
  CUvideodecoder videodecoder = reinterpret_cast<CUvideodecoder>(pUserData);
  CUdeviceptr mappedframe     = 0;
  unsigned int ndecodedpitch;
  CUVIDPROCPARAMS vprocparam;

  vprocparam.progressive_frame = pPicParams->progressive_frame;
  vprocparam.top_field_first   = pPicParams->top_field_first;
  vprocparam.unpaired_field    = (pPicParams->progressive_frame == 1 ||
                               pPicParams->repeat_first_field <= 1);
  vprocparam.second_field = 0;

  /* Map video frame to CUDA memory */
  if (cuvidMapVideoFrame(videodecoder,
                         pPicParams->picture_index,
                         &mappedframe,
                         &ndecodedpitch,
                         &vprocparam) != CUDA_SUCCESS)
    std::cout << "cuvidMapVideoFrame failed" << std::endl;

  {
    CCtxAutoLock lck(ctxlock);
    uint8_t* yuv;
    cuMemAllocHost(reinterpret_cast<void**>(&yuv), (2560 * 1600 * 3 / 2));
    std::cout << "pitch " << ndecodedpitch << std::endl;

    cuMemcpyDtoHAsync(yuv, mappedframe, (ndecodedpitch * 1600 * 3 / 2), 0);
    std::string filename = "output_" + std::to_string(decodedframe) + ".NV12";
    std::ofstream ofs(filename, std::ofstream::out | std::ofstream::binary);
    ofs.write(reinterpret_cast<char*>(yuv), ndecodedpitch * 1600 * 3 / 2);
    ofs.close();
    cuMemFreeHost(reinterpret_cast<void*>(yuv));
  }

  if (cuvidUnmapVideoFrame(videodecoder, mappedframe) != CUDA_SUCCESS)
    std::cout << "cuvidUnmapVideoFrame failed" << std::endl;
  decodedframe++;
  return 1;
}

int main(int, char** argv)
{
  CUvideodecoder videodecoder = nullptr;
  CUVIDDECODECREATEINFO cuviddecodecreateinfo;
  // CUVIDPICPARAMS picparam;
  CUvideoparser videoparser = nullptr;
  CUVIDPARSERPARAMS vpparam;
  // CUvideosource videosource = nullptr;
  // CUVIDSOURCEPARAMS vsparam;  // video source parameters
  CUVIDSOURCEDATAPACKET pPacket;
  std::ifstream bs;

  CUresult cuResult;
  CUdevice device;

  cuResult = cuInit(0);
  if (cuResult != CUDA_SUCCESS)
  {
    std::cout << "cuInit error: " << cuResult << std::endl;
  }

  cuResult = cuDeviceGet(&device, 0);
  if (cuResult != CUDA_SUCCESS)
  {
    std::cout << "cuDeviceGet error: " << cuResult << std::endl;
  }

  cuResult = cuCtxCreate(reinterpret_cast<CUcontext*>(&gdevice), 0, device);
  if (cuResult != CUDA_SUCCESS)
  {
    std::cout << "cuCtxCreate error: " << cuResult << std::endl;
  }

  cuResult = cuCtxPopCurrent(&devctx);
  if (cuResult != CUDA_SUCCESS)
  {
    std::cout << "cuCtxPopCurrent error: " << cuResult << std::endl;
  }

  cuvidCtxLockCreate(&ctxlock, devctx);

  {
    CCtxAutoLock lck(ctxlock);

    /* Prepare video codec create information and create the decoder */
    std::memset(&cuviddecodecreateinfo, 0, sizeof(CUVIDDECODECREATEINFO));
    cuviddecodecreateinfo.CodecType           = cudaVideoCodec_H264;
    cuviddecodecreateinfo.ulWidth             = 2560;
    cuviddecodecreateinfo.ulHeight            = 1600;
    cuviddecodecreateinfo.ulNumDecodeSurfaces = 4;
    cuviddecodecreateinfo.ChromaFormat        = cudaVideoChromaFormat_420;
    cuviddecodecreateinfo.OutputFormat        = cudaVideoSurfaceFormat_NV12;
    cuviddecodecreateinfo.DeinterlaceMode     = cudaVideoDeinterlaceMode_Weave;
    cuviddecodecreateinfo.ulTargetWidth       = 2560;
    cuviddecodecreateinfo.ulTargetHeight      = 1600;
    cuviddecodecreateinfo.ulNumOutputSurfaces = 4;
    cuviddecodecreateinfo.ulCreationFlags     = cudaVideoCreate_PreferCUVID;
    cuviddecodecreateinfo.vidLock             = ctxlock;

    if (cuvidCreateDecoder(&videodecoder, &cuviddecodecreateinfo) !=
        CUDA_SUCCESS)
    {
      std::cout << "cuvidCreateDecoder failed" << std::endl;
    }

    /* Prepare video parser parameters and create video parser */
    std::memset(&vpparam, 0, sizeof(CUVIDPARSERPARAMS));
    vpparam.CodecType              = cuviddecodecreateinfo.CodecType;
    vpparam.ulMaxNumDecodeSurfaces = cuviddecodecreateinfo.ulNumDecodeSurfaces;
    vpparam.ulMaxDisplayDelay      = 0;
    vpparam.pUserData              = videodecoder;
    vpparam.pfnSequenceCallback    = CheckVideoSequence;
    vpparam.pfnDecodePicture       = DecodeFrame;
    vpparam.pfnDisplayPicture      = HandlePictureDisplay;
    if (cuvidCreateVideoParser(&videoparser, &vpparam) != CUDA_SUCCESS)
    {
      std::cout << "cuvidCreateVideoParser failed" << std::endl;
    }
  }

  // First frame
  bs.open(argv[1], std::ifstream::in | std::ifstream::binary);
  bs.seekg(0, bs.end);
  size = bs.tellg();
  bs.seekg(0, bs.beg);
  buffer = std::unique_ptr<uint8_t[]>(new uint8_t[size + 10]);
  bs.read(reinterpret_cast<char*>(buffer.get()), size);
  buffer[size]     = 0x00;
  buffer[size + 1] = 0x00;
  buffer[size + 2] = 0x01;
  buffer[size + 3] = 0x09;
  buffer[size + 4] = 0xFF;
  buffer[size + 5] = 0x00;
  buffer[size + 6] = 0x00;
  buffer[size + 7] = 0x01;
  buffer[size + 8] = 0x09;
  buffer[size + 9] = 0xFF;
  bs.close();
  bufferptr = buffer.get();

  pPacket.flags        = 0;
  pPacket.payload_size = size + 10;
  pPacket.payload      = reinterpret_cast<const unsigned char*>(bufferptr);

  std::cout << pPacket.flags << std::endl;
  std::cout << pPacket.payload_size << std::endl;
  std::cout << reinterpret_cast<uint64_t>(pPacket.payload) << std::endl;
  std::cout << pPacket.timestamp << std::endl;

  if (cuvidParseVideoData(videoparser, &pPacket) != CUDA_SUCCESS)
  {
    std::cout << "cuvidParseVideoData failed." << std::endl;
    return 0;
  }

  // Second frame
  bs.open(argv[2], std::ifstream::in | std::ifstream::binary);
  bs.seekg(0, bs.end);
  size = bs.tellg();
  bs.seekg(0, bs.beg);
  buffer           = std::unique_ptr<uint8_t[]>(new uint8_t[size + 10]);
  buffer[size]     = 0x00;
  buffer[size + 1] = 0x00;
  buffer[size + 2] = 0x01;
  buffer[size + 3] = 0x09;
  buffer[size + 4] = 0xFF;
  buffer[size + 5] = 0x00;
  buffer[size + 6] = 0x00;
  buffer[size + 7] = 0x01;
  buffer[size + 8] = 0x09;
  buffer[size + 9] = 0xFF;
  bs.read(reinterpret_cast<char*>(buffer.get()), size);
  bs.close();
  bufferptr = buffer.get();

  pPacket.flags        = 0;
  pPacket.payload_size = size + 10;
  pPacket.payload      = reinterpret_cast<const unsigned char*>(bufferptr);
  std::cout << pPacket.flags << std::endl;
  std::cout << pPacket.payload_size << std::endl;
  std::cout << reinterpret_cast<uint64_t>(pPacket.payload) << std::endl;
  std::cout << pPacket.timestamp << std::endl;

  if (cuvidParseVideoData(videoparser, &pPacket) != CUDA_SUCCESS)
  {
    std::cout << "cuvidParseVideoData failed." << std::endl;
    return 0;
  }

  // Third frame
  bs.open(argv[3], std::ifstream::in | std::ifstream::binary);
  bs.seekg(0, bs.end);
  size = bs.tellg();
  bs.seekg(0, bs.beg);
  buffer           = std::unique_ptr<uint8_t[]>(new uint8_t[size + 10]);
  buffer[size]     = 0x00;
  buffer[size + 1] = 0x00;
  buffer[size + 2] = 0x01;
  buffer[size + 3] = 0x09;
  buffer[size + 4] = 0xFF;
  buffer[size + 5] = 0x00;
  buffer[size + 6] = 0x00;
  buffer[size + 7] = 0x01;
  buffer[size + 8] = 0x09;
  buffer[size + 9] = 0xFF;
  bs.read(reinterpret_cast<char*>(buffer.get()), size);
  bs.close();
  bufferptr = buffer.get();

  pPacket.flags        = 0;
  pPacket.payload_size = size + 10;
  pPacket.payload      = reinterpret_cast<const unsigned char*>(bufferptr);

  if (cuvidParseVideoData(videoparser, &pPacket) != CUDA_SUCCESS)
  {
    std::cout << "cuvidParseVideoData failed." << std::endl;
    return 0;
  }
  while (decodedframe != 3)
  {
  }
  if (videodecoder)
  {
    if (cuvidDestroyDecoder(videodecoder) != CUDA_SUCCESS)
      std::cout << "cuvidDestroyDecoder failed" << std::endl;

    videodecoder = NULL;
  }

  if (videoparser)
  {
    if (cuvidDestroyVideoParser(videoparser) != CUDA_SUCCESS)
      std::cout << "cuvidDestroyVideoParser failed" << std::endl;

    videoparser = NULL;
  }

  return 0;
}
