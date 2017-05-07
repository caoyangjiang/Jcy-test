cmake_minimum_required(VERSION 3.5)
project(apps)

add_subdirectory(AlembicReader)
add_subdirectory(ArithmeticCoder)
add_subdirectory(AvcWrapper)
add_subdirectory(BitSet)
add_subdirectory(SimpleBitStream)
add_subdirectory(ByteRemap)
add_subdirectory(DataStructure)
add_subdirectory(HvrMesh)
add_subdirectory(MatrixTest)
add_subdirectory(MeshFileParse)
add_subdirectory(MoofParser)
add_subdirectory(NetworkBytes)
add_subdirectory(PDiff)
add_subdirectory(SimpleMatrix)
add_subdirectory(SimpleAVCAnalyzer)
add_subdirectory(ThreadPool)
add_subdirectory(Cpp11)
add_subdirectory(SimpleX264)
add_subdirectory(AvxTest) # Require archiecture support
add_subdirectory(BitOp) # Require Boost
add_subdirectory(Client) # Require Boost
add_subdirectory(CurlTest) # Require CURL
add_subdirectory(DctApp) # Require Eign3
add_subdirectory(GraphTest) # Require Boost
add_subdirectory(ObjToTriangle) # Require PCL
add_subdirectory(PCLGen) # Require PCL
add_subdirectory(PCLLoad) # Require PCL
add_subdirectory(Server) # Require Boost
add_subdirectory(SimpleBoostClient) # Require Boost
add_subdirectory(SimpleBoostServer) # Require Boost
add_subdirectory(Xml2) # Require LIBXML2
add_subdirectory(SHA256) # Require libcrypto

add_subdirectory(BlockCompression) # Require Squish, OpenCV
add_subdirectory(SimpleNvEncode) # Require CUDA, NVIDIA-ENCODE
add_subdirectory(SimpleNvDecode) # Require CUDA, NVCUVID

add_subdirectory(MPITest)
add_subdirectory(MPIIntegration)
add_subdirectory(IEClient)
add_subdirectory(IEServer)
add_subdirectory(GpuMotionEstimate)
#Deprecated
#add_subdirectory(BufferDecoder) # Require FFmpeg
#add_subdirectory(SimpleRTPServer) # Require FFmpeg
#add_subdirectory(StreamDecode) # Require FFmpeg
