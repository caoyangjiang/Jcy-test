// Copyright 2016 Caoyang Jiang

#ifndef MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_BITSTREAM_H_
#define MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_BITSTREAM_H_

HVR_WINDOWS_DISABLE_ALL_WARNING
#include <string>
#include <vector>
HVR_WINDOWS_ENABLE_ALL_WARNING

namespace Jcy
{
class BitStream
{
 public:
  enum MODE
  {
    RD = 1,
    WR = 2,
    // RDWR = 3
  };

  enum ENDIAN
  {
    LITTLE = 0,
    // BIG
  };

 public:
  HVR_WINDOWS_DLL_API explicit BitStream(enum MODE mode, enum ENDIAN endian);
  HVR_WINDOWS_DLL_API ~BitStream();

  /**
   * @brief      Reset internal bit buffer. If mode is MODE::RD, the read
   *             position is moved to the beginning of the last loaded buffer.
   *             If mode is MODE::WR, the write position is reset to the
   *             beginning of the buffer. Bit counter and bit buffer are
   *             cleared.
   */
  HVR_WINDOWS_DLL_API void Reset();

  /**
   * @brief      Writes zero.
   */
  HVR_WINDOWS_DLL_API void WriteZero();

  /**
   * @brief      Writes one.
   */
  HVR_WINDOWS_DLL_API void WriteOne();

  /**
   * @brief      Write bit(s)
   *
   * @param[in]  bits      The bit container (uint8_t, uint16_t, uint32_t, or
   *                       uint64_t)
   * @param[in]  bitcount  The bitcount
   *
   * @tparam     T         Bit container type.
   */
  HVR_WINDOWS_DLL_API void Write(const uint8_t* bits, size_t bitcount);

  /**
   * @brief      Reads a bit.
   *
   * @return     Zero or one.
   */
  HVR_WINDOWS_DLL_API uint8_t ReadBit();

  /**
   * @brief      Read bit without performing length check.
   *
   * @return     Zero or one.
   */
  HVR_WINDOWS_DLL_API uint8_t UnsafeReadBit();

  /**
   * @brief      If the number of bit left is less than requested or all bits
   *             are consumed, function returns nullptr.
   *
   * @param[in]  bitcount  The requested number of bits.
   *
   * @return     Positive if successful, else nullptr.
   */
  HVR_WINDOWS_DLL_API const uint8_t* Read(size_t bitcount);

  /**
   * @brief      Similar to Read() except this read do not perform length check.
   *
   * @param[in]  bitcount  The requested number of bits.
   *
   * @return     Positive if successful, else nullptr.
   */
  HVR_WINDOWS_DLL_API const uint8_t* UnsafeRead(size_t bitcount);

  /**
   * @brief      To avoid extra copying and allow various external bit buffer
   *             type, the buffer is in raw pointer format. Raw bit stream is
   *             not copied during bit reading. So it is up to the client to
   * make
   *             sure the bit stream is not tempered until the reading finishes.
   *
   * @param[in]  rawstream  The rawstream
   * @param[in]  sizes      The sizes
   */
  HVR_WINDOWS_DLL_API void Load(const uint8_t* bits, size_t sizes);

  /**
   * @brief      Loading bitstream with unknow length. This is unsafe and prone
   *             to segmentation fault. User is responsible for make sure the
   *             bit stream is not over read.
   *
   * @param[in]  bits  The bit stream
   */
  HVR_WINDOWS_DLL_API void Load(const uint8_t* bits);

  /**
   * @brief      Gets the bit buffer. If mode is MODE::RD, this simply returns
   *             loaded bitstream pointer. If mode is MODE::WR, this returns
   *             pointer to the written bits.
   *
   * @return     The bit buffer.
   */
  HVR_WINDOWS_DLL_API const uint8_t* GetBitBuffer() const;

  /**
   * @brief      Gets the rem size.
   *
   * @return     The rem size.
   */
  HVR_WINDOWS_DLL_API size_t GetRemSize() const;

  /**
   * @brief      Gets the written size.
   *
   * @return     The written size.
   */
  HVR_WINDOWS_DLL_API size_t GetWrittenSize() const;

 private:
  // For writing
  std::vector<uint8_t> wrbuf_;
  size_t wrpos_ = 0;  // write position in last byte

  // For reading
  std::vector<uint8_t> rdbuf_;
  const uint8_t* inbuf_ = nullptr;
  size_t rdsize_        = 0;
  size_t rdpos_         = 0;  // read position in last byte

  // Common
  enum MODE mode_;
  enum ENDIAN endian_;
  size_t bitcounter_ = 0;
};
}  // namespace Jcy
#endif  // MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_BITSTREAM_H_
