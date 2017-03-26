// Copyright 2015 Caoyang Jiang

#ifndef MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_BITHANDLER_H_
#define MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_BITHANDLER_H_

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <fstream>
#include <string>
JCY_WINDOWS_ENABLE_ALL_WARNING

namespace jcy
{
/**
 * @brief      * BitHandler performs bit writing and reading.
 *
 *             In bit writing, the last byte indicates the number of remaining
 *             bits in the last byte of the bit stream.
 */
class BitHandler
{
 public:
  BitHandler();
  virtual ~BitHandler();

  //!< Convert bit ascii representation to binary and append to binary buffer
  size_t AppendToBinaryBuffer(const std::string &rcAcii);
  //!< Write binary buffer to output file stream
  size_t WriteBinaryBufferToFile(std::ostream &rcBitFile) const;
  //!< Convert bit binary representation to ascii and append to ascii buffer
  std::string RemoveNBitsFromAsciiBuffer(unsigned int uiNumBits);
  //!< Read bit file and convert to ascii representation
  void ReadFileToAsciiBuffer(std::istream &rcBitFile);

  // Class Invariant
  bool ClassInvariant() const;

  // private:
  char AsciiToBinary(const char c);
  char BinaryToAscii(const char b);
  void ResetByteBuffer();
  void ResetNextAvailabPos();

 private:
  const char BITS_IN_BYTE;
  char mchByteBuffer;
  char mchNextAvailablePos;    //!< Next avaliable position from MSbit
  std::string mcAsciiBuffer;   //!< buffer converted input bits
  std::string mcBinaryBuffer;  //!< buffer output bits
};
}  // namespace jcy

#endif  // MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_BITHANDLER_H_
