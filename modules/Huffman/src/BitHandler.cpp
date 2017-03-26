// Copyright 2015 Caoyang Jiang

#include "Jcy/Huffman/BitHandler.h"

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <sstream>
#include <string>
JCY_WINDOWS_ENABLE_ALL_WARNING

#include "Jcy/Huffman/Contract.h"

namespace jcy
{
BitHandler::BitHandler()
    : BITS_IN_BYTE(8)
    , mchByteBuffer(0)
    , mchNextAvailablePos(BITS_IN_BYTE - 1)
    , mcAsciiBuffer()
    , mcBinaryBuffer()
{
  Contract::Require(mcBinaryBuffer, mcBinaryBuffer.size() == 0);
  Contract::Require(mcAsciiBuffer, mcAsciiBuffer.size() == 0);
  Contract::Invariant(ClassInvariant());
}

BitHandler::~BitHandler()
{
}

bool BitHandler::ClassInvariant() const
{
  bool expr0 = (mchNextAvailablePos <= 7) && (mchNextAvailablePos >= 0);
  bool expr1 = BITS_IN_BYTE == 8;
  return expr0 && expr1;
}

size_t BitHandler::AppendToBinaryBuffer(const std::string &rcAscii)
{
  Contract::Require(rcAscii, rcAscii.size() != 0);

  for (auto c : rcAscii)
  {
    //!< Write from MSBit to LSBit
    mchByteBuffer |= AsciiToBinary(c) << (mchNextAvailablePos--);

    //!< Push byte containing 8 bits to buffer
    if (mchNextAvailablePos == -1)
    {
      mcBinaryBuffer.push_back(mchByteBuffer);
      ResetByteBuffer();
      ResetNextAvailabPos();
    }
  }

  Contract::Invariant(ClassInvariant());

  return mcBinaryBuffer.size();  //! For unit-test
}

size_t BitHandler::WriteBinaryBufferToFile(std::ostream &rcBitFile) const
{
  rcBitFile.write(mcBinaryBuffer.c_str(), mcBinaryBuffer.size());
  rcBitFile << mchByteBuffer;        // writing remainig bits in the last byte
  rcBitFile << mchNextAvailablePos;  // writing number of remain bits in last
                                     // byte.

  return (size_t)rcBitFile.tellp();
}

std::string BitHandler::RemoveNBitsFromAsciiBuffer(unsigned int uiNumBits)
{
  Contract::Require(uiNumBits, uiNumBits >= 0);
  std::string cAscii;
  if (mcAsciiBuffer.size() < uiNumBits)
  {
    cAscii.assign(mcAsciiBuffer);
    mcAsciiBuffer.erase(0, mcAsciiBuffer.size());
  }
  else
  {  // return what is left in the buffer (may be zero)
    cAscii.assign(mcAsciiBuffer, 0, uiNumBits);
    mcAsciiBuffer.erase(0, uiNumBits);
  }  // Remove requested number of bits
  Contract::Invariant(ClassInvariant());
  Contract::Ensure(cAscii, cAscii.size() == uiNumBits);
  return cAscii;
}

void BitHandler::ReadFileToAsciiBuffer(std::istream &rcBitFile)
{
  std::stringstream cBitFileSsBuffer;
  std::string cBinaryBuffer;
  char chRemByte;
  char chFirstInvalidBit;
  char chAsciiBit;

  //!< create a sstream to buffer file content
  cBitFileSsBuffer << rcBitFile.rdbuf();

  //!< move sstream to string
  cBinaryBuffer = cBitFileSsBuffer.str();

  /**
   * The last byte in the file indicates number of
   * leftover bits. This leaves two bytes to be processed
   * later.
   */

  //!< convert all 'bytable' bits to ASCII representation
  while (cBinaryBuffer.size() > 2)
  {
    chRemByte = cBinaryBuffer.front();

    for (int ibitpos = BITS_IN_BYTE - 1; ibitpos >= 0; ibitpos--)
    {
      chAsciiBit = BinaryToAscii((chRemByte >> ibitpos) & 0x01);
      mcAsciiBuffer.push_back(chAsciiBit);
    }

    //!< remove front character
    cBinaryBuffer.erase(cBinaryBuffer.begin());
  }

  //!< Take care remaining bits
  chRemByte         = cBinaryBuffer[0];
  chFirstInvalidBit = cBinaryBuffer[1];

  for (int ibitpos = BITS_IN_BYTE - 1; ibitpos > chFirstInvalidBit; ibitpos--)
  {
    chAsciiBit = BinaryToAscii((chRemByte >> ibitpos) & 0x01);
    mcAsciiBuffer.push_back(chAsciiBit);
  }

  Contract::Invariant(ClassInvariant());
}

//!< Ascii '0'/'1' to binary 0/1
char BitHandler::AsciiToBinary(const char c)
{
  Contract::Require(c, c == '0' || c == '1');
  return c - '0';
}

//!< Binary '0/1 to Ascii '0'/'1'
char BitHandler::BinaryToAscii(const char b)
{
  Contract::Require(b, b == 0x00 || b == 0x01);
  return b + '0';
}

void BitHandler::ResetByteBuffer()
{
  mchByteBuffer = 0;
  Contract::Invariant(ClassInvariant());
}

void BitHandler::ResetNextAvailabPos()
{
  mchNextAvailablePos = BITS_IN_BYTE - 1;
  Contract::Invariant(ClassInvariant());
}

}  // namespace  jcy
