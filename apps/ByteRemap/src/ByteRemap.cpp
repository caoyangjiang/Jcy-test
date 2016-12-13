// Copyright @ 2016 Caoyang Jiang

#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class ByteRemap
{
 public:
  ByteRemap()
  {
  }
  ~ByteRemap()
  {
  }
  void Generate()
  {
    std::vector<int> maptable(256);
    std::vector<int> remaptable(256);

    int count         = 0;
    int num           = 0;
    bool existed      = false;
    unsigned int seed = time(NULL);
    // srand(time(NULL));

    for (size_t i = 0; i < 256; i++)
    {
      maptable[i] = -1;
    }

    while (count != 256)
    {
      num = rand_r(&seed) % 256;

      for (int i = 0; i < count; i++)
      {
        if (maptable[i] == num)
        {
          existed = true;
          break;
        }
        else
          existed = false;
      }

      if (!existed)
      {
        std::cout << num << " " << count << std::endl;
        maptable[count] = num;
        remaptable[num] = count;
        count++;
      }
    }

    std::cout << "Map table" << std::endl;
    for (size_t i = 0; i < 256; i++)
    {
      std::cout << maptable[i] << ",";
    }
    std::cout << std::endl;

    std::cout << "Remap table" << std::endl;
    for (size_t i = 0; i < 256; i++)
    {
      std::cout << remaptable[i] << ",";
    }
    std::cout << std::endl;
  }
};

class Cryptor
{
 public:
  const uint8_t MapTable[256] = {
      198, 61,  44,  154, 124, 238, 133, 65,  150, 28,  49,  122, 110, 217, 249,
      170, 235, 215, 40,  222, 90,  228, 106, 243, 157, 127, 253, 236, 205, 108,
      178, 10,  152, 77,  134, 210, 199, 151, 92,  144, 85,  125, 123, 45,  165,
      89,  135, 137, 195, 39,  66,  119, 19,  15,  227, 25,  160, 2,   229, 103,
      31,  211, 96,  182, 47,  51,  70,  132, 176, 193, 177, 27,  57,  179, 5,
      33,  43,  48,  14,  223, 74,  138, 242, 234, 141, 81,  172, 98,  168, 93,
      149, 105, 72,  200, 99,  251, 206, 38,  231, 180, 53,  254, 191, 184, 232,
      76,  143, 58,  248, 18,  209, 111, 102, 126, 80,  240, 24,  83,  194, 230,
      140, 29,  147, 221, 75,  115, 41,  218, 173, 34,  153, 37,  208, 32,  219,
      56,  156, 6,   146, 162, 128, 203, 166, 202, 185, 214, 71,  64,  116, 16,
      97,  104, 187, 130, 84,  36,  244, 113, 163, 1,   101, 62,  22,  20,  35,
      114, 100, 212, 148, 91,  88,  50,  164, 161, 237, 17,  3,   46,  131, 87,
      183, 245, 67,  192, 136, 181, 201, 155, 8,   78,  169, 145, 197, 30,  225,
      112, 246, 120, 7,   0,   188, 4,   13,  11,  12,  159, 196, 55,  60,  69,
      158, 63,  86,  247, 207, 190, 204, 121, 175, 79,  52,  9,   42,  189, 233,
      220, 213, 94,  241, 216, 174, 109, 54,  250, 139, 59,  21,  252, 129, 68,
      171, 239, 82,  26,  226, 167, 117, 23,  107, 118, 224, 95,  255, 142, 73,
      186};

  const uint8_t RemapTable[256] = {
      199, 159, 57,  176, 201, 74,  137, 198, 188, 221, 31,  203, 204, 202, 78,
      53,  149, 175, 109, 52,  163, 236, 162, 247, 116, 55,  243, 71,  9,   121,
      193, 60,  133, 75,  129, 164, 155, 131, 97,  49,  18,  126, 222, 76,  2,
      43,  177, 64,  77,  10,  171, 65,  220, 100, 232, 207, 135, 72,  107, 235,
      208, 1,   161, 211, 147, 7,   50,  182, 239, 209, 66,  146, 92,  254, 80,
      124, 105, 33,  189, 219, 114, 85,  242, 117, 154, 40,  212, 179, 170, 45,
      20,  169, 38,  89,  227, 251, 62,  150, 87,  94,  166, 160, 112, 59,  151,
      91,  22,  248, 29,  231, 12,  111, 195, 157, 165, 125, 148, 246, 249, 51,
      197, 217, 11,  42,  4,   41,  113, 25,  140, 238, 153, 178, 67,  6,   34,
      46,  184, 47,  81,  234, 120, 84,  253, 106, 39,  191, 138, 122, 168, 90,
      8,   37,  32,  130, 3,   187, 136, 24,  210, 205, 56,  173, 139, 158, 172,
      44,  142, 245, 88,  190, 15,  240, 86,  128, 230, 218, 68,  70,  30,  73,
      99,  185, 63,  180, 103, 144, 255, 152, 200, 223, 215, 102, 183, 69,  118,
      48,  206, 192, 0,   36,  93,  186, 143, 141, 216, 28,  96,  214, 132, 110,
      35,  61,  167, 226, 145, 17,  229, 13,  127, 134, 225, 123, 19,  79,  250,
      194, 244, 54,  21,  58,  119, 98,  104, 224, 83,  16,  27,  174, 5,   241,
      115, 228, 82,  23,  156, 181, 196, 213, 108, 14,  233, 95,  237, 26,  101,
      252};

  bool Encrypt(char* buf, int nbytes)
  {
    for (int isz = 0; isz < nbytes; isz++)
    {
      buf[isz] = MapTable[static_cast<uint8_t>(buf[isz])];
    }

    return true;
  }

  bool Decrypt(char* buf, int nbytes)
  {
    for (int isz = 0; isz < nbytes; isz++)
    {
      buf[isz] = RemapTable[static_cast<uint8_t>(buf[isz])];
    }

    return true;
  }
};

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "not enough argument!" << std::endl;
    return -1;
  }

  std::ifstream infs(argv[1], std::ifstream::in | std::ifstream::binary);
  std::ofstream outfs(std::string(argv[1]) + "_Encrypt",
                      std::ofstream::out | std::ofstream::binary);

  std::unique_ptr<char[]> filebuf;
  size_t filesize;
  Cryptor cyptor;

  infs.seekg(0, infs.end);
  filesize = infs.tellg();
  infs.seekg(0, infs.beg);

  filebuf = std::unique_ptr<char[]>(new char[filesize]);

  infs.read(filebuf.get(), filesize);
  cyptor.Encrypt(filebuf.get(), static_cast<int>(filesize));
  outfs.write(filebuf.get(), filesize);

  infs.close();
  outfs.close();

  return 0;
}
