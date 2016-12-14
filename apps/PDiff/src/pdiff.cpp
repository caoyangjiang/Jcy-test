// Copyright 2016 Caoyang Jiang

#include <fstream>
#include <iostream>

int main(int, char** argv)
{
  std::ifstream fs[2];
  size_t filesize[2];
  char byte[2];
  uint32_t count = 0;

  fs[0].open(argv[1], std::ifstream::out | std::ifstream::binary);
  fs[1].open(argv[2], std::ifstream::out | std::ifstream::binary);

  fs[0].seekg(0, fs[0].end);
  filesize[0] = fs[0].tellg();

  fs[1].seekg(0, fs[1].end);
  filesize[1] = fs[1].tellg();

  fs[0].seekg(0, fs[0].beg);
  fs[1].seekg(0, fs[1].beg);

  if (filesize[0] != filesize[1]) std::cout << "File size differs" << std::endl;
  for (size_t sz = 0; sz < filesize[0]; sz++)
  {
    fs[0].read(&byte[0], 1);
    fs[1].read(&byte[1], 1);

    if (byte[0] != byte[1])
    {
      // std::cout << "Two files differs at " << sz << std::endl;
      count++;
    }
  }

  std::cout << count << " place are different" << std::endl;
  return 0;
}
