// Copyright @ 2017 Caoyang Jiang

#include <openssl/sha.h>
#include <cstdio>
#include <cstring>
#include <string>

int main()
{
  std::string ibuf = "Hello, World";
  unsigned char obuf[300];

  memset(obuf, 0, 300);
  SHA256(
      reinterpret_cast<const unsigned char*>(ibuf.c_str()), ibuf.size(), obuf);

  int i;
  for (i = 0; i < 32; i++)
  {
    printf("%02x ", obuf[i]);
  }
  printf("\n");

  return 0;
}
