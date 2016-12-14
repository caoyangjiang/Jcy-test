// Copyright @ 2016 Caoyang Jiang

#include <opencv/cv.h>
#include <squish/squish.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>

int main(int, char** argv)
{
  squish::u8 pixels[16 * 4];  // 16 pixels of input
  squish::u8 block[8];        // 8 bytes of output
  cv::Mat image;
  /* write some pixel data */
  image = cv::imread(argv[1], 1);

  squish::u8* pixel_tmp = pixels;
  for (int bh = 0; bh < 4; bh++)
  {
    for (int bw = 0; bw < 4; bw++)
    {
      pixel_tmp[0] = image.at<cv::Vec3b>(bh, bw)[2];
      pixel_tmp[1] = image.at<cv::Vec3b>(bh, bw)[1];
      pixel_tmp[2] = image.at<cv::Vec3b>(bh, bw)[0];
      pixel_tmp[3] = 255;
      pixel_tmp += 4;
    }
  }
  for (int i = 0; i < 64; i += 4)
  {
    printf("%u %u %u %u\n",
           pixels[i + 0],
           pixels[i + 1],
           pixels[i + 2],
           pixels[i + 3]);
  }

  printf("\n");
  // compress the 4x4 block using DXT1 compression
  squish::Compress(pixels, block, squish::kDxt1);

  for (int i = 0; i < 8; i++)
  {
    printf("%u ", block[i]);
  }
  printf("\n");

  // decompress the 4x4 block using DXT1 compression
  squish::Decompress(pixels, block, squish::kDxt1);

  for (int i = 0; i < 64; i += 4)
  {
    printf("%u %u %u %u\n",
           pixels[i + 0],
           pixels[i + 1],
           pixels[i + 2],
           pixels[i + 3]);
  }
}
