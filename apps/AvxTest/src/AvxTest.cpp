// Copyright @ 2016 Caoyang Jiang

#include <immintrin.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#define N 10000000

int main()
{
  std::chrono::milliseconds dur;
  std::chrono::system_clock::time_point beg;
  std::chrono::system_clock::time_point end;
  std::chrono::duration<double, std::milli> millicnt;
  int16_t* p1;
  int16_t* p2;

  p1 = new int16_t[65536];
  p2 = new int16_t[65536];

  for (int i = 0; i < 65536; i++)
  {
    p1[i] = 255;
    p2[i] = 253;
  }

  __m256i reg1, reg2, reg3;
  __m256i reg[256];
  int16_t* p;
  int sum = 0;
  int inc = 0;

  beg = std::chrono::high_resolution_clock::now();

  for (int j = 0; j < 10000; j++)
  {
    inc = 0;
    sum = 0;
    for (int i = 0; i < 4096; i++)
    {
      reg1 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p1 + inc));
      reg2 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p2 + inc));
      // for (int i = 0; i < 256; i++)
      // {
      //   reg[i] = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p2 +
      //   inc));
      // }

      // for (int i = 1; i < 256; i++)
      // {
      //   reg[i] = _mm256_sub_epi16(reg[i], reg[i - 1]);
      // }

      reg3 = _mm256_sub_epi16(reg1, reg2);
      reg3 = _mm256_add_epi16(reg3, reg[3]);
      reg3 = _mm256_abs_epi16(reg3);

      p = reinterpret_cast<int16_t*>(&reg3);
      for (int k = 0; k < 16; k++)
      {
        sum += p[k];
      }
      inc += 16;
    }
  }
  end = std::chrono::high_resolution_clock::now();

  millicnt = end - beg;

  std::cout << " uses " << millicnt.count() / 1000.0 << std::endl;
  std::cout << sum << std::endl;
  delete[] p1;
  delete[] p2;
}

// int main()
// {
//   std::unique_ptr<int[]> array1(new int[N]);
//   std::unique_ptr<int[]> array2(new int[N]);
//   std::unique_ptr<int[]> array3(new int[N]);
//   std::unique_ptr<int[]> array4(new int[N]);

//   std::chrono::milliseconds dur;
//   std::chrono::system_clock::time_point beg;
//   std::chrono::system_clock::time_point end;
//   std::chrono::duration<double, std::milli> millicnt;

//   for (int i = 0; i < N; i++)
//   {
//     array1[i] = i;
//     array2[i] = 2;
//   }

//   beg = std::chrono::high_resolution_clock::now();

//   for (int i = 0; i < N; i++)
//   {
//     array3[i] = array1[i] * array2[i] + array2[i] * 3;
//   }

//   end = std::chrono::high_resolution_clock::now();

//   millicnt = end - beg;

//   std::cout << "Normal operation uses " << millicnt.count() / 1000.0
//             << std::endl;

//   for (int i = 0; i < N; i++)
//   {
//     array1[i] = i;
//     array2[i] = 2;
//   }

//   int count = 0;
//   __m256i vec1, vec2, vecall3;
//   vecall3 = _mm256_setr_epi32(3, 3, 3, 3, 3, 3, 3, 3);

//   beg = std::chrono::high_resolution_clock::now();
//   while (count < N)
//   {
//     vec1 = _mm256_loadu_si256(reinterpret_cast<const
//     __m256i*>(array1.get()));
//     vec2 = _mm256_loadu_si256(reinterpret_cast<const
//     __m256i*>(array2.get()));
//     // vec1 = _mm256_setr_epi32(array1[count],
//     //                          array1[count + 1],
//     //                          array1[count + 2],
//     //                          array1[count + 3],
//     //                          array1[count + 4],
//     //                          array1[count + 5],
//     //                          array1[count + 6],
//     //                          array1[count + 7]);
//     // vec2 = _mm256_setr_epi32(array2[count],
//     //                          array2[count + 1],
//     //                          array2[count + 2],
//     //                          array2[count + 3],
//     //                          array2[count + 4],
//     //                          array2[count + 5],
//     //                          array2[count + 6],
//     //                          array2[count + 7]);
//     vec1 = _mm256_mullo_epi32(vec1, vec2);
//     vec2 = _mm256_mullo_epi32(vec2, vecall3);
//     vec1 = _mm256_add_epi32(vec1, vec2);

//     int* p = reinterpret_cast<int*>(&vec1);
//     for (int i = 0; i < 8; i++)
//     {
//       array4[count + i] = p[i];
//     }

//     count += 8;
//   }

//   end = std::chrono::high_resolution_clock::now();

//   millicnt = end - beg;

//   std::cout << "AVX2 operation uses " << millicnt.count() / 1000.0 <<
//   std::endl;

//   for (int i = 0; i < N; i++)
//   {
//     if (array3[i] != array4[i])
//     {
//       std::cout << "Result not match!" << std::endl;
//       std::cout << "At " << i << std::endl;
//       std::cout << "Normal " << array3[i] << std::endl;
//       std::cout << "AVX " << array4[i] << std::endl;
//       break;
//     }
//   }
//   // __m256i vec1 = _mm256_setr_epi32(1, 2, 3, 4, 5, 6, 7, 8);
//   // __m256i vec2 = _mm256_setr_epi32(1, 2, 3, 4, 5, 6, 7, 8);

//   // __m256i vec3 = _mm256_add_epi32(vec1, vec2);
//   // int* p       = (int*)&vec3;
// }

// // double array[1] = {1.1, 2.2, 3.3, 4.4};
// // __m256d vec1    = _mm256_setr_pd(4.0, 5.0, 13.0, array[3]);
// // __m256d vec2    = _mm256_setr_pd(1.0, 2.0, 3.0, 4.0);
// // __m256d vec3    = _mm256_setr_pd(1.0, 1.0, 1.0, 1.0);

// // __m256d r = _mm256_fmadd_pd(vec1, vec3, vec2);

// // double* p = (double*)&r;

// // for (int i = 0; i < 8; i++)
// //   std::cout << std::setprecision(5) << p[i] << std::endl;
