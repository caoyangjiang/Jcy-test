// Copyright 2015 Jason Juang

#include <vector>

#include "CUDATest.cuh"

int main()
{
  std::vector<int> v;

  for (int i = 0; i < 4096; i++)
  {
    v.push_back(i);
  }

  hvr::CUDATest cudatestobj;

  cudatestobj.AddOneToArray(v);

  for (int i = 0; i < 4096; i++)
  {
    if (v.at(i) == i + 1)
      continue;
    else
    {
      std::cout << "something is wrong" << std::endl;
      return -1;
    }
  }

  std::cout << "you passed the test" << std::endl;

  return 0;
}
