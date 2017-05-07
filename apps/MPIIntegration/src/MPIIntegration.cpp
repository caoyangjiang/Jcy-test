// Copyright Caoyang Jiang 2017

#include <mpi.h>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <vector>

#include "Jcy/HPCTool/CudaIntegration.h"

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cout << "Not enough argument." << std::endl;
    return -1;
  }

  // Initialize the MPI environment
  MPI_Init(NULL, NULL);

  // Get the number of processes
  int world_size;
  MPI_Comm_size(MPI_COMM_WORLD, &world_size);

  // Get the rank of the process
  int world_rank;
  MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

  // Get the name of the processor
  char processor_name[MPI_MAX_PROCESSOR_NAME];
  int name_len;
  MPI_Get_processor_name(processor_name, &name_len);

  // Print off a hello world message
  std::printf(
      "Hello world from processor %s, rank %d"
      " out of %d processors\n",
      processor_name,
      world_rank,
      world_size);

  jcy::CudaIntegration integrator;

  // global parameter
  double gstart = std::atof(argv[1]);
  double gend   = std::atof(argv[2]);
  double grange = gend - gstart;

  // local setting
  size_t tcount = 128000;  // Multiple of 512
  double lstart = gstart + world_rank * grange / world_size;
  double lend   = gstart + (world_rank + 1) * grange / world_size;

  std::vector<double> num;
  std::vector<double> recv;
  num.resize(tcount);
  recv.resize(tcount);

  integrator.StartIntegration(lstart, lend, tcount, num);

  MPI_Allreduce(
      num.data(), recv.data(), num.size(), MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD);

  double sum = 0;
  for (auto psum : recv)
  {
    sum += psum;
  }

  std::cout << sum << std::endl;
  // Finalize the MPI environment.
  MPI_Finalize();
}
