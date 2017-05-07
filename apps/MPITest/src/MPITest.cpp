// Copyright Caoyang Jiang 2017

#include <mpi.h>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <vector>
int main()
{
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

  std::vector<float> num;
  std::vector<float> rev;
  for (size_t i = 0; i < 10; i++) num.push_back(i * 1.0f);
  rev.resize(num.size());

  if (world_rank == 1)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  MPI_Allreduce(
      num.data(), rev.data(), num.size(), MPI_FLOAT, MPI_SUM, MPI_COMM_WORLD);

  float globalsum = 0;
  for (auto psum : rev)
  {
    globalsum += psum;
  }

  std::this_thread::sleep_for(std::chrono::seconds(world_rank));

  std::cout << "global sum: " << globalsum << std::endl;

  // Finalize the MPI environment.
  MPI_Finalize();
}
