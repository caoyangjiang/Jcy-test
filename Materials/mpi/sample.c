/* program hello */
/* Adapted from mpihello.f by drs */

#include <mpi.h>
#include <stdio.h>
#include <unistd.h>
int main(int argc, char **argv)
{
  int rank;
  char hostname[256];
  int sum=0;

  MPI_Init(&argc,&argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  gethostname(hostname,255);

  printf("Hello world!  I am process number: %d on host %s\n", rank, hostname);
 
  for(size_t i=0;i<5000;i++)
  {
	for(size_t j=0;j<5000;j++)
	{
		for(size_t k=0;k<1000;k++)
		{
			sum++;
		}
	}
  }
  MPI_Finalize();

  return 0;
}
