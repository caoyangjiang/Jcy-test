/* program hello */
/* Adapted from mpihello.f by drs */

#include <mpi.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

int main(int argc, char **argv)
{
  int rank;
  char hostname[256];
  int sum=0;
  size_t size; 
 std::string filename;
  std::ifstream ifs;
  

  MPI_Init(&argc,&argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  gethostname(hostname,255);

  std::string name = std::to_string(rank+1);
  name.insert(name.begin(), 3 - name.size(), '0');

  filename = "/home/hypevr/Videos/Triangulation/In_FrontPNG/FrontPano" + name +".png";
 

  ifs.open(filename, std::ifstream::in | std::ifstream::binary);
  if(!ifs.is_open())
  {
	std::cout << "Open " << filename << " failed"<<std::endl;
	ifs.close();
	return -1; 
  }
 
  ifs.seekg(0,ifs.end);
  size = ifs.tellg();
  std::cout << hostname << "[aProcess " << rank << "]: " << size <<std::endl;
  ifs.close();	
  MPI_Finalize();

  return 0;
}
