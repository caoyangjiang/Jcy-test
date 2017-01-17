// Copyright @ 2016 Caoyang Jiang

#include "Jcy/MiscTools/HvrMeshContainer.h"

int main(int, char**)
{
  Jcy::HvrMeshFrameCreator creator;
  Jcy::HvrMeshSequenceCreator screator;

  std::vector<float> vertex;

  vertex.push_back(1.0);
  vertex.push_back(2.0);
  vertex.push_back(3.0);

  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);
  creator.AddVertex(vertex);

  creator.Write("M001.hmf");
  creator.Write("M002.hmf");
  creator.Write("M003.hmf");

  screator.Merge("test.hms", "buffalo_tri_uvonly_uvonly", 1, 1);

  Jcy::HvrMeshSequenceLoader loader;

  loader.Parse("test.hms");

  std::cout << loader.GetHMS().totalframe << std::endl;
  for (int i = 0; i < static_cast<int>(loader.GetHMS().totalframe); i++)
  {
    std::vector<std::vector<float>> vbuf;
    vbuf = loader.GetVertexByFrameID<float>(i);
    std::cout << "Frame " << i << " has" << vbuf.size() << std::endl;

    for (size_t v = 0; v < vbuf.size(); v++)
    {
      for (size_t e = 0; e <  vbuf[v].size(); e++)
      {
        std::cout << vbuf[v][e] << " ";
      }
      std::cout << std::endl;
    }
  }
  return 0;
}
