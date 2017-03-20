// Copyright @ 2016 Caoyang Jiang

#ifndef MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_HVRMESHCONTAINER_H_
#define MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_HVRMESHCONTAINER_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace jcy
{
struct HvrMeshFrame
{
  const char kHVRMESHF[8] = {'H', 'V', 'R', 'M', 'E', 'S', 'H', 'F'};
  uint8_t datatype;  // 0 for float
  uint8_t chlayout;  // 0 for interleaved, 1 for planar
  uint8_t numch;     // number of channels
  uint32_t datasize;
  // A mesh data follows
};

struct HvrMeshSequence
{
  const char kHVRMESHS[8] = {'H', 'V', 'R', 'M', 'E', 'S', 'H', 'S'};
  uint8_t datatype;
  uint8_t chlayout;
  uint8_t numch;
  uint32_t vmax;
  uint32_t totalframe;
  // beg,end for mesh #1
  // beg,end for mesh #2
  // ...
  // Actual meshes data follow
};

class HvrMeshFrameCreator
{
 public:
  JCY_MISCTOOLS_DLL HvrMeshFrameCreator();
  JCY_MISCTOOLS_DLL ~HvrMeshFrameCreator();

  /**
   * @brief      Adds a vertex.
   *
   * @param[in]  vertex  The vertex
   */
  JCY_MISCTOOLS_DLL void AddVertex(const std::vector<float>& vertex);

  /**
   * @brief      Write vertex buffer to output. Use output file extension name
   *             ".hmf".
   *
   * @param[in]  filename  The filename
   */
  JCY_MISCTOOLS_DLL void Write(const std::string& filename);

 private:
  std::vector<std::vector<float>> vertices_;
};

class HvrMeshSequenceCreator
{
 public:
  JCY_MISCTOOLS_DLL HvrMeshSequenceCreator();
  JCY_MISCTOOLS_DLL ~HvrMeshSequenceCreator();

  /**
   * @brief      Merge all HvrMesh frames into HvrMesh sequence. Use output file
   *             extension name ".hms".
   *
   * @param[in]  outputfilename   The output file name.
   * @param[in]  inputfileprefix  The mesh file prefix (must have extension name
   *                              "hvrmeshf")
   * @param[in]  first            The first frame id.
   * @param[in]  last             The last frame id.
   */
  JCY_MISCTOOLS_DLL void Merge(const std::string& outputfilename,
                               const std::string& inputfileprefix,
                               int first,
                               int last);
};

class HvrMeshSequenceLoader
{
 public:
  JCY_MISCTOOLS_DLL HvrMeshSequenceLoader();
  JCY_MISCTOOLS_DLL ~HvrMeshSequenceLoader();

  /**
   * @brief      Parse a HvrMeshSequence file.
   *
   * @param[in]  filename  The file name
   *
   * @return     True if parsing successful, false otherwise;
   */
  JCY_MISCTOOLS_DLL bool Parse(const std::string& filename);

  template <typename T>
  JCY_MISCTOOLS_DLL std::vector<std::vector<T>> GetVertexByFrameID(int frameid);

  /**
   * @brief      Gets the HvrMeshFrame.
   *
   * @return     The HvrMeshFrame.
   */
  JCY_MISCTOOLS_DLL const HvrMeshSequence& GetHMS() const;

 private:
  HvrMeshSequence hms_;
  std::vector<int> begs_;
  std::vector<int> sizes_;
  std::ifstream ifs_;
};

}  // namespace jcy
#endif  // MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_HVRMESHCONTAINER_H_
