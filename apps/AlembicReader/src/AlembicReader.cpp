// Copyright @ 2016 Caoyang Jiang

// #include <Alembic/AbcCoreFactory/IFactory.h>
#include <Alembic/Abc/IObject.h>
#include <Alembic/Abc/ISampleSelector.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcGeom/XformSample.h>
#include <Alembic/Util/All.h>

#include <array>
#include <fstream>
#include <string>
#include <vector>

void SaveObjFile(const std::string& filename,
                 const std::string& mtlname,
                 const std::vector<float>& vb,
                 const std::vector<float>& uvb,
                 const std::vector<int>& idxb)
{
  std::ofstream vstream(
      filename,
      std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
  vstream << "mtllib " << mtlname << std::endl;

  for (size_t i = 0; i < vb.size(); i += 3)
  {
    vstream << "v"
            << " " << vb[i] << " " << vb[i + 1] << " " << vb[i + 2]
            << std::endl;
  }

  for (size_t i = 0; i < uvb.size(); i += 2)
  {
    vstream << "vt"
            << " " << uvb[i] << " " << uvb[i + 1] << " " << std::endl;
  }

  for (size_t i = 0; i < idxb.size(); i += 3)
  {
    vstream << "f"
            << " " << idxb[i] + 1 << "/" << idxb[i] + 1 << " "
            << idxb[i + 1] + 1 << "/" << idxb[i + 1] + 1 << " "
            << idxb[i + 2] + 1 << "/" << idxb[i + 2] + 1 << std::endl;
  }

  vstream.close();
}

class AlembicReader
{
 public:
  AlembicReader()
  {
  }

  /**
   * @brief      Open alembic file
   *
   * @param[in]  filename  The filename
   *
   * @return     return true if file is valid
   */
  bool Open(const std::string& filename)
  {
    Alembic::AbcCoreOgawa::ALEMBIC_VERSION_NS::ReadArchive reader;
    readerptr_ = reader(filename);

    if (readerptr_ == nullptr)
    {
      std::cout << "[ERROR]: Reading alembic file failed." << std::endl;
      return false;
    }

    objreaderptr_ = readerptr_->getTop();

    if (objreaderptr_ == nullptr)
    {
      std::cout << "[ERROR]: Getting object reader failed." << std::endl;
      return false;
    }

    return true;
  }

  /**
   * @brief      Gets the geometry by geometry ID and frame ID. To avoid extra
   *             copying, the vertex sample and uv sample are temporarily saved.
   *             However, calling GetGeoemetry() again will destroy the previous
   *             samples. Upon return, user needs to test if uv buffer pointer
   *             is nullptr to determine validity.
   *
   * @param[in]  frameid        The frame ID
   * @param[in]  geometryid     The geometry ID
   * @param      idxbuffer      The index buffer
   * @param      verbuffer      The vertex buffer
   * @param      uvbuffer       The uv buffer
   * @param      verbuffersize  The vertex buffer size
   * @param      idxbuffersize  The index buffer size
   * @param      facecount      The face count
   * @param      dim            The dimension of the bounding box
   * @param      center         The center of the bounding box
   *
   * @return     The geometry.
   */
  bool GetGeometry(int frameid,
                   int geometryid,
                   const int*& idxbuffer,
                   const float*& verbuffer,
                   const float*& uvbuffer,
                   int& verbuffersize,
                   int& idxbuffersize,
                   int& facecount,
                   std::array<float, 3>* dim    = nullptr,
                   std::array<float, 3>* center = nullptr)
  {
    Alembic::Abc::ALEMBIC_VERSION_NS::IObject topobject(readerptr_->getTop());
    Alembic::AbcGeom::IPolyMeshSchema meshobject(
        topobject.getChild(geometryid).getChild(0).getProperties());
    // Get vertex, index buffer
    Alembic::Abc::ALEMBIC_VERSION_NS::ISampleSelector selector(
        static_cast<int64_t>(frameid));

    vsample_.reset();
    meshobject.get(vsample_, selector);
    if (!vsample_.valid())
    {
      std::cout << "[ERROR]: VSample not valid for getting bound." << std::endl;
      return false;
    }

    if (vsample_.getFaceCounts()->get()[0] != 3)
    {
      std::cout << "[ERROR]: mesh face has "
                << vsample_.getFaceCounts()->get()[0]
                << " vertices. Not triangle mesh." << std::endl;
      return false;
    }

    facecount = static_cast<int>(vsample_.getFaceCounts()->size());
    verbuffersize =
        static_cast<int>(vsample_.getPositions()->size()) * sizeof(float) * 3;
    idxbuffersize =
        static_cast<int>(vsample_.getFaceIndices()->size()) * sizeof(int);
    idxbuffer = vsample_.getFaceIndices()->get();
    verbuffer = vsample_.getPositions()->get()->getValue();

    // Get bounding box
    Alembic::Abc::ALEMBIC_VERSION_NS::Box3d box = vsample_.getSelfBounds();

    std::cout << "box center " << box.center()[0] << " " << box.center()[1]
              << " " << box.center()[2] << std::endl;
    std::cout << "box size " << box.size()[0] << " " << box.size()[1] << " "
              << box.size()[2] << std::endl;

    if (dim != nullptr)
    {
      (*dim)[0] = box.center()[0];
      (*dim)[1] = box.center()[1];
      (*dim)[2] = box.center()[2];
    }

    if (center != nullptr)
    {
      (*center)[0] = box.size()[0];
      (*center)[1] = box.size()[1];
      (*center)[2] = box.size()[2];
    }

    // Get UVs
    if (meshobject.getUVsParam().valid())
    {
      Alembic::AbcGeom::ALEMBIC_VERSION_NS::IV2fGeomParam uvparam =
          meshobject.getUVsParam();

      if (uvparam.getNumSamples() != meshobject.getNumSamples())
      {
        std::cout << "[ERROR]: Number of UV frames " << uvparam.getNumSamples()
                  << " do not match with number of vertex frame "
                  << meshobject.getNumSamples() << std::endl;
        return -1;
      }

      uvsample_.reset();
      uvparam.getIndexed(uvsample_, selector);
      if (!uvsample_.valid())
      {
        std::cout << "[ERROR]: UVSample not valid for getting bound."
                  << std::endl;
        return false;
      }
      else
      {
        uvbuffer = uvsample_.getVals()->get()->getValue();
      }
    }
    else
    {
      std::cout << "[WARNING]: Mesh has no UV." << std::endl;
      uvbuffer = nullptr;
    }

    // Get transformation
    // Alembic::AbcGeom::ALEMBIC_VERSION_NS::IXformSchema xform(
    //     topobject.getChild(geometryid));
    // Alembic::AbcGeom::ALEMBIC_VERSION_NS::XformSample xformsample;
    // xform.get(xformsample, selector);
    // std::cout << xformsample.getNumOps() << std::endl;
    // std::cout << xformsample.getMatrix() << std::endl;
    return true;
  }

  /**
   * @brief      Gets the readable geometry ID
   *
   * @return     The geometry IDs.
   */
  std::vector<std::string> GetGeometryIDs() const
  {
    std::vector<std::string> geometryids;
    Alembic::Abc::ALEMBIC_VERSION_NS::IObject topobject(readerptr_->getTop());

    for (size_t i = 0; i < topobject.getNumChildren(); i++)
    {
      geometryids.push_back(std::string(topobject.getChild(i).getName()));
    }

    return geometryids;
  }

  /**
   * @brief      Query the number of frames for the geometry. User must use
   *             GetNumOfGeometries() to get the range of valid geometry IDs.
   *
   * @param[in]  geometryid  id  The geometry id
   *
   * @return     The number of frames.
   */
  int GetNumOfFrames(int geometryid) const
  {
    Alembic::Abc::ALEMBIC_VERSION_NS::IObject topobject(readerptr_->getTop());
    Alembic::AbcGeom::IPolyMeshSchema meshobject(
        topobject.getChild(geometryid).getChild(0).getProperties());
    return static_cast<int>(meshobject.getNumSamples());
  }

  /**
   * @brief      Gets the number of geometries.
   *
   * @return     The number of geometries.
   */
  int GetNumOfGeometries() const
  {
    Alembic::Abc::ALEMBIC_VERSION_NS::IObject topobject(readerptr_->getTop());
    return static_cast<int>(topobject.getNumChildren());
  }

 private:
  Alembic::AbcCoreAbstract::ArchiveReaderPtr readerptr_;
  Alembic::AbcCoreAbstract::ObjectReaderPtr objreaderptr_;
  Alembic::AbcGeom::IPolyMeshSchema::Sample vsample_;
  Alembic::AbcGeom::ALEMBIC_VERSION_NS::IV2fGeomParam::Sample uvsample_;
};

int main(int argc, char** argv)
{
  if (argc < 2) return -1;

  AlembicReader reader;
  std::vector<std::string> names;

  if (!reader.Open(argv[1]))
  {
    std::cout << "Open failed" << std::endl;
    return -1;
  }

  names = reader.GetGeometryIDs();

  for (size_t i = 0; i < names.size(); i++)
  {
    std::cout << i << "th geometry: " << names[i] << std::endl;
  }

  std::cout << "Number of geometries: " << reader.GetNumOfGeometries()
            << std::endl;

  for (int i = 0; i < reader.GetNumOfGeometries(); i++)
  {
    std::cout << i << "th geometries has " << reader.GetNumOfFrames(i)
              << " frames." << std::endl;
  }

  const int* idxbuffer;
  const float* vbuffer;
  const float* uvbuffer;

  int vsize;
  int idxsize;
  int facecount;
  std::array<float, 3> center;
  std::array<float, 3> size;
  if (!reader.GetGeometry(1000,
                          0,
                          idxbuffer,
                          vbuffer,
                          uvbuffer,
                          vsize,
                          idxsize,
                          facecount,
                          &size,
                          &center))
  {
    std::cout << "GetGeometry failed" << std::endl;
    return false;
  }

  std::cout << "Vercount: " << vsize << std::endl;
  std::cout << "Idxcount: " << idxsize << std::endl;
  std::cout << "facecount: " << facecount << std::endl;
  std::cout << size[0] << " " << size[1] << " " << size[2] << std::endl;
  std::cout << center[0] << " " << center[1] << " " << center[2] << std::endl;
  std::vector<float> uvs;
  std::vector<float> vertices;
  std::vector<int> indices;

  uvs.resize(vsize / 4);
  vertices.resize(vsize / 4);
  indices.resize(idxsize / 4);

  for (int i = 0; i < vsize / 4; i++)
  {
    vertices[i] = vbuffer[i];
  }

  for (int i = 0; i < vsize / 4; i++)
  {
    uvs[i] = uvbuffer[i];
  }

  for (int i = 0; i < idxsize / 4; i++)
  {
    indices[i] = idxbuffer[i];
  }

  SaveObjFile(
      std::string("test.obj"), std::string("test.mtl"), vertices, uvs, indices);

  return 0;
}
