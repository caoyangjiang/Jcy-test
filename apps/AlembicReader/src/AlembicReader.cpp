// Copyright @ 2016 Caoyang Jiang

// #include <Alembic/AbcCoreFactory/IFactory.h>
#include <Alembic/Abc/IObject.h>
#include <Alembic/Abc/ISampleSelector.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/Util/All.h>

#include <array>
#include <string>
#include <vector>
class AlembicReader
{
 public:
  AlembicReader()
  {
  }

  bool Open(const std::string& filename)
  {
    Alembic::AbcCoreOgawa::ALEMBIC_VERSION_NS::ReadArchive reader;
    readerptr_    = reader(filename);
    objreaderptr_ = readerptr_->getTop();

    return true;
  }

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

    facecount = static_cast<int>(vsample_.getFaceCounts()->size());
    verbuffersize =
        static_cast<int>(vsample_.getPositions()->size()) * sizeof(float) * 3;
    idxbuffersize =
        static_cast<int>(vsample_.getFaceIndices()->size()) * sizeof(float) * 3;
    idxbuffer = vsample_.getFaceIndices()->get();
    verbuffer = vsample_.getPositions()->get()->getValue();

    // Get bounding box
    Alembic::Abc::ALEMBIC_VERSION_NS::Box3d box = vsample_.getSelfBounds();

    if (dim != nullptr)
    {
      (*dim)[0] = box.center()[0];
      (*dim)[1] = box.center()[1];
      (*dim)[2] = box.center()[2];
    }

    if (center != nullptr)
    {
      (*center)[0] = box.size()[0];
      (*center)[0] = box.size()[1];
      (*center)[0] = box.size()[2];
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
      uvbuffer = nullptr;
    }

    return true;
  }

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

  int GetNumOfFrames(int geometryid) const
  {
    Alembic::Abc::ALEMBIC_VERSION_NS::IObject topobject(readerptr_->getTop());
    Alembic::AbcGeom::IPolyMeshSchema meshobject(
        topobject.getChild(geometryid).getChild(0).getProperties());
    return static_cast<int>(meshobject.getNumSamples());
  }

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
  int vercount;
  int idxcount;
  int facecount;

  if (!reader.GetGeometry(
          0, 0, idxbuffer, vbuffer, uvbuffer, vercount, idxcount, facecount))
  {
    std::cout << "GetGeometry failed" << std::endl;
    return false;
  }

  std::cout << vbuffer[0] << std::endl;
  std::cout << vbuffer[1] << std::endl;
  std::cout << vbuffer[2] << std::endl;
  std::cout << "Vercount: " << vercount << std::endl;
  std::cout << "Idxcount: " << idxcount << std::endl;
  std::cout << "facecount: " << facecount << std::endl;

  return 0;
}

// int main(int argc, char** argv)
// {
//   if (argc < 2) return -1;

//   Alembic::AbcCoreOgawa::ALEMBIC_VERSION_NS::ReadArchive reader;
//   Alembic::AbcCoreAbstract::ArchiveReaderPtr readerptr =
//       reader(std::string(argv[1]));
//   Alembic::AbcCoreAbstract::ObjectReaderPtr objreaderptr =
//   readerptr->getTop();

//   Alembic::Abc::ALEMBIC_VERSION_NS::IObject topobject(readerptr->getTop());
//   std::cout << readerptr->getNumTimeSamplings() << std::endl;
//   std::cout << readerptr->getMaxNumSamplesForTimeSamplingIndex(0) <<
//   std::endl;
//   std::cout << readerptr->getMaxNumSamplesForTimeSamplingIndex(1) <<
//   std::endl;
//   std::cout << topobject.getName() << std::endl;
//   std::cout << topobject.getNumChildren() << std::endl;
//   std::cout << topobject.getChild(0).getName() << std::endl;
//   std::cout << topobject.getChild(0).getNumChildren() << std::endl;

//   Alembic::AbcGeom::IPolyMeshSchema meshobject(
//       topobject.getChild(0).getChild(0).getProperties());
//   std::cout << meshobject.getNumSamples() << std::endl;
//   int64_t idx = 119;
//   Alembic::Abc::ALEMBIC_VERSION_NS::ISampleSelector selector(idx);
//   Alembic::AbcGeom::IPolyMeshSchema::Sample sample;

//   meshobject.get(sample, selector);
//   Alembic::Abc::ALEMBIC_VERSION_NS::Box3d box = sample.getSelfBounds();
//   std::cout << sample.getFaceCounts()->size() << std::endl;
//   std::cout << sample.getFaceIndices()->size() << std::endl;
//   std::cout << box.size()[0] << std::endl;
//   std::cout << box.size()[1] << std::endl;
//   std::cout << box.size()[2] << std::endl;

//   std::cout << box.center()[0] << std::endl;
//   std::cout << box.center()[1] << std::endl;
//   std::cout << box.center()[2] << std::endl;
//   std::cout << sample.getPositions()->size() << std::endl;
//   // std::cout << sample.getVelocities()->size() << std::endl;
//   std::cout << sample.getPositions()->get()[0] << std::endl;
//   std::cout << sample.getPositions()->get()->getValue()[0] << std::endl;
//   std::cout << sample.getFaceCounts()->get()[0] << std::endl;
//   std::cout << sample.getFaceCounts()->get()[1] << std::endl;
//   std::cout << sample.getFaceCounts()->get()[2] << std::endl;
//   std::cout << sample.getFaceCounts()->get()[50] << std::endl;
//   std::cout << sample.getFaceIndices()->get()[0] << std::endl;
//   std::cout << sample.getFaceIndices()->get()[1] << std::endl;
//   std::cout << sample.getFaceIndices()->get()[2] << std::endl;
//   std::cout << sample.getFaceIndices()->get()[3] << std::endl;
//   std::cout << sample.getFaceIndices()->get()[4] << std::endl;
//   std::cout << sample.getFaceIndices()->get()[5] << std::endl;
//   return 0;
// }
