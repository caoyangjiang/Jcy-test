// Copyright @ 2016 Caoyang Jiang

HVR_WINDOWS_DISABLE_ALL_WARNING
#include "Jcy/PointCloudTools/CloudReader.h"
#include <string>
HVR_WINDOWS_ENABLE_ALL_WARNING

namespace Jcy
{
CloudReader::CloudReader()
{
}

CloudReader::~CloudReader()
{
}

bool CloudReader::ReadCloud(const std::string cloudfile,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const
{
  cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloudfile, *cloud) == -1)
  {
    PCL_ERROR("[ERROR]: Read file failed! \n");
    return false;
  }

  return true;
}

}  // namespace Jcy
