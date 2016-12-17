// Copyright 2016 Caoyang Jiang

#ifndef MODULES_POINTCLOUDTOOLS_INCLUDE_JCY_POINTCLOUDTOOLS_CLOUDREADER_H_
#define MODULES_POINTCLOUDTOOLS_INCLUDE_JCY_POINTCLOUDTOOLS_CLOUDREADER_H_

HVR_WINDOWS_DISABLE_ALL_WARNING
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
HVR_WINDOWS_ENABLE_ALL_WARNING

namespace Jcy
{
class CloudReader
{
 public:
  CloudReader();
  ~CloudReader();

  bool ReadCloud(const std::string cloudfile,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
};
}  // namespace Jcy
#endif  // MODULES_POINTCLOUDTOOLS_INCLUDE_JCY_POINTCLOUDTOOLS_CLOUDREADER_H_
