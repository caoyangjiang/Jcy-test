// Copyright 2016 Caoyang Jiang

#ifndef MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_CLOUDREADER_H_
#define MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_CLOUDREADER_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

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
#endif  // MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_CLOUDREADER_H_
