// Copyright @ 2016 Caoyang Jiang
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "pcl/io/obj_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include "thrust/device_vector.h"

#include "PCLKernel.cuh"

size_t curpointidx;

template <class T>
struct CompS2L
{
  bool operator()(T i, T j)
  {
    return (i < j);
  }
};

CompS2L<uint32_t> CompS2LU32;

template <class T>
std::ostream& operator<<(std::ostream& os,
                         const std::vector<std::vector<T>>& mat)
{
  for (size_t i = 0; i < mat.size(); i++)
  {
    for (size_t j = 0; j < mat[i].size(); j++)
    {
      os << mat[i][j] << " ";
    }
    os << std::endl;
  }

  return os;
}

struct RGB
{
  RGB()
  {
    r = 0;
    g = 0;
    b = 0;
  }

  RGB(uint8_t rr, uint8_t gg, uint8_t bb)
  {
    r = rr;
    g = gg;
    b = bb;
  }
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct Surface
{
  uint32_t idx[3];
};

int main(int argc, char** argv)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  if (argc < 2)
  {
    std::cout << "not enough argument" << std::endl;
    return -1;
  }

  pcl::io::loadPLYFile(argv[1], mesh);
  pcl::io::loadPLYFile(argv[1], *rgbcloud);
  std::queue<uint32_t> vqueue;
  std::vector<std::vector<uint32_t>> connectivity;
  std::vector<bool> visited;
  std::vector<std::vector<uint32_t>> gpupolygon;

  std::chrono::system_clock::time_point beg;
  std::chrono::system_clock::time_point end;
  std::chrono::duration<double, std::milli> tms;
  std::vector<double> timerecord;

  std::cout << "# of polygons " << mesh.polygons.size() << std::endl;
  std::cout << "# of points " << rgbcloud->points.size() << std::endl;

  // for (size_t sz = 0; sz < 5; sz++)
  // {
  //   pcl::Vertices v;
  //   v = mesh.polygons[sz];
  //   pcl::PointXYZRGB p[3];

  //   p[0] = rgbcloud->points[v.vertices[0]];
  //   p[1] = rgbcloud->points[v.vertices[1]];
  //   p[2] = rgbcloud->points[v.vertices[2]];

  //   std::cout << "[Triangle #" << sz << "] " << std::endl;
  //   std::cout << "      [Vertex #1] " << v.vertices[0] << " " << p[0].x <<
  //   ","
  //             << p[0].y << "," << p[0].z << "," <<
  //             static_cast<uint32_t>(p[0].r)
  //             << "," << static_cast<uint32_t>(p[0].g) << ","
  //             << static_cast<uint32_t>(p[0].b) << std::endl;
  //   std::cout << "      [Vertex #1] " << v.vertices[1] << " " << p[1].x <<
  //   ","
  //             << p[1].y << "," << p[1].z << "," <<
  //             static_cast<uint32_t>(p[1].r)
  //             << "," << static_cast<uint32_t>(p[1].g) << ","
  //             << static_cast<uint32_t>(p[1].b) << std::endl;
  //   std::cout << "      [Vertex #2] " << v.vertices[2] << " " << p[2].x <<
  //   ","
  //             << p[2].y << "," << p[2].z << "," <<
  //             static_cast<uint32_t>(p[2].r)
  //             << "," << static_cast<uint32_t>(p[2].g) << ","
  //             << static_cast<uint32_t>(p[2].b) << std::endl;
  // }

  visited = std::vector<bool>(rgbcloud->points.size(), false);

  std::cout << "[Build connectivity]" << std::endl;
  beg = std::chrono::high_resolution_clock::now();

  // Build connectivity
  for (size_t np = 0; np < mesh.polygons.size(); np++)
  {
    std::vector<uint32_t> tmp;
    tmp.push_back(mesh.polygons[np].vertices[0]);
    tmp.push_back(mesh.polygons[np].vertices[1]);
    tmp.push_back(mesh.polygons[np].vertices[2]);
    gpupolygon.push_back(tmp);
  }

  FindNeighbor(rgbcloud->points.size(), gpupolygon, connectivity);

  // for (size_t sz = 0; sz < 10; sz++)
  // {
  //   std::cout << "Point" << sz << std::endl;
  //   for (size_t n = 0; n < connectivity[sz].size(); n++)
  //   {
  //     std::cout << connectivity[sz][n] << std::endl;
  //   }
  // }

  // connectivity = std::vector<std::vector<uint32_t>>(rgbcloud->points.size());

  // for (curpointidx = 0; curpointidx < 1000; curpointidx++)
  // {
  //   size_t sz = curpointidx;
  //   for (size_t polysz = 0; polysz < mesh.polygons.size(); polysz++)
  //   {
  //     uint32_t idx = static_cast<uint32_t>(sz);
  //     pcl::Vertices v;
  //     v = mesh.polygons[polysz];

  //     if (idx == v.vertices[0])
  //     {
  //       std::vector<uint32_t>::const_iterator it;
  //       it = find(
  //           connectivity[sz].begin(), connectivity[sz].end(), v.vertices[1]);

  //       if (it == connectivity[sz].end())
  //       {
  //         connectivity[sz].push_back(v.vertices[1]);
  //       }

  //       it = find(
  //           connectivity[sz].begin(), connectivity[sz].end(), v.vertices[2]);

  //       if (it == connectivity[sz].end())
  //       {
  //         connectivity[sz].push_back(v.vertices[2]);
  //       }
  //     }

  //     if (idx == v.vertices[1])
  //     {
  //       std::vector<uint32_t>::const_iterator it;
  //       it = find(
  //           connectivity[sz].begin(), connectivity[sz].end(), v.vertices[0]);

  //       if (it == connectivity[sz].end())
  //       {
  //         connectivity[sz].push_back(v.vertices[0]);
  //       }

  //       it = find(
  //           connectivity[sz].begin(), connectivity[sz].end(), v.vertices[2]);

  //       if (it == connectivity[sz].end())
  //       {
  //         connectivity[sz].push_back(v.vertices[2]);
  //       }
  //     }

  //     if (idx == v.vertices[2])
  //     {
  //       std::vector<uint32_t>::const_iterator it;
  //       it = find(
  //           connectivity[sz].begin(), connectivity[sz].end(), v.vertices[0]);

  //       if (it == connectivity[sz].end())
  //       {
  //         connectivity[sz].push_back(v.vertices[0]);
  //       }

  //       it = find(
  //           connectivity[sz].begin(), connectivity[sz].end(), v.vertices[1]);

  //       if (it == connectivity[sz].end())
  //       {
  //         connectivity[sz].push_back(v.vertices[1]);
  //       }
  //     }
  //   }

  //   //   std::sort(connectivity[sz].begin(), connectivity[sz].end(),
  //   //   CompS2LU32);
  // }
  end = std::chrono::high_resolution_clock::now();
  tms = end - beg;
  timerecord.push_back(tms.count() / 1000.0);

  // std::cout << connectivity << std::endl;

  // for (size_t sz = 0; sz < 10; sz++)
  // {
  //   std::cout << "Point" << sz << std::endl;
  //   for (size_t n = 0; n < connectivity[sz].size(); n++)
  //   {
  //     std::cout << connectivity[sz][n] << std::endl;
  //   }
  // }

  std::vector<RGB> imgpix;
  size_t max = 0;
  vqueue.push(1);
  visited[1] = true;

  std::cout << "[Propogate]" << std::endl;
  beg = std::chrono::high_resolution_clock::now();
  while (!vqueue.empty())
  {
    // Remove a vertice from queue
    uint32_t vtmp = vqueue.front();
    vqueue.pop();

    // std::cout << "Point " << vtmp << " has " << connectivity[vtmp].size()
    //           << " neighbors" << std::endl;
    if (connectivity[vtmp].size() > max) max = connectivity[vtmp].size();
    for (size_t nsz = 0; nsz < connectivity[vtmp].size(); nsz++)
    {
      uint32_t neighp = connectivity[vtmp][nsz];

      if (!visited[neighp])
      {
        visited[neighp] = true;

        imgpix.push_back(RGB(rgbcloud->points[neighp].r,
                             rgbcloud->points[neighp].g,
                             rgbcloud->points[neighp].b));

        vqueue.push(neighp);
        // std::cout << "push neigh " << neighp << std::endl;
      }
    }
  }
  std::cout << "Maximum number of neighbor is " << max << std::endl;
  end = std::chrono::high_resolution_clock::now();
  tms = end - beg;
  timerecord.push_back(tms.count() / 1000.0);

  // cv::Mat img;
  // img.create(10, 10, CV_8UC3);
  // img = cv::Scalar(0);
  std::ofstream imgfs;

  imgfs.open("trial2.rgb", std::ofstream::out | std::ofstream::trunc);
  for (size_t sz = 0; sz < imgpix.size(); sz++)
  {
    // std::cout << "Point: " << static_cast<uint32_t>(imgpix[sz].r) << ","
    //           << static_cast<uint32_t>(imgpix[sz].g) << ","
    //           << static_cast<uint32_t>(imgpix[sz].b) << std::endl;
    // img.at<cv::Vec3b>(sz / 5, sz % 5)[0] = imgpix[sz].r;
    // img.at<cv::Vec3b>(sz / 5, sz % 5)[1] = imgpix[sz].g;
    // img.at<cv::Vec3b>(sz / 5, sz % 5)[2] = imgpix[sz].b;

    imgfs.write(reinterpret_cast<char*>(&(imgpix[sz].r)), 1);
    imgfs.write(reinterpret_cast<char*>(&(imgpix[sz].g)), 1);
    imgfs.write(reinterpret_cast<char*>(&(imgpix[sz].b)), 1);
  }

  imgfs.close();

  std::cout << "[Saving point cloud]" << std::endl;
  beg = std::chrono::high_resolution_clock::now();
  std::vector<int> indices;
  for (size_t i = 0; i < rgbcloud->points.size(); i++)
  {
    indices.push_back(i);
  }

  for (size_t i = 0; i < visited.size(); i++)
  {
    if (visited[i])
    {
      rgbcloud->points[i].r = 255;
      rgbcloud->points[i].g = 0;
      rgbcloud->points[i].b = 0;
    }
  }

  pcl::io::savePLYFile(std::string("anatomyprocessed.ply"), *rgbcloud, indices);
  end = std::chrono::high_resolution_clock::now();
  tms = end - beg;
  timerecord.push_back(tms.count() / 1000.0);
  // cv::imwrite("trial.png", img);

  std::cout << "[Build connectivity uses: " << timerecord[0] << " s]"
            << std::endl;
  std::cout << "[Propogate uses: " << timerecord[1] << " s]" << std::endl;
  std::cout << "[Save to file uses: " << timerecord[2] << " s]" << std::endl;
}
