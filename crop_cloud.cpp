#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
// #include "crop_cloud.h"

struct point2d{
    double x;
    double y;
} ;
  
float area(point2d pt1, point2d pt2, point2d pt3){
    double x1 = pt1.x;
    double y1 = pt1.y;
    double x2 = pt2.x;
    double y2 = pt2.y;
    double x3 = pt3.x;
    double y3 = pt3.y;
   return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0);
}

bool isInsideTriangle(point2d pt1, point2d pt2, point2d pt3, point2d pt){
   float A = area (pt1,pt2,pt3);
   float A1 = area (pt,pt2,pt3);
   float A2 = area (pt1,pt,pt3);
   float A3 = area (pt1,pt2,pt);
   return (A == A1 + A2 + A3);
}

bool isInsideQuadrilateral(point2d pt1, point2d pt2, point2d pt3, point2d pt4, point2d pt){
    bool isInTri1 = isInsideTriangle(pt1,pt2,pt3,pt);
    bool isInTri2 = isInsideTriangle(pt1,pt4,pt3,pt);
    return (isInTri1 || isInTri2);
}

//take quadrilateral corner coordinates (counterclockwise or clockwise sequencially defined) to crop point cloud
auto cropShape(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const point2d pt1, const point2d pt2, const point2d pt3, const point2d pt4) {
    pcl::PointCloud<pcl::PointXYZI> cropCloud;
    int count = 0;
    cropCloud.width    = 349556;
    cropCloud.height   = 1;
    cropCloud.is_dense = false;
    cropCloud.resize (cropCloud.width * cropCloud.height);
    for (const auto& point: *cloud){
        point2d pt;
        pt.x = point.x;
        pt.y = point.y;
        if (isInsideQuadrilateral(pt1,pt2,pt3,pt4,pt) && point.z < 1.0) {
            cropCloud[count].x = point.x;
            cropCloud[count].y = point.y;
            cropCloud[count].z = point.z;
            cropCloud[count].intensity = point.intensity;
            count++;
        }
    }
    std::cerr << count << std::endl;
    return cropCloud;
}

int
main (int argc, char** argv)
{
    point2d p1;
    p1.x = std::stod(argv[1]);
    p1.y = std::stod(argv[2]);
    point2d p2;
    p2.x = std::stod(argv[3]);
    p2.y = std::stod(argv[4]);
    point2d p3;
    p3.x = std::stod(argv[5]);
    p3.y = std::stod(argv[6]);
    point2d p4;
    p4.x = std::stod(argv[7]);
    p4.y = std::stod(argv[8]);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> cloud2;

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (argv[9], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        return (-1);
    }

    cloud2 = cropShape(cloud,p1,p2,p3,p4);

    pcl::io::savePCDFileASCII ("cropped_pcd.pcd", cloud2);
    std::cerr << "Saved data points to cropped_pcd.pcd." << std::endl;

    return (0);
}
