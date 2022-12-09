#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/filters/crop_box.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <pcl/filters/statistical_outlier_removal.h>

const static double pixelSize = 0.3;
const static int pixelValue = 255;
const static double offset = 0.5;

//remove noise to isolate ground
auto removeNoise(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*cloud_filtered);

    std::vector< double > z_value;
    for (const auto& point: *cloud_filtered){
        z_value.push_back(point.z);
    }

    std::sort(z_value.begin(), z_value.end());
    int length = z_value.size();
    int med_z = length/2;
    double med = z_value[med_z];

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground (new pcl::PointCloud<pcl::PointXYZI>);

    for (const auto& point: *cloud_filtered){
        if ( point.z < (offset + med)) {
            cloud_ground->push_back(point);
        }
    }
    pcl::io::savePCDFileASCII ("cloud_ground.pcd", *cloud_ground);////
    return cloud_ground;
}

//project intensity value on to 2d plane
cv::Mat projectTo2d(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    //get boundary
    double x_min { 10E8 };
    double x_max { -10E8 };
    double y_min { 10E8 };
    double y_max { -10E8 };
    for (const auto& point: *cloud){
        if (point.x < x_min){
            x_min = point.x;
        }else if(point.x > x_max){
            x_max = point.x;
        }
        if (point.y < y_min){
            y_min = point.y;
        }else if(point.y > y_max){
            y_max = point.y;
        }
    }

    int height = static_cast<int>(y_max - y_min)/pixelSize + 1;
    int width = static_cast<int>(x_max - x_min)/pixelSize + 1;
    cv::Mat img = cv::Mat::zeros(cv::Size(width,height),CV_64F);
    cv::Mat alpha = cv::Mat::zeros(cv::Size(width,height),CV_64F);
    
        
    std::cerr << "x_min: " << x_min << std::endl;
    std::cerr << "y_min: " << y_min << std::endl;

    double min_intensity { 10E8 };
    double max_intensity { -10E8 };
 
    pcl::PointCloud<pcl::PointXYZI> newpcd;
    newpcd.width    = width*height;
    newpcd.height   = 1;
    newpcd.is_dense = false;
    newpcd.resize (newpcd.width * newpcd.height);
    int index = 0;

    for (int h = 0; h < height; h++)
    {
        for (int w = 0; w < width; w++)
        {
            float proj_x_min = x_min+pixelSize*w;
            float proj_x_max = proj_x_min+pixelSize;
            float proj_y_min = y_min+pixelSize*h;
            float proj_y_max = proj_y_min+pixelSize;

            double sum_intensity = 0.0;
            int count = 0;
            for (const auto& point: *cloud){
                if (point.x < proj_x_max && point.x > proj_x_min && point.y < proj_y_max && point.y > proj_y_min){
                    sum_intensity += point.intensity;
                    count ++;
                }
            }
            
            if(count!=0){
                img.at<double>(cv::Point(w,h)) = sum_intensity/count;
                alpha.at<double>(cv::Point(w,h)) = 255.0;
                if(img.at<double>(cv::Point(w,h)) < min_intensity){
                    min_intensity = img.at<double>(cv::Point(w,h));
                }else if (img.at<double>(cv::Point(w,h)) > max_intensity){
                    max_intensity = img.at<double>(cv::Point(w,h));
                }
                newpcd[index].x = (proj_x_min+proj_x_max)/2;
                newpcd[index].y = (proj_y_min+proj_y_max)/2;
                newpcd[index].z = 0;
                newpcd[index].intensity = sum_intensity/count;
                index++;
            }else{
                img.at<double>(cv::Point(w,h)) = (min_intensity+max_intensity)/2;
                newpcd[index].x = (proj_x_min+proj_x_max)/2;
                newpcd[index].y = (proj_y_min+proj_y_max)/2;
                newpcd[index].z = 0;
                newpcd[index].intensity = (min_intensity+max_intensity)/2;
                index++;
            }
        }
    }
    
    pcl::io::savePCDFileASCII ("pcd_intensity.pcd", newpcd);
    double intensity_scale = pixelValue/(max_intensity - min_intensity);

    // cv::Mat fin_img;
    // std::vector<cv::Mat> channels;

    // channels.push_back(img);
    // channels.push_back(img);
    // channels.push_back(img);
    // channels.push_back(alpha);/////
    // cv::merge(channels,fin_img);
    // std::cerr << fin_img << std::endl;
    // return fin_img;
    return img;
}

int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        return (-1);
    }

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    cv::Mat img = projectTo2d(removeNoise(cloud));
    cv::imwrite("projected.png", img, compression_params);

    return (0);
}

