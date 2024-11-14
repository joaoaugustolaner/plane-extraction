#include "CloudPointProcessor.h"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <pthread.h>
#include <sstream>
#include <string>
#include <vector>

#ifdef __APPLE__
#include <sys/_pthread/_pthread_t.h>
#elif __linux__ 
#include <pthread.h>
#else 
#error "Platform not suported"
#endif

#define NUM_LINES 15438380
#define PLANE_MAX_X_LIMIT 3.43346
#define PLANE_MIN_X_LIMIT -4.43514
#define PLANE_MAX_Y_LIMIT 1.32841
#define PLANE_MIN_Y_LIMIT -1.09815
#define PLANE_Z 1.5



	
struct cellData {
	bool hasPoint = false;
	pcl::PointXYZRGBNormal assignedPoint;
};

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CloudPointProcessor::loadPointCloud(const std::string &filename){

	auto cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	std::ifstream file(filename);
	if (!file.is_open()) {
        std::cerr << "Error opening file! " << std::endl;
		return NULL;
	}

    std::string line;
	while (std::getline(file, line)) {
		std::istringstream stringStream(line);
		pcl::PointXYZRGBNormal point;
		float x, y, z, r, g, b, nx, ny, nz;

		if (stringStream >> x >> y >> z >> r >> g >> b >> nx >> ny >> nz) {
			point.x = x;
			point.y = y;
			point.z = z;
			point.r = static_cast<uint8_t>(r);
			point.g = static_cast<uint8_t>(g);
			point.b = static_cast<uint8_t>(b);
			point.normal_x = nx;
			point.normal_y = ny;
			point.normal_z = nz;
		}

        cloud->points.push_back(point);
    }

	std::cout << "File parsing completed. \n" << std::endl; 
	return cloud;
}


 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CloudPointProcessor::mapToPixel(cv::Mat& image,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& pointCloud) {
	
	std::cout << "Started DepthMap assemble.\n" << std::endl;

	auto depthMap = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	depthMap->width = image.cols;
	depthMap->height = image.rows;
	
	std::vector<std::vector<cellData>> pointMap(image.rows, std::vector<cellData>(image.cols));

	//Source point
	pcl::PointXYZ sourcePoint(0,0,10);
	
	std::cout << "Performing Calculations in point cloud\n" << std::endl;

	for (const auto& point : pointCloud->points) {
		double t = (PLANE_Z - sourcePoint.z)/(point.z - sourcePoint.z);

		if(t < 0 || t > 1) {
			continue;
		}

		double xInt = sourcePoint.x + (t *(point.x - sourcePoint.x));
		double yInt = sourcePoint.y + (t *(point.y - sourcePoint.y));

		if (xInt < PLANE_MIN_X_LIMIT || xInt > PLANE_MAX_X_LIMIT 
				|| yInt <  PLANE_MIN_Y_LIMIT|| yInt > PLANE_MAX_Y_LIMIT) {
			continue;
		}

		int col = static_cast<int>((xInt - PLANE_MIN_X_LIMIT) / 
				(PLANE_MAX_X_LIMIT - PLANE_MIN_X_LIMIT) * (image.cols - 1));
        int row = static_cast<int>((yInt - PLANE_MIN_Y_LIMIT) / 
				(PLANE_MAX_Y_LIMIT - PLANE_MIN_Y_LIMIT) * (image.rows - 1));

		if (!pointMap[row][col].hasPoint || 
				point.z > pointMap[row][col].assignedPoint.z) {
            pointMap[row][col].assignedPoint = point;
            pointMap[row][col].hasPoint = true;
        }

	}

	for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            if (pointMap[r][c].hasPoint) {
                pcl::PointXYZRGBNormal& point = pointMap[r][c].assignedPoint; 
                // Add the highest-Z point for this cell to the output point cloud
                depthMap->points.push_back(point);
            }
        }
    }
	
	std::cout << "Calculations finished." << std::endl;
	return depthMap;
}


