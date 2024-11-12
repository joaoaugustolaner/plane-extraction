#include "CloudPointProcessor.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <pthread.h>
#include <sstream>
#include <string>
#include <vector>
#include <omp.h>

#ifdef __APPLE__
#include <sys/_pthread/_pthread_t.h>
#elif __linux__ 
#include <pthread.h>
#else 
#error "Platform not suported"
#endif

#define NUM_LINES 15438380

constexpr double CAMERA_DISTANCE = 2.23;
constexpr double REAL_IMAGE_HEIGHT = 1.94;
constexpr double REAL_IMAGE_WIDTH = 6.4;





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
	std::cout << "Loaded " << cloud->points.size() << " points from " << filename << std::endl;
	return cloud;
}


cv::Mat CloudPointProcessor::mapToPixel(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& pointCloud,
		const cv::Point3f& cameraPosition,
		const cv::Size& imageSize,
		float pixelSize) {

	cv::Mat depthMap = cv::Mat::zeros(imageSize, CV_32F);
	
	#pragma omp parallel for
    for (size_t i = 0; i < pointCloud->points.size(); ++i) {
        const pcl::PointXYZRGBNormal& point = pointCloud->points[i];

        // Calculate vector from camera to point
        cv::Point3f vector = cv::Point3f(point.x, point.y, point.z) - cameraPosition;

        // Skip points behind the camera
        if (vector.z <= 0) continue;

        // Project to 2D pixel coordinates
        int px = static_cast<int>((vector.x / vector.z) / (pixelSize + (imageSize.width / 2.0f)));
        int py = static_cast<int>((vector.y / vector.z) / (pixelSize + (imageSize.height / 2.0f)));

        // Check if the projected point falls within the image boundaries
        if (px >= 0 && px < imageSize.width && py >= 0 && py < imageSize.height) {
            // Use critical section to safely update shared depth_map
            #pragma omp critical
            {
                // Update depth map only if this point is closer
                if (vector.z < depthMap.at<float>(py, px)) {
                    depthMap.at<float>(py, px) = vector.z;
					std::cout << "Assigned Z " << vector.z << " to pixel ["<< py << "] [" << px << "].\n";
                }
            }
        }
    }

	return depthMap;
}


