#ifndef CLOUD_POINT_PROCESSOR_H
#define CLOUD_POINT_PROCESSOR_H

#include <opencv2/core/mat.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <pthread.h>



struct Camera {
	cv::Point3f position;
	cv::Size imageSize;
};

class CloudPointProcessor {


	public:
		static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadPointCloud(const std::string& filename);

		static cv::Mat mapToPixel(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& point_cloud,
				const cv::Point3f& camera_position,
				const cv::Size& image_size,
				float pixel_size
				);

	private:
		std::vector<pcl::PointXYZRGBNormal> points;
};

#endif //CLOUD_POINT_PROCESSOR_H
