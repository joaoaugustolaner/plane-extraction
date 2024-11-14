#ifndef CLOUD_POINT_PROCESSOR_H
#define CLOUD_POINT_PROCESSOR_H

#include <opencv2/core/mat.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <pthread.h>





class CloudPointProcessor {


	public:
		static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadPointCloud(const std::string& filename);

		static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mapToPixel(cv::Mat& image,
				const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& point_cloud);

	private:
		std::vector<pcl::PointXYZRGBNormal> points;
};

#endif //CLOUD_POINT_PROCESSOR_H
