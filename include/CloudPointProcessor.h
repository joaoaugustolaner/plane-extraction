#ifndef CLOUD_POINT_PROCESSOR_H
#define CLOUD_POINT_PROCESSOR_H

#include <opencv2/core/mat.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <utility>
#include <vector>
#include <string>
#include <pthread.h>





class CloudPointProcessor {


	public:
		static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadPointCloud(const std::string& filename, int numThreads);

		static std::pair<cv::Mat&, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> mapToPixel(
				const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& pointCloud,
				cv::Mat& image);
	private:
		std::vector<pcl::PointXYZRGBNormal> points;
};

#endif //CLOUD_POINT_PROCESSOR_H
