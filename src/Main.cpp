#include "Stitcher.h"
#include "CloudPointProcessor.h"
//#include "ClickHandler.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <filesystem>


cv::Mat cropImage(const cv::Mat& image) {
    // Define the cropping rectangle based on the specified dimensions
    int top = 1162;
    int bottom = 5720 - 965;
    int left = 2297;
    int right = 14492 - 1696;

    // Crop the image by defining the Rect object
    cv::Rect cropRegion(left, top, right - left, bottom - top);

    // Crop and return the new image
    cv::Mat croppedImage = image(cropRegion).clone();
    return croppedImage;
}

cv::Mat stitch_images(std::string& images_directory){

	ImageStitcher stitcher;
	cv::Mat pano;
	std::vector<cv::Mat> images = stitcher.readImages(images_directory);

	if (!stitcher.stitchImages(images, pano)){
		std::cerr << "Stitching Failed!" << std::endl;
		EXIT_FAILURE;
	}

	std::cout << "Stitching Sucessfull\n" << std::endl;
	std::cout << "The size of the original panorama image is: " << pano.rows * pano.cols;

	return pano;
}



int main (int argc, char *argv[]) {

	if(!std::filesystem::exists(std::filesystem::path("../resources/panorama.jpg"))){
		std::string imagesDirectory = "../resources/images/";

		std::cout << "Beginning file stitching...\n" << std::endl;
		system("sleep 0.5");

		cv::Mat panorama = stitch_images(imagesDirectory);
		cv::Mat cropedPano = cropImage(panorama);

		std::cout << "Writting image to /resources/panorama.jpg\n";
		cv::imwrite("../resources/panorama.jpg", cropedPano);
		cv::imshow("croped", cropedPano);
		cv::waitKey(10);
		std::cout << "Stitching Process Finished! Check /resources .\n" << std::endl;

	} else {

		std::cout << "Skipping stiching process! \n" << std::endl;
		system("sleep 2");


	}
	std::cout << "Starting cloud point processing!\n" << std::endl;

	std::string cloudPointFile = "../resources/point-cloud.txt";
	CloudPointProcessor cloudPointProcessor;
	auto cloud =  CloudPointProcessor::loadPointCloud(cloudPointFile);

	//read pano
	//auto depthMap = CloudPointProcessor::mapToPixel(image, cloud);
	
	// Extracts Centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);
	

	// Move to origin
	Eigen::Affine3f translation = Eigen::Affine3f::Identity();
    translation.translation() << -centroid[0], -centroid[1], -centroid[2];
    pcl::transformPointCloud(*cloud, *cloud, translation);

	//rotate clockwise Y
	float theta = -M_PI / 5;  // angle to be rotated in radians
	Eigen::Affine3f rotationY = Eigen::Affine3f::Identity();
    rotationY.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
	
	//rotate clockwise Z
	float alpha = M_PI / 12;  // angle to be rotated in radians
	Eigen::Affine3f rotationZ = Eigen::Affine3f::Identity();
    rotationZ.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()));
	
	Eigen::Affine3f combinedRotations = rotationZ * rotationY;
	pcl::transformPointCloud(*cloud, *cloud, combinedRotations);

	// configure visualizer
	pcl::visualization::PCLVisualizer viewer("Point Cloud with Image Overlay");
	viewer.setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
	viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud, rgb, "cloud");

	
	float minX, minY = std::numeric_limits<float>::max();
    float maxX, maxY, maxZ = std::numeric_limits<float>::lowest();

    for (const auto& point : cloud->points) {
        if (point.x < minX) minX = point.x;
        if (point.x > maxX) maxX = point.x;
        if (point.y < minY) minY = point.y;
        if (point.y > maxY) maxY = point.y;
        if (point.z > maxZ) maxZ = point.z;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr bounding_box_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    bounding_box_cloud->push_back(pcl::PointXYZ(minX, minY, maxZ));  // Bottom-left
    bounding_box_cloud->push_back(pcl::PointXYZ(minX, maxY, maxZ));  // Top-left
    bounding_box_cloud->push_back(pcl::PointXYZ(maxX, maxY, maxZ));  // Top-right
    bounding_box_cloud->push_back(pcl::PointXYZ(maxX, minY, maxZ));  // Bottom-right

    viewer.addPolygon<pcl::PointXYZ>(bounding_box_cloud, 0.0, 1.0, 1.0, "bounding_box");  // Green color for the polygon
	
//	cv::Mat depthMap = 
//	cv::imshow("Depth Map", depth_map);
//    cv::waitKey(0);
//
//    return 0;

	while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

	return 0;
}


	//std::cout << "Correlating pixel with point from point cloud\n" << std::endl;
	//
	//cv::Mat pano = cv::imread("../resources/panorama.jpg");	
	//cv::Mat depthMap = cloudPointProcessor.mapToPixel(pano, points);
	//
	//ClickHandler clickHandler(pano, depthMap);
	//clickHandler.start();
