#include "ClickHandler.h"
#include "CloudPointProcessor.h"
#include "Stitcher.h"
#include "Draw.h"

#include <opencv2/imgproc.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

cv::Mat scaleImage(cv::Mat &image, int targetWidth, int targetHeight) {

	int originalWidth = image.cols;
	int originalHeight = image.rows;

	if (originalWidth == targetWidth && originalHeight == targetHeight) {
		return image;
	}

	double scaleX = ((double)targetWidth / originalWidth);
	double scaleY = ((double)targetHeight / originalHeight);

	double scaleFactor = (scaleX > scaleY) ? scaleX : scaleY;

	int newWidth = static_cast<int>(originalWidth * scaleFactor);
	int newHeight = static_cast<int>(originalHeight * scaleFactor);

	cv::resize(image, image, cv::Size(newWidth, newHeight));

	return image;
}

cv::Mat cropImage(cv::Mat &image) {
	// Define the cropping rectangle based on the specified dimensions
	int top = 1330;
	int bottom = 5720 - 449;
	int left = 1184;
	int right = 14493 - 1688;

	// Crop the image by defining the Rect object
	cv::Rect cropRegion(left, top, right - left, bottom - top);

	// Crop and return the new image
	cv::Mat croppedImage = image(cropRegion).clone();
	return croppedImage;
}

cv::Mat stitch_images(std::string &images_directory) {

	ImageStitcher stitcher;
	cv::Mat pano;
	std::vector<cv::Mat> images = stitcher.readImages(images_directory);

	if (!stitcher.stitchImages(images, pano)) {
		std::cerr << "Stitching Failed!" << std::endl;
		EXIT_FAILURE;
	}

	std::cout << "Stitching Sucessfull\n" << std::endl;

	return pano;
}

int main(int argc, char *argv[]) {

	if (!std::filesystem::exists(
				std::filesystem::path("../resources/panorama.jpg"))) {
		std::string imageDirectory = "../resources/images/";

		std::cout << "Beginning file stitching...\n" << std::endl;

		cv::Mat panorama = stitch_images(imageDirectory);
		cv::Mat cropedPano = cropImage(panorama);

		std::cout << "Writting image to /resources/panorama.jpg\n";
		cv::imwrite("../resources/panorama.jpg", cropedPano);
		std::cout << "Stitching Process Finished! Check /resources .\n"
			<< std::endl;

	} else {
		std::cout << "Skipping stiching process! \n" << std::endl;
	}

	std::cout << "Starting cloud point processing!\n" << std::endl;

	std::string cloudPointFile = "../resources/point-cloud.txt";
	cv::Mat image = cv::imread("../resources/panorama.jpg");
	image = scaleImage(image, 2160, 1080); // 5120, 2160);
	std::cout << "Size: " << image.cols << " x " << image.rows << std::endl;

	CloudPointProcessor cloudPointProcessor;
	int numThreads = 10;
	if (!std::filesystem::exists(
				std::filesystem::path("../resources/point-cloud.pcd"))) {
		auto cloud = CloudPointProcessor::loadPointCloud(cloudPointFile, numThreads);
		std::cout << "Finished Processing!\nStarted Rotations!\n" << std::endl;
		// Extracts Centroid
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);

		// Move Centroid to Origin
		Eigen::Affine3f translation = Eigen::Affine3f::Identity();
		translation.translation() << -centroid[0], -centroid[1], -centroid[2];
		pcl::transformPointCloud(*cloud, *cloud, translation);

		// rotate clockwise Y
		float theta = -M_PI / 5; // angle to be rotated in radians
		Eigen::Affine3f rotationY = Eigen::Affine3f::Identity();
		rotationY.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

		// rotate clockwise Z
		float alpha = -M_PI / 15; // angle to be rotated in radians
		Eigen::Affine3f rotationZ = Eigen::Affine3f::Identity();
		rotationZ.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()));

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rotatedCloud;
		rotatedCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		Eigen::Affine3f combinedRotations = rotationY * rotationZ;

		pcl::transformPointCloud(*cloud, *rotatedCloud, combinedRotations);
		std::cout << "Finished Rotations!\n" << std::endl;

		pcl::io::savePCDFileASCII("../resources/point-cloud.pcd", *rotatedCloud);
		std::cout << "Finished saving.\n";
	}
	
	std::cout << "Opening the file\n";

	auto cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());	
	pcl::io::loadPCDFile("../resources/point-cloud.pcd", *cloud);

	auto [outImage,depthMap] = CloudPointProcessor::mapToPixel(cloud, image);

	std::cout << "Original size: " << cloud->points.size() << "\n DepthMap size: " << depthMap->points.size();

	ClickHandler handler(outImage, depthMap);
	handler.handleClicks("Viewer");

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGBNormal>(depthMap, "sample cloud");
	viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");


	Draw draw(viewer);

	if(handler.getEnterStatus()){	
		draw.fitPlaneAndRender(handler.getSelectedPoints());
		draw.projectPlaneToImage(image, handler.getSelectedPoints());
	}

	while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
	
	return 0;
}
