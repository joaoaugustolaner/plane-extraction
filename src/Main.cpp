#include "Stitcher.h"
#include "CloudPointProcessor.h"
#include "ClickHandler.h"

#include <cstdlib>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

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
	CloudPointProcessor cloudPointProcessor(cloudPointFile);
	cloudPointProcessor.readCloudPoints();
	std::vector<Point> points = cloudPointProcessor.getPoints();
	
	std::cout << "Correlating pixel with point from point cloud\n" << std::endl;
	
	cv::Mat pano = cv::imread("../resources/panorama.jpg");	
	cv::Mat depthMap = cloudPointProcessor.mapToPixel(pano, points);
	
	ClickHandler clickHandler(pano, depthMap);
	clickHandler.start();


	return 0;
}
