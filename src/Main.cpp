#include "Stitcher.hpp"
#include "CloudPointProcessor.hpp"

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



cv::Mat stitch_images(std::string& images_directory){

	ImageStitcher stitcher;
	cv::Mat pano;
	std::vector<cv::Mat> images = stitcher.readImages(images_directory);

	if (!stitcher.stitchImages(images, pano)){
		std::cerr << "Stitching Failed!" << std::endl;
		EXIT_FAILURE;
	}

	std::cout << "Stitching Sucessfull\n" << std::endl;

	return pano;
}

int main (int argc, char *argv[]) {

	if(!std::filesystem::exists(std::filesystem::path("../resources/pano/panorama.jpg"))){
		std::string imagesDirectory = "../resources/images/";

		std::cout << "Beginning file stitching...\n" << std::endl;
		system("sleep 0.5");

		cv::Mat panorama = stitch_images(imagesDirectory);
		std::cout << "Writting image to /resources/pano/panorama.jpg\n";

		cv::imwrite("../resources/pano/panorama.jpg", panorama);
		std::cout << "Stitching Process Finished! Check /resources/pano/ .\n" << std::endl;

	} else {

		std::cout << "Skipping stiching process! \n" << std::endl;
		system("sleep 2");

	}

	std::cout << "Starting cloud point processing!\n";

	std::string cloudPointFile = "../resources/point-cloud.txt";
	CloudPointProcessor cloudPointProcessor(cloudPointFile);
	cloudPointProcessor.readCloudPoints();

	std::cout << "Process finished!\n";


	std::vector<Point> points = cloudPointProcessor.getPoints();
	std::cout << "The software has processed " << points.size() << " points from the file:" << cloudPointFile << std::endl;

	return 0;
}
