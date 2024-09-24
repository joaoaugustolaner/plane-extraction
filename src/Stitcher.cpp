#include "Stitcher.hpp"

#include <filesystem>
#include <iostream>
#include <vector>
#include <opencv2/imgcodecs.hpp>

// Constructor
ImageStitcher::ImageStitcher() {

    // Create a stitcher instance with PANORAMA mode
    stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);
}

ImageStitcher::~ImageStitcher(){}

// Stitch Images Method
// boolean return

bool ImageStitcher::stitchImages(const std::vector<cv::Mat> &images, cv::Mat &result_image){
	cv::Stitcher::Status status = stitcher->stitch(images, result_image);

	if(status != cv::Stitcher::Status::OK){
		std::cerr << "Error: " << status << std::endl;
		return false;
	}

	return true;
}



std::vector<cv::Mat> ImageStitcher::readImages(std::string path){
	
	std::vector<cv::Mat> images;
	
	for(const auto& file : std::filesystem::directory_iterator(path)){
		if(file.path().extension() == ".JPG"){
			cv::Mat image = cv::imread(file.path().string());
			images.push_back(image);
		}	
	}
	
	return images;
}
