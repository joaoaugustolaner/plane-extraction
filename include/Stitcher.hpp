#ifndef STITCHER_HPP
#define STITCHER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <vector>
#include <string>

class ImageStitcher {
	
	public:
    	ImageStitcher();
    	~ImageStitcher();

    	// Stitcher declaration
    	bool stitchImages(const std::vector<cv::Mat>& images, cv::Mat& result_image);
		
		// Read .JPGs from specified directory 
		std::vector<cv::Mat> readImages(std::string path);

	private:
		cv::Ptr<cv::Stitcher> stitcher;
};

#endif // IMAGE_STITCHER_HPP

