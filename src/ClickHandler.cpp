#include "ClickHandler.h"
#include <iostream>
#include <opencv2/highgui.hpp>


ClickHandler::ClickHandler(const cv::Mat& image, const cv::Mat& depthMap)
	:image(image), depthMap(depthMap){}

void ClickHandler::onMouse(int event, int x, int y, int flags, void *userdata){
	ClickHandler* handler = static_cast<ClickHandler*>(userdata);

	if(event == cv::EVENT_LBUTTONDOWN) {
		handler->displayPointInfo(x, y);
	}

}

void ClickHandler::start(){
	cv::namedWindow("Plane Extraction");
	cv::setMouseCallback("Plane Extraction", onMouse);

	cv::imshow("Plane Extraction", image);
	cv::waitKey(0);
}


void ClickHandler::displayPointInfo(int x, int y){

	if (x >= 0 && x < depthMap.cols && y >= 0 && y < depthMap.rows) {
		std::cout << "Clicked on pixel [" << y << "][" << x << "] with Z value: " << depthMap.at<double>(y, x) << std::endl;
	} else {
		std::cout << "Clicked point is out of bounds." << std::endl;
	}
}




