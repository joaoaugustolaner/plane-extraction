#include "ClickHandler.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


ClickHandler::ClickHandler(const cv::Mat& image, const cv::Mat& depthMap)
	:image(image.clone()), depthMap(depthMap){}

void ClickHandler::onMouse(int event, int x, int y, int flags, void *userdata){
	ClickHandler* handler = static_cast<ClickHandler*>(userdata);

	if(event == cv::EVENT_LBUTTONDOWN) {
		handler->displayPointInfo(x, y);
		handler->drawClick(x, y);
	}

}

void ClickHandler::start(){
	cv::namedWindow("Plane Extraction");
	cv::setMouseCallback("Plane Extraction", onMouse, this);

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

void ClickHandler::drawClick(int x, int y){

	cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), cv::FILLED);

	cv::imshow("Plane Extraction", image);
	cv::waitKey(1); //delay to make sure is rendered
}



