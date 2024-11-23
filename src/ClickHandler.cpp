#include "ClickHandler.h"
#include <opencv2/core/check.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

ClickHandler::ClickHandler(const cv::Mat& image, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr depthMap): 
	image_(image), depthMap_(depthMap){	
		displayedImage_ = image;
}

pcl::PointXYZRGBNormal ClickHandler::findCorrelatedPoint(int x, int y){
	if(depthMap_->empty()){
		return NULL;	
	}
	return depthMap_->points[y * image_.cols + x];
}

void ClickHandler::drawCircle(int x, int y) {

    cv::circle(displayedImage_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
	cv::imshow("Viewer", displayedImage_);
}

std::vector<pcl::PointXYZRGBNormal> ClickHandler::getSelectedPoints(){
	return this->selectedPoints;
}


void ClickHandler::onMouse(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		
		ClickHandler* handler = static_cast<ClickHandler*>(userdata);

		std::cout << "Clicked at (" << x << ", " << y << ")" << std::endl;

		pcl::PointXYZRGBNormal point = handler->findCorrelatedPoint(x, y);
		std::cout << "Correlated Point: "
			<< "X: " << point.x << ", Y: " << point.y << ", Z: " << point.z << std::endl;
		handler->selectedPoints.push_back(point);
		
		handler->drawCircle(x, y);
	}
}

bool ClickHandler::getEnterStatus(){
	return this->enterPressed_;
}

void ClickHandler::handleClicks(const std::string &windowName){
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(windowName, onMouse, this);
	
	cv::imshow(windowName, displayedImage_);
	
	while (true) {
        int key = cv::waitKey(1);
        if (key == 27) { // ESC key
            std::cout << "Exiting..." << std::endl;
            break;
        }

		if (key == 13) { // Enter key
            std::cout << "Enter pressed. Rendering plane..." << std::endl;
            this->enterPressed_ = true;
            break;
        }
    }

	cv::destroyWindow(windowName);
}
