#include "ClickHandler.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


ClickHandler::ClickHandler(const cv::Mat& image, const cv::Mat& depthMap)
	:image(image.clone()), depthMap(depthMap){}

void ClickHandler::onMouse(int event, int x, int y, int flags, void *userdata){
	ClickHandler* handler = static_cast<ClickHandler*>(userdata);

	if(event == cv::EVENT_LBUTTONDOWN) {
		handler->drawClick(x, y);
		handler->selectedPoints.push_back(cv::Point3f(x, y, handler->depthMap.at<float>(y,x)));
	}

}

void ClickHandler::start(){
	cv::namedWindow("Plane Extraction");
	cv::setMouseCallback("Plane Extraction", onMouse, this);

	cv::imshow("Plane Extraction", image);

	while (true) {
		// Listen for keypresses
		const int key = cv::waitKey(0); // Wait indefinitely for a keypress
		
		if (key == 13) { // ASCII code for "Enter"
			drawPlane(); 
		} else if (key == 114){
			reset();
		} else if (key == 27) { // ASCII code for "Esc" to exit
			break;
		}
	}
}

void ClickHandler::drawClick(int x, int y){

	cv::circle(image, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), cv::FILLED);

	cv::imshow("Plane Extraction", image);
	cv::waitKey(1); //delay to make sure is rendered
}

void ClickHandler::drawPlane(){

	int numPoints = selectedPoints.size();
    
    cv::Mat A(numPoints, 3, CV_32F);  // Matrix for X, Y, and 1
    cv::Mat b(numPoints, 1, CV_32F);  // Matrix for Z

    for (int i = 0; i < numPoints; ++i) {
        A.at<float>(i, 0) = selectedPoints[i].x;
        A.at<float>(i, 1) = selectedPoints[i].y;
        A.at<float>(i, 2) = 1.0f;  // Add a constant term for the intercept
        b.at<float>(i, 0) = selectedPoints[i].z;
    }

    // Step 2: Solve the linear system A * P = b using least squares (cv::solve).
    cv::Mat P;
    cv::solve(A, b, P, cv::DECOMP_SVD);
	
	// P contains the parameters for the plane equation: ax + by + c = z
    float a = P.at<float>(0, 0);  // Coefficient for x
    float b_coeff = P.at<float>(1, 0);  // Coefficient for y
    float c = P.at<float>(2, 0);// Intercept
								
	float maxY, maxX = -1; 
	float minY, minX = 100000;
	
	for (const cv::Point3f& point : selectedPoints) {
		std::cout << "[" << point.x << ", " << point.y << ", " << point.z << "]\n";

		if (point.x < minX) { 
			minX = point.x; 
		}

        if (point.x > maxX) { 
			maxX = point.x; 
		}

        if (point.y < minY) { 
			minY = point.y; 
		}

        if (point.y > maxY) { 
			maxY = point.y; 
		}
	}

	cv::Point3f p1(minX, minY, a * minX + b_coeff * minY + c);
    cv::Point3f p2(maxX, minY, a * maxX + b_coeff * minY + c);
    cv::Point3f p3(minX, maxY, a * minX + b_coeff * maxY + c);
    cv::Point3f p4(maxX, maxY, a * maxX + b_coeff * maxY + c);

	//project into 2D
	cv::Point2f p1_2D(p1.x, p1.y);
    cv::Point2f p2_2D(p2.x, p2.y);
    cv::Point2f p3_2D(p3.x, p3.y);
    cv::Point2f p4_2D(p4.x, p4.y);
	
	// Step 4: Draw the plane (polygon)
    std::vector<cv::Point> planePoints = {p1_2D, p2_2D, p4_2D, p3_2D};

	std::cout << "Drawing Plane ...\n";
    cv::polylines(image, planePoints, true, cv::Scalar(0, 255, 255), 10);
	
	cv::imshow("Plane Extraction", image);
	cv::waitKey(1); //delay to make sure is rendered
	
}

void ClickHandler::reset() {
	
	selectedPoints.clear();

	image = originalImage.clone();

	cv::imshow("Plane Extraction Reseted", image);

}
