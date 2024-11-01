#include "CloudPointProcessor.h"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <pthread.h>
#include <sstream>
#include <string>
#include <vector>
#include <omp.h>

#ifdef __APPLE__
	#include <sys/_pthread/_pthread_t.h>
#elif __linux__ 
	#include <pthread.h>
#else 
	#error "Platform not suported"
#endif

#define NUM_LINES 15438380

// Initializes contructuor and store it in a variable defined in header
// The contructor also initializes the mutexes so the process can be Thread Safe
CloudPointProcessor::CloudPointProcessor(std::string filename) 
	: file(filename){}

CloudPointProcessor::~CloudPointProcessor(){}

void CloudPointProcessor::readCloudPoints() {
	std::ifstream file(this->file);

	if (!file.is_open()) {
		std::cerr << "File could not be open. Process failed! " << this->file << std::endl;
		return;
	}
	
	std::vector<Point> tempPoints(NUM_LINES);

	#pragma omp parallel
	{
		std::ifstream threadFile(this->file);
		if (!threadFile.is_open()) {
			std::cerr << "Error: Could not open file in thread." << std::endl;
			#pragma omp barrier
		} else {
			std::string line;
			#pragma omp for schedule(static)
			for (long i = 0; i < NUM_LINES; ++i) {
				if (std::getline(threadFile, line)) {
					this->processLine(line, tempPoints.at(i));
				}
			}
		}
	}

	points = std::move(tempPoints);


}


void CloudPointProcessor::processLine(const std::string &line, Point &point) {
	std::istringstream stringStream(line);

	if (!(stringStream >> point.x >> point.y >> point.z >> point.r >> point.g >> point.b)) {
		std::cerr << "Error parsing line!" << line << std::endl;
	}
}



const std::vector<Point> &CloudPointProcessor::getPoints() const { 
	return points; 
}


double computeAngle(const cv::Vec3d& vec1, const cv::Vec3d& vec2) {
    double dotProduct = vec1.dot(vec2);
    double magnitude1 = cv::norm(vec1);
    double magnitude2 = cv::norm(vec2);
    return std::acos(dotProduct / (magnitude1 * magnitude2));
}

cv::Mat CloudPointProcessor::mapToPixel(const cv::Mat &image, const std::vector<Point> &points) {

	// Initialize depthMap 
	cv::Mat depthMap = cv::Mat::zeros(image.size(), CV_32F);
	
	#pragma omp parallel for
	for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
			
			float theta = (float(j) / image.cols) * 2.0f * M_PI;
			float phi = (float(i) / image.rows) * M_PI; 

			cv::Vec3f vecX(std::cos(theta), std::sin(theta), 0);
            cv::Vec3f vecY(0, std::sin(phi), std::cos(phi));


			float closestZ = std::numeric_limits<float>::max();
            Point closestPoint;
	
			for (const Point& pt : points) {
                cv::Vec3f vecP(pt.x, pt.y, pt.z);

                // Check if pt falls within pixel's solid angle
                if ((vecX.dot(vecP) >= 0) && (vecY.dot(vecP) >= 0)) {
                    if (fabs(pt.z) < closestZ) {
                        closestZ = pt.z;
                        closestPoint = pt;
                    }
                }
            }
			
			#pragma omp critical
			{
				if (closestZ != std::numeric_limits<float>::max()) {
					depthMap.at<float>(i, j) = closestZ;
				} else {
					closestZ = 0;
					depthMap.at<float>(i,j) = closestZ;
				}
				
				std::cout << "Assigned Z " << closestZ << " to pixel [" << i << "] [" << j << "]\n";
			}
		}
	}
	return depthMap;
}
