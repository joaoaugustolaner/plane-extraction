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



std::vector<Point> CloudPointProcessor::getPoints() const{ 
	return points; 
}


cv::Mat CloudPointProcessor::mapToPixel(const cv::Mat &image, std::vector<Point> &points) {

	// Initialize depthMap 
	cv::Mat depthMap = cv::Mat::zeros(image.size(), CV_32F);

	float zMin = -3.99f; 
	float zMax = 2.99f;  
	float zRange = zMax - zMin;

	std::vector<float> thetaCache(image.cols);
    std::vector<float> phiCache(image.rows);

    #pragma omp parallel for
    for (int j = 0; j < image.cols; j++) {
        thetaCache[j] = (float(j) / image.cols) * 2.0f * M_PI;
    }
    #pragma omp parallel for
    for (int i = 0; i < image.rows; i++) {
        phiCache[i] = (float(i) / image.rows) * M_PI;
    }

	#pragma omp parallel for collapse(2)
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
			
            float theta = thetaCache[j];
            float phi = phiCache[i];

            cv::Vec3f vecX(std::cos(theta), std::sin(theta), 0);
            cv::Vec3f vecY(0, std::sin(phi), std::cos(phi));

            float closestZ = std::numeric_limits<float>::lowest();

            for (const Point& point : points) {
                float normalizedZ = (point.z - zMin) / zRange;  // Normalize Z
                cv::Vec3f vecP(point.x, point.y, normalizedZ);

                if ((vecX.dot(vecP) >= 0) && (vecY.dot(vecP) >= 0) && normalizedZ > closestZ) {
                    closestZ = normalizedZ;
                }
            }

            depthMap.at<float>(i, j) = (closestZ != std::numeric_limits<float>::lowest()) ? closestZ : 0;

            // std::cout << "Assigned Z " << closestZ << " to pixel [" << i << "] [" << j << "]\n";
        }
	}
	return depthMap;
}
