#include "CloudPointProcessor.hpp"
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

#ifdef __APPLE__
	#include <sys/_pthread/_pthread_t.h>
#elif __linux__ 
	#include <pthread.h>
#else 
	#error "Platform not suported"
#endif

#define NUM_THREADS 10
#define NUM_LINES 15438380

struct Thread {
	CloudPointProcessor *processor;
	long startLine;
	long numLines;
	std::vector<Point> points;
};

// Initializes contructuor and store it in a variable defined in header
// The contructor also initializes the mutexes so the process can be Thread Safe
CloudPointProcessor::CloudPointProcessor(std::string filename) 
	: file(filename){
		pthread_mutex_init(&fileMutex, nullptr);
	}

CloudPointProcessor::~CloudPointProcessor(){
	pthread_mutex_destroy(&fileMutex);
}

void CloudPointProcessor::readCloudPoints() {
	std::ifstream file(this->file);

	if (!file.is_open()) {
		std::cerr << "File could not be open. Process failed! " << this->file
			<< std::endl;
		return;
	}

	pthread_t threads[NUM_THREADS];
	struct Thread threadData[NUM_THREADS];

	// each thread call processChunk
	// populate each structure
	for (int i = 0; i < NUM_THREADS; i++) {

		threadData[i].processor = this;
		threadData[i].startLine = i * (NUM_LINES / NUM_THREADS);
		threadData[i].numLines = (NUM_LINES / NUM_THREADS); 
		pthread_create(&threads[i], nullptr, processChunk, &threadData[i]);
	}

	// Wait each thread to finish
	for (int i = 0; i < NUM_THREADS; i++) {
		pthread_join(threads[i], nullptr);
	}

	for (int i = 0; i < NUM_THREADS; i++) {
		points.insert(points.end(), threadData[i].points.begin(), threadData[i].points.end());
	}
}

void *CloudPointProcessor::processChunk(void *args) {
	// safely converts to it's original type
	Thread *threads = static_cast<Thread *>(args);

	CloudPointProcessor *processor = threads->processor;
	long start = threads->startLine;
	long numLines = threads->numLines;

	std::ifstream file(processor->file);

	if (!file.is_open()) {
		std::cerr << "Couldn't open file" << processor->file << std::endl;
		pthread_exit(nullptr);
	}

	std::string line;
	std::vector<Point>& pointsSection = threads->points;

	for (long i = start; i < start + numLines && std::getline(file, line); i++) {
		processor->processLine(line, pointsSection);
	}

	pthread_exit(nullptr);

}

void CloudPointProcessor::processLine(const std::string &line, std::vector<Point>& points) {

	std::istringstream stringStream(line);
	Point point;

	if (!(stringStream >> point.x >> point.y >> point.z >> point.r >> point.g >>
				point.b)) {
		std::cerr << "Error parsing line!" << line << std::endl;
		return;
	}

	points.push_back(point);
}

const std::vector<Point> &CloudPointProcessor::getPoints() const { return points; }

// Function to calculate the Euclidean distance in the XY plane
double calculateXYDistance(double pixelX, double pixelY, double pointX, double pointY) {
    return std::sqrt((pixelX - pointX) * (pixelX - pointX) + (pixelY - pointY) * (pixelY - pointY));
}

cv::Mat CloudPointProcessor::mapToPixel(const cv::Mat &image, const std::vector<Point> &points){
	
	//Create depthMap with same size of image, type is a 64 bit float and fill it with zeros.
	cv::Mat depthMap(image.rows, image.cols, CV_64F, cv::Scalar(std::numeric_limits<double>::quiet_NaN()));
	
	// All distances are in meters
	double width = 6.17; 
    double height = 2.23;   
	double distance = 2.23;

    double fovX = 2 * atan(width / (2 * distance)) * (180 / M_PI); 
    double fovY = 2 * atan(height / (2 * distance)) * (180 / M_PI);
	
	 //this determines the area of influence of the conic projection, so to keep precise, we will use the average reprensation of a pixel in real world;
	double coneRadius = ((width/image.cols) + (height/image.rows)) / 2;

	for(int i = 0; i < image.rows; i++){
		for(int j = 0; j < image.cols; j++){
			
			double minZdistance = std::numeric_limits<double>::max();
			const Point* closestPoint = nullptr;

			//Map pixel to it's position and normalize coordinates
			double pixelX = ((double)i / image.cols) * 2.0 - 1.0;
			double pixelY = ((double)j / image.rows) * 2.0 - 1.0;

			pixelX *= std::tan(fovX * 0.5 * M_PI / 180.0);
			pixelY *= std::tan(fovY * 0.5 * M_PI / 180.0);

			for (const Point& point: points){
				double xyDistance = calculateXYDistance(pixelX, pixelY, point.x, point.y);

				if(xyDistance < coneRadius) {
					double zDistance = std::abs(0 - point.z);

					if (zDistance < minZdistance) {
						minZdistance = zDistance;
						closestPoint = &point;
					}	
				}
			}

			if (closestPoint) {
				depthMap.at<double>(i,j) = closestPoint->z;
			}
	

		}
	}

	return depthMap;
}


