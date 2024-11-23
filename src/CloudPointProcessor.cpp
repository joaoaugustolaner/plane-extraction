#include "CloudPointProcessor.h"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <pthread.h>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef __APPLE__
#include <sys/_pthread/_pthread_t.h>
#elif __linux__ 
#include <pthread.h>
#else 
#error "Platform not suported"
#endif

#define NUM_LINES 15438380
	
struct ThreadArgs {
    int startLine;  
    int endLine;    
    std::string filename; 
    std::vector<pcl::PointXYZRGBNormal>* threadPoints; 
};

void* parseFileChunk(void* args) {
    auto* threadArgs = static_cast<ThreadArgs*>(args);

    std::ifstream file(threadArgs->filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file in thread!" << std::endl;
        pthread_exit(nullptr);
    }
    // Skip lines before the start line
    std::string line;
    for (int i = 0; i < threadArgs->startLine && std::getline(file, line); ++i);

    // Process lines in the given range
    for (int i = threadArgs->startLine; i < threadArgs->endLine; ++i) {
        if (!std::getline(file, line)) break;

        std::istringstream stringStream(line);
        pcl::PointXYZRGBNormal point;
        float x, y, z, r, g, b, nx, ny, nz;

        if (stringStream >> x >> y >> z >> r >> g >> b >> nx >> ny >> nz) {
            point.x = x;
            point.y = y;
            point.z = z;
            point.r = static_cast<uint8_t>(r);
            point.g = static_cast<uint8_t>(g);
            point.b = static_cast<uint8_t>(b);
            point.normal_x = nx;
            point.normal_y = ny;
            point.normal_z = nz;

            threadArgs->threadPoints->push_back(point);
        }
    }

    pthread_exit(nullptr);
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CloudPointProcessor::loadPointCloud(const std::string &filename, int numThreads){

	auto cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);	

	int chunkSize = (NUM_LINES + numThreads - 1) / numThreads;
	
	pthread_t threads[numThreads];
    ThreadArgs threadArgs[numThreads];
    std::vector<std::vector<pcl::PointXYZRGBNormal>> threadResults(numThreads);
	
	for (int i = 0; i < numThreads; ++i) {
        threadArgs[i].startLine = i * chunkSize;
        threadArgs[i].endLine = std::min((i + 1) * chunkSize, NUM_LINES);
        threadArgs[i].filename = filename;
        threadArgs[i].threadPoints = &threadResults[i];

        if (pthread_create(&threads[i], nullptr, parseFileChunk, &threadArgs[i]) != 0) {
            std::cerr << "Error creating thread " << i << std::endl;
            return nullptr;
        }
    }

	    for (int i = 0; i < numThreads; ++i) {
        pthread_join(threads[i], nullptr);
    }

    // Combine results
    for (const auto& threadResult : threadResults) {
        cloud->points.insert(cloud->points.end(), threadResult.begin(), threadResult.end());
    }

    return cloud;
}


std::pair< cv::Mat&, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> CloudPointProcessor::mapToPixel(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& pointCloud,
		cv::Mat& image) {
	
	
	const double X_MIN = -3.15, X_MAX = 3.15;
    const double Y_MIN = -0.8, Y_MAX = 1.1;
    const int IMG_WIDTH = image.cols;
    const int IMG_HEIGHT = image.rows;

	auto depthMap = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	depthMap->width = IMG_WIDTH;
	depthMap->height = IMG_HEIGHT;
	depthMap->points.resize(IMG_WIDTH * IMG_HEIGHT);
    
	std::vector<double> zBuffer(IMG_WIDTH * IMG_HEIGHT, std::numeric_limits<double>::lowest());
	
	for (const auto& point : pointCloud->points) {
        if (point.x < X_MIN || point.x > X_MAX || point.y < Y_MIN || point.y > Y_MAX) {
            continue;
        }
		
		
        int pixelX = static_cast<int>(((point.x - X_MIN) / (X_MAX - X_MIN)) * (IMG_WIDTH - 1));
        int pixelY = static_cast<int>(((point.y - Y_MIN) / (Y_MAX - Y_MIN)) * (IMG_HEIGHT - 1));
		pixelY = IMG_HEIGHT - 1 - pixelY;	


        int pixelIndex = pixelY * IMG_WIDTH + pixelX;
		
        if (point.z > zBuffer[pixelIndex]) {
            zBuffer[pixelIndex] = point.z;  
            depthMap->points[pixelIndex] = point;  
			
			std::cout << "Updating pixel: " << pixelX << ", " << pixelY
              << " with Z: " << point.z << std::endl;
        } 
    }
	
	return {image, depthMap};
}


