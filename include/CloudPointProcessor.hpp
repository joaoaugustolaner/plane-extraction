#ifndef CLOUD_POINT_PROCESSOR_HPP
#define CLOUD_POINT_PROCESSOR_HPP

#include <opencv2/core/mat.hpp>
#include <sys/_pthread/_pthread_mutex_t.h>
#include <vector>
#include <string>
#include <pthread.h>

struct Point {

	//spatial coordinates
    double x, y, z;

	//point color
    int r, g, b;
};

class CloudPointProcessor {


	public:
    	CloudPointProcessor(const std::string filePath);
    	~CloudPointProcessor();
    
    	void readCloudPoints();
		
		//readonly, don't change state of Object
		const std::vector<Point>& getPoints() const;
		
		// Map Point to Pixel creating two matrices with same height and length
		cv::Mat mapToPixel(const cv::Mat& image, const std::vector<Point>& points);

	private:
		std::string file;
		std::vector<Point> points;

		//Thread processing
    	static void* processChunk(void* arg);
		
		//Process line of file
   		void processLine(const std::string& line, std::vector<Point>& points);

		pthread_mutex_t fileMutex;

		pthread_mutex_t pointsMutexl;
};



#endif // CLOUD_POINT_PROCESSOR_HPP

