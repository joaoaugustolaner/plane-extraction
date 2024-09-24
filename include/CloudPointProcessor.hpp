#ifndef CLOUD_POINT_PROCESSOR_HPP
#define CLOUD_POINT_PROCESSOR_HPP

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
	
	private:

		std::string file;
		std::vector<Point> points;

		//Thread processing
    	static void* processChunk(void* arg);
		
		//Process line
   		void processLine(const std::string& line, std::vector<Point>& points);
				
		//mutex for file
    	pthread_mutex_t fileMutex;

		//mutex for point array
		pthread_mutex_t pointsMutex;
};



#endif // CLOUD_POINT_PROCESSOR_HPP

