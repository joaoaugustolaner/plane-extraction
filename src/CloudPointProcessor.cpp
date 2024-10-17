#include "CloudPointProcessor.hpp"
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <sstream>
#include <string>

#ifdef __APPLE__
	#include <sys/_pthread/_pthread_t.h>
#elif __linux__ 
	#include <pthread.h>
#else 
	#error "Platform not suported"
#endif

#include <vector>
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
	: file(filename) {
		pthread_mutex_init(&fileMutex, nullptr);
	}

// Destroy mutex and free file so it can be read by another process
CloudPointProcessor::~CloudPointProcessor() {
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

void CloudPointProcessor::processLine(const std::string &line,
		std::vector<Point>& points) {

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
