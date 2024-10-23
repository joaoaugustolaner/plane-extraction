#ifndef CLICK_HANDLER_H
#define CLICK_HANDLER_H

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <vector>


class ClickHandler {

	public: 
		ClickHandler(const cv::Mat& image, const cv::Mat& depthMap);

		static void onMouse(int event, int x, int y, int flags, void* userdata);

		void start();

	private:
		cv::Mat image;
		cv::Mat depthMap;
		std::vector<cv::Point> selectedPoints;

		void displayPointInfo(int x, int y);
};

#endif // CLICK_HANDLER_H