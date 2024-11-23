#ifndef CLICKHANDLER_H
#define CLICKHANDLER_H


#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>



class ClickHandler {
public:
    ClickHandler(const cv::Mat& image, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr depthMap);

    void handleClicks(const std::string& windowName);
	
	bool getEnterStatus();

	std::vector<pcl::PointXYZRGBNormal> getSelectedPoints();

private:

    cv::Mat image_; 
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr depthMap_;
    cv::Mat displayedImage_;        
	std::vector<pcl::PointXYZRGBNormal> selectedPoints;

	pcl::PointXYZRGBNormal findCorrelatedPoint(int x, int y);
	bool enterPressed_ = false;

    void drawCircle(int x, int y);

    static void onMouse(int event, int x, int y, int flags, void* userdata);
};

#endif // CLICKHANDLER_H
