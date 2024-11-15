#ifndef CLICK_HANDLER_H
#define CLICK_HANDLER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ClickHandler {

	public: 
		ClickHandler(pcl::visualization::PCLVisualizer::Ptr viewer, 
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr depthMap);	
		void initialize();		
		

		std::vector<pcl::PointXYZRGBNormal> getSelectedPoints() const;



	private:
		void pointPickingCallback(const pcl::visualization::PointPickingEvent& event);
		void keyboardCallback(const pcl::visualization::KeyboardEvent& kbdEvent);
		
		void drawPlane();

		pcl::visualization::PCLVisualizer::Ptr viewer_;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr depthMap;
		std::vector<pcl::PointXYZRGBNormal> selectedPoints;

};

#endif // CLICK_HANDLER_H
