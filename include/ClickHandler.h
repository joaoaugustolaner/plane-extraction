#ifndef CLICK_HANDLER_H
#define CLICK_HANDLER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ClickHandler {

	public: 
		explicit ClickHandler(pcl::visualization::PCLVisualizer::Ptr viewer);	
		void initialize();		

	private:
		pcl::visualization::PCLVisualizer::Ptr viewer_;
		void onPointPicked(const pcl::visualization::PointPickingEvent& event);

		
};

#endif // CLICK_HANDLER_H
