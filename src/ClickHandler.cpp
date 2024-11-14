#include "ClickHandler.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <string>


ClickHandler::ClickHandler(pcl::visualization::PCLVisualizer::Ptr viewer)
    : viewer_(viewer){}

void ClickHandler::initialize() {
	if(viewer_){
		viewer_->registerPointPickingCallback(
				[this](const pcl::visualization::PointPickingEvent& event) {
					this->onPointPicked(event);
				}
		);
	}
}

void ClickHandler::onPointPicked(const pcl::visualization::PointPickingEvent& event) {
        if (event.getPointIndex() == -1) {
            return; 
        }
	
        float x, y, z;
        event.getPoint(x, y, z);
		
        viewer_->removeShape("highlighted_point");

        // Highlight the selected point with a sphere
        pcl::PointXYZ pickedPoint(x, y, z);
        viewer_->addSphere(pickedPoint, 0.2, 1.0, 0.0, 0.0, "highlighted_point");

        std::cout << "Selected point at (" << x << ", " << y << ", " << z << ")" << std::endl;
    }
