#include "ClickHandler.h"
#include <functional>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>




ClickHandler::ClickHandler(pcl::visualization::PCLVisualizer::Ptr viewer, 
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr depthMap)
    : viewer_(viewer), depthMap(depthMap){}


void ClickHandler::initialize() {
	
    	viewer_->registerPointPickingCallback(
				std::bind(&ClickHandler::pointPickingCallback, this, std::placeholders::_1));
	
		viewer_->registerKeyboardCallback(
				std::bind(&ClickHandler::keyboardCallback, this, std::placeholders::_1));
		
}


std::vector<pcl::PointXYZRGBNormal> ClickHandler::getSelectedPoints() const {
    return this->selectedPoints;
}


void ClickHandler::pointPickingCallback(const pcl::visualization::PointPickingEvent& event) {

	if (!depthMap || depthMap->points.empty()) {
    	std::cerr << "Depth map is not initialized or empty!" << std::endl;
    	return;
	}


	if (event.getPointIndex() == -1) {
		std::cerr << "No point selected!";
		return;
	}
	
	int index = event.getPointIndex();

	pcl::PointXYZRGBNormal selectedPoint = 	this->depthMap->points[index];
	this->viewer_->addSphere(
			selectedPoint, 0.01, 1.0, 0.0, 0.0, "highlighted_point_" + std::to_string(index)
			);

	this->selectedPoints.push_back(selectedPoint);

	std::cout << "Selected point: (" << selectedPoint.x << ", " << selectedPoint.y << ", " << selectedPoint.z << ")\n";

	std::cout << "Selected Points size: " << this->selectedPoints.size() << "\n";

}

void ClickHandler::keyboardCallback(const pcl::visualization::KeyboardEvent& event) {

	if (event.keyDown() && event.getKeySym() == "Return") {
		std::cout << "Return pressed, selectedPoints.size() = " << this->selectedPoints.size() << "\n";

        if (this->selectedPoints.size() < 3){ 
			std::cerr << "At least 3 points are required to draw a plane!\n" << std::endl;
            return;
        }

        this->drawPlane();
    }
}

float calculateZ(float x, float y, const pcl::ModelCoefficientsPtr& planeCoefficients) {
    // Extract coefficients from the plane
    float a = planeCoefficients->values[0];
    float b = planeCoefficients->values[1];
    float c = planeCoefficients->values[2];
    float d = planeCoefficients->values[3];

    // Check if c is zero to avoid division by zero
    if (std::abs(c) < 1e-6) {
        throw std::runtime_error("Cannot calculate Z because the plane is vertical (c = 0)");
    }

    // Calculate z using the plane equation
    return -(a * x + b * y + d) / c;
}

void ClickHandler::drawPlane(){

	Eigen::Vector3d centroid(0,0,0);

	for (const auto& point : this->selectedPoints) {
        centroid += Eigen::Vector3d(point.x, point.y, point.z);
    }

	centroid /= this->selectedPoints.size();


	//Eigen values covariance matrix;
	Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
	
	for (const auto& point : this->selectedPoints) {
		Eigen::Vector3d p(point.x, point.y, point.z);
		covariance += (p-centroid) * (p-centroid).transpose();
	}

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
	Eigen::Vector3d normal = solver.eigenvectors().col(0);

	//Plane Coefficients
	double d = -normal.dot(centroid);

	pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
    planeCoefficients->values = {(float)normal[0], (float)normal[1], (float)normal[2], (float)d};

	float minX = std::numeric_limits<float>::max();
	float maxX = std::numeric_limits<float>::lowest();
	float minY = std::numeric_limits<float>::max();
	float maxY = std::numeric_limits<float>::lowest();

	for (const auto& point : this->selectedPoints) {
    	minX = std::min(minX, point.x);
    	maxX = std::max(maxX, point.x);
    	minY = std::min(minY, point.y);
    	maxY = std::max(maxY, point.y);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr planeCorners (new pcl::PointCloud<pcl::PointXYZ>);

	planeCorners->points.push_back({minX, minY, calculateZ(minX, minY, planeCoefficients)});
	planeCorners->points.push_back({maxX, minY, calculateZ(maxX, minY, planeCoefficients)});
	planeCorners->points.push_back({maxX, maxY, calculateZ(maxX, maxY, planeCoefficients)});
	planeCorners->points.push_back({minX, maxY, calculateZ(minX, maxY, planeCoefficients)});

	
	pcl::PolygonMesh mesh;
    mesh.polygons.resize(2);
	mesh.polygons[0].vertices = {0, 1, 2};
	mesh.polygons[1].vertices = {2, 3, 0};
	pcl::toPCLPointCloud2(*planeCorners, mesh.cloud);

	this->viewer_->addPolygonMesh(mesh, "filled_plane");

	this->viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "filled_plane");

	this->viewer_->spinOnce();

}
