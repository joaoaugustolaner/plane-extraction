#include "Draw.h"
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

Draw::Draw(pcl::visualization::PCLVisualizer::Ptr viewer)
    : viewer_(viewer) {}

Eigen::Vector3f Draw::computePlaneNormal(const Eigen::MatrixXf& points) {
    Eigen::Vector3f centroid = points.colwise().mean();
    Eigen::MatrixXf centered = points.rowwise() - centroid.transpose();
    Eigen::Matrix3f covariance = centered.transpose() * centered;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f normal = solver.eigenvectors().col(0); // Eigenvector with the smallest eigenvalue
    return normal;
}

void Draw::fitPlaneAndRender(const std::vector<pcl::PointXYZRGBNormal>& selectedPoints) {
    if (selectedPoints.size() < 3) {
        std::cerr << "At least 3 points are required to define a plane." << std::endl;
        return;
    }

    Eigen::MatrixXf points(selectedPoints.size(), 3);
    for (size_t i = 0; i < selectedPoints.size(); ++i) {
        points(i, 0) = selectedPoints[i].x;
        points(i, 1) = selectedPoints[i].y;
        points(i, 2) = selectedPoints[i].z;
    }
	
	planeNormal_ = computePlaneNormal(points);
    planeCentroid_ = points.colwise().mean();

    renderPlane(planeNormal_, planeCentroid_, selectedPoints);
	renderDipAndStrikeVectors();
}

Eigen::Vector3f Draw::getPlaneNormal() const{
	return this->planeNormal_;
}


Eigen::Vector3f Draw::getPlaneCentroid() const{
	return this->planeCentroid_;
}


void Draw::renderPlane(const Eigen::Vector3f& normal, const Eigen::Vector3f& centroid, const std::vector<pcl::PointXYZRGBNormal>& selectedPoints) {
	float planeSize = 0.3f; // Adjust the size of the plane as needed
    Eigen::Vector3f u = Eigen::Vector3f::UnitX().cross(normal).normalized();
    Eigen::Vector3f v = normal.cross(u).normalized();

    // Create four corners of the plane
    Eigen::Vector3f corner1 = centroid - planeSize * u - planeSize * v;
    Eigen::Vector3f corner2 = centroid + planeSize * u - planeSize * v;
    Eigen::Vector3f corner3 = centroid + planeSize * u + planeSize * v;
    Eigen::Vector3f corner4 = centroid - planeSize * u + planeSize * v;

    // Define the mesh vertices and triangles
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeVertices(new pcl::PointCloud<pcl::PointXYZ>);
    planeVertices->push_back(pcl::PointXYZ(corner1.x(), corner1.y(), corner1.z()));
    planeVertices->push_back(pcl::PointXYZ(corner2.x(), corner2.y(), corner2.z()));
    planeVertices->push_back(pcl::PointXYZ(corner3.x(), corner3.y(), corner3.z()));
    planeVertices->push_back(pcl::PointXYZ(corner4.x(), corner4.y(), corner4.z()));

    pcl::PolygonMesh planeMesh;
    planeMesh.polygons.resize(2);

    // First triangle (corner1, corner2, corner3)
    planeMesh.polygons[0].vertices = {0, 1, 2};

    // Second triangle (corner1, corner3, corner4)
    planeMesh.polygons[1].vertices = {0, 2, 3};

    // Convert point cloud to PCL format for mesh
    pcl::toPCLPointCloud2(*planeVertices, planeMesh.cloud);

    // Add the translucent plane to the viewer
    viewer_->addPolygonMesh(planeMesh, "translucent_plane");
    viewer_->setRepresentationToSurfaceForAllActors();
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "translucent_plane"); // Red color
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "translucent_plane"); // 50% opacity
    viewer_->spinOnce();
}

void Draw::renderDipAndStrikeVectors() {
    // Compute strike and dip vectors
    Eigen::Vector3f strikeVector = Eigen::Vector3f::UnitX().cross(planeNormal_).normalized();
    Eigen::Vector3f dipVector = planeNormal_.cross(strikeVector).normalized();

    float vectorScale = 0.5f; 
    Eigen::Vector3f strikeEnd = planeCentroid_ + vectorScale * strikeVector;
    Eigen::Vector3f dipEnd = planeCentroid_ + vectorScale * dipVector;

    // Render the strike vector
    viewer_->addLine(
        pcl::PointXYZ(planeCentroid_.x(), planeCentroid_.y(), planeCentroid_.z()),
        pcl::PointXYZ(strikeEnd.x(), strikeEnd.y(), strikeEnd.z()),
        0.0, 1.0, 0.0, "strike_vector" // Green line for strike
    );

    // Render the dip vector
   	renderThickVector(
        planeCentroid_, strikeEnd, "strike_cylinder", 0.01f, 0.0f, 1.0f, 0.0f // Green cylinder
    );

	renderThickVector(
        planeCentroid_, dipEnd, "dip_cylinder", 0.01f, 0.0f, 0.0f, 1.0f // Blue cylinder
    );

    viewer_->spinOnce();
}

void Draw::renderThickVector(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
                             const std::string& id, float radius, float r, float g, float b) {
    // Compute direction vector and length of the cylinder
    Eigen::Vector3f axis = end - start;
    float length = axis.norm();

    // Normalize the direction vector for cylinder orientation
    axis.normalize();

    // Define the cylinder's parameters
    pcl::ModelCoefficients cylinder;
    cylinder.values.resize(7); // x, y, z (center), dx, dy, dz (direction), radius
    cylinder.values[0] = start.x();
    cylinder.values[1] = start.y();
    cylinder.values[2] = start.z();
    cylinder.values[3] = axis.x();
    cylinder.values[4] = axis.y();
    cylinder.values[5] = axis.z();
    cylinder.values[6] = radius; // Thickness of the vector

    // Add the cylinder to the viewer
    viewer_->addCylinder(cylinder, id);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id); // Fully opaque
}


void Draw::projectPlaneToImage(const cv::Mat& image, const std::vector<pcl::PointXYZRGBNormal>& selectedPoints) {
    cv::Mat projectedImage= image.clone();
	cv::Mat overlay = cv::Mat::zeros(image.size(), image.type());
    
	const float X_MIN = -3.15, X_MAX = 3.15;
    const float Y_MIN = -0.8, Y_MAX = 1.1;

    // Define the grid size and resolution
    float planeSize = 0.3f; // Size of the plane
    float resolution = 0.001f; // Spacing between points in the grid

    Eigen::Vector3f u = Eigen::Vector3f::UnitX().cross(planeNormal_).normalized();
    Eigen::Vector3f v = planeNormal_.cross(u).normalized();

    // Iterate over grid points to generate plane
    for (float i = -planeSize; i <= planeSize; i += resolution) {
        for (float j = -planeSize; j <= planeSize; j += resolution) {
            Eigen::Vector3f point = planeCentroid_ + i * u + j * v;

            // Project to 2D image space
            int x = static_cast<int>(((point.x() - X_MIN) / (X_MAX - X_MIN)) * (image.cols - 1));
            int y = static_cast<int>(((point.y() - Y_MIN) / (Y_MAX - Y_MIN)) * (image.rows - 1));
            y = image.rows - 1 - y; // Flip Y-axis for OpenCV

            // Check bounds
            if (x >= 0 && x < overlay.cols && y >= 0 && y < overlay.rows) {
                overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red
            } else {
                std::cerr << "Invalid pixel: (" << x << ", " << y << ")\n";
            }
        }

    }
  	
	float dipAngle = std::acos(planeNormal_.z()) * 180.0f / M_PI; // Angle with Z-axis
    float strikeAngle = std::atan2(planeNormal_.y(), planeNormal_.x()) * 180.0f / M_PI;

	if (strikeAngle < 0) {
		strikeAngle += 360.0f;
	}

	std::string dipText = "Dip Angle: " + std::to_string(dipAngle) + " degrees";
    std::string strikeText = "Strike Angle: " + std::to_string(strikeAngle) + " degrees";
    std::string centroidText = "Centroid: [" + std::to_string(planeCentroid_.x()) + ", " +
                                std::to_string(planeCentroid_.y()) + ", " +
                                std::to_string(planeCentroid_.z()) + "]";


    float alpha = 0.3; // Adjust transparency (0.0 = fully transparent, 1.0 = opaque)
	
	if (!projectedImage.empty() && !overlay.empty()) {
        cv::addWeighted(overlay, alpha, projectedImage, 1 - alpha, 0, projectedImage);
    } else {
        std::cerr << "Error: Failed to blend overlay with image.\n";
        return;
    }

	cv::putText(projectedImage, dipText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(projectedImage, strikeText, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(projectedImage, centroidText, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    cv::imshow("Viewer", projectedImage);
    cv::waitKey(0);
}
