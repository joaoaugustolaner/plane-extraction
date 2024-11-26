#ifndef DRAW_H
#define DRAW_H

#include <opencv2/core/mat.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <vector>

class Draw {
public:
    Draw(pcl::visualization::PCLVisualizer::Ptr viewer);

    void fitPlaneAndRender(const std::vector<pcl::PointXYZRGBNormal>& selectedPoints);
	void projectPlaneToImage(const cv::Mat& image, const std::vector<pcl::PointXYZRGBNormal>& selectedPoints);
	

	Eigen::Vector3f getPlaneNormal() const;
    Eigen::Vector3f getPlaneCentroid() const;


private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
	
	Eigen::Vector3f planeNormal_;
    Eigen::Vector3f planeCentroid_;

    Eigen::Vector3f computePlaneNormal(const Eigen::MatrixXf& points);
    void renderPlane(const Eigen::Vector3f& normal, const Eigen::Vector3f& centroid, const std::vector<pcl::PointXYZRGBNormal>& selectedPoints);
	void renderDipAndStrikeVectors();
	void renderThickVector(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
                             const std::string& id, float radius, float r, float g, float b);
   };

#endif // DRAW_H
