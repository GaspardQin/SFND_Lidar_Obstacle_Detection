/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	size_t cloudSize = cloud->points.size();
	for(size_t iter_i=0; iter_i<maxIterations; iter_i++){
		// Randomly sample subset and fit the plane
		unsigned int index_1 = rand() % cloudSize;
		unsigned int index_2 = rand() % cloudSize;
		unsigned int index_3 = rand() % cloudSize;
		auto point_1 = cloud->points[index_1];
		auto point_2 = cloud->points[index_2];
		auto point_3 = cloud->points[index_3];
		if(index_1 == index_2 || index_1 == index_3 || index_2 == index_3){
			iter_i --;
			continue;
		}
		float a = (point_2.y - point_1.y)*(point_3.z - point_1.z) - (point_2.z - point_1.z)*(point_3.y - point_1.y);
		float b = (point_2.z - point_1.z)*(point_3.x - point_1.x) - (point_2.x - point_1.x)*(point_3.z - point_1.z);
		float c = (point_2.x - point_1.x)*(point_3.y - point_1.y) - (point_2.y - point_1.y)*(point_3.x - point_1.x);
		float d = -(a*point_1.x + b*point_1.y + c*point_1.z);

		std::unordered_set<int> inliersResult_i;
		for (size_t point_i = 0; point_i < cloudSize; point_i++)
		{	
			auto point = cloud->points[point_i];
			float distance = fabs(a * point.x + b * point.y + c * point.z + d)/(sqrt(a*a + b*b + c*c));
			if(distance < distanceTol){
				inliersResult_i.insert(point_i);
			}
		}
		if(inliersResult_i.size() > inliersResult.size()){
			inliersResult.swap(inliersResult_i);
			
		}
	}
	return inliersResult;

}
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	size_t cloudSize = cloud->points.size();
	for(size_t iter_i=0; iter_i<maxIterations; iter_i++){
		// Randomly sample subset and fit line
		unsigned int index_1 = rand() % cloudSize;
		unsigned int index_2 = rand() % cloudSize;
		float a = cloud->points[index_1].y - cloud->points[index_2].y;
		float b = cloud->points[index_2].x - cloud->points[index_1].x;	
		float c = cloud->points[index_1].x*cloud->points[index_2].y -  cloud->points[index_2].x*cloud->points[index_1].y;
		std::unordered_set<int> inliersResult_i;
		for (size_t point_i = 0; point_i < cloudSize; point_i++)
		{	
			float distance = fabs(a * cloud->points[point_i].x + cloud->points[point_i].y * b + c)/(sqrt(a*a + b*b));
			if(distance < distanceTol){
				inliersResult_i.insert(point_i);
			}
		}
		if(inliersResult_i.size() > inliersResult.size()){
			inliersResult.swap(inliersResult_i);
			
		}
		
	}
	std::cout<<"the inlier's size: "<<inliersResult.size()<<std::endl;
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 30, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
