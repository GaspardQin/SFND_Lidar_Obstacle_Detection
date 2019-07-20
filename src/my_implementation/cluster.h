#include "kdtree.h"

template <typename PointT>
class MyEuclideanClusterExtraction{
private:
    typename pcl::PointCloud<PointT>::ConstPtr cloud;
    std::unique_ptr<KdTree> tree;
    std::vector<bool> processed;
    float distanceTol;
    int minSize, maxSize;

    void proximity(int id, std::vector<int>& cluster){
		if(processed[id])
			return;
		processed[id] = true;
		cluster.push_back(id);

        std::vector<float> pointData(3);
        pointData[0] = cloud->points[id].x;
        pointData[1] = cloud->points[id].y;
        pointData[2] = cloud->points[id].z;
		std::vector<int> neighbors = tree->search(pointData, distanceTol);
		for(auto neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt){
			proximity(*neighborIt, cluster);
		}
	}
public:
    MyEuclideanClusterExtraction(){
    }
    ~MyEuclideanClusterExtraction(){
    }
    void setClusterTolerance(float tolerance){
        distanceTol = tolerance;
    }
    void setMinClusterSize(int minSizeInput){
        minSize = minSizeInput;
    }
    void setMaxClusterSize(int maxSizeInput){
        maxSize = maxSizeInput;
    }
    void setInputCloud(typename pcl::PointCloud<PointT>::ConstPtr inputCloud){
        cloud = inputCloud;
        processed.clear();
        processed.resize(cloud->points.size(), false);
        tree.reset(new KdTree(3));

        int id=0;
        for(auto pointIter=cloud->points.begin(); pointIter != cloud->points.end(); ++pointIter, ++id){
            std::vector<float> pointData(3);
            pointData[0] = pointIter->x;
            pointData[1] = pointIter->y;
            pointData[2] = pointIter->z;
            tree->insert(pointData, id);
        }
    }
    
    void extract(std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters){
        clusters.clear();
        const int numCloudPoints = static_cast<int>(cloud->points.size());
        for(int i=0; i< numCloudPoints; i++){
            if(processed[i]) continue;
            
            // obtain indice for the clustered obstacle
            std::vector<int> clusterIndice;
            proximity(i, clusterIndice);
            
            if(clusterIndice.size() < minSize || clusterIndice.size() > maxSize) continue;

            // generate point cloud for clustered obstacle
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            clusterCloud->points.reserve(clusterIndice.size());
            for (auto pit = clusterIndice.begin (); pit != clusterIndice.end (); ++pit)
                clusterCloud->points.push_back(cloud->points[*pit]); 
            clusterCloud->width = clusterCloud->points.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            clusters.push_back(clusterCloud);
        }
    }
};
