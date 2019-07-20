#include <pcl/common/common.h>
template <typename PointT>
class MyRANSACPlanSegmentation{
private:
    float distanceThreshold;
    int maxIterations;
    typename pcl::PointCloud<PointT>::ConstPtr cloud;


public:
    void setDistanceThreshold(float distanceThresholdInput){
        distanceThreshold = distanceThresholdInput;
    }
    void setMaxIterations(int maxIterationsInput){
        maxIterations = maxIterationsInput;
    }
    void setInputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloudInput){
        cloud = cloudInput;
    }
    void segment(std::vector<int>& inliersIndiceOutput){
        std::vector<int> inliersResult;
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

            std::vector<int> inliersResult_i;
            for (size_t point_i = 0; point_i < cloudSize; point_i++)
            {	
                auto point = cloud->points[point_i];
                float distance = fabs(a * point.x + b * point.y + c * point.z + d)/(sqrt(a*a + b*b + c*c));
                if(distance < distanceThreshold){
                    inliersResult_i.push_back(point_i);
                }
            }
            if(inliersResult_i.size() > inliersResult.size()){
                inliersResult.swap(inliersResult_i);
                
            }
        }
        inliersIndiceOutput = inliersResult;
    }
    
};