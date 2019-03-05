#include <pcl/segmentation/supervoxel_clustering.h>

typedef pcl::PointXYZRGBA PointT;
typedef std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> OutT;

OutT foo(pcl::SupervoxelClustering<PointT>& ctr) {
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> out;
    ctr.extract(out);
    return out;
}
