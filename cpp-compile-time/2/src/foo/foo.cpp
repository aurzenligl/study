#include <foo/foo.hpp>

#include <pcl/segmentation/supervoxel_clustering.h>

typedef pcl::PointXYZRGBA PointT;

pcl::PointXYZRGBA foo(float x, float y) {
    pcl::SupervoxelClustering<PointT> ctr(x, y);

    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> out;
    ctr.extract(out);

    pcl::PointXYZRGBA pt;
    out[42]->getCentroidPoint(pt);
    return pt;
}
