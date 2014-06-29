// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

using namespace pcl;

PointXYZ minCloudDepth (PointCloud<PointXYZRGBA>::ConstPtr &cloud);

void rectaRegre (PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointXYZ pointJoin);

PointXYZ centralSection (PointCloud<PointXYZRGBA>::ConstPtr &section);