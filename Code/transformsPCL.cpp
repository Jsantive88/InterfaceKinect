#include "transformsPCL.h"

void minCloudDepth (PointCloud<PointXYZRGBA>::ConstPtr &cloud) {
	float x_max = 0.0, y_max = 0.0, z_max = 0;
	int max = 0;

	for (size_t i= 0; i < cloud->points.size(); i++) { 

		if (!pcl_isnan (cloud->points[i].x) || !pcl_isnan (cloud->points[i].y) || !pcl_isnan (cloud->points[i].z)) {
					
			if (z_max < cloud->points[i].z && cloud->points[i].z > 0) {
				x_max = cloud->points[i].x; y_max = cloud->points[i].y;  z_max = cloud->points[i].z;
				max = i;
			}
		}
	}
		

	// Print point with low depth
	cout << "This is Point with less depth: ";
	cout << max << " with coords: " << endl;
	cout << "  X: "<< x_max << "  Y: "<< y_max << "  Z: "<< z_max << endl << endl;
 
	// reset variables
	x_max = 0.0; y_max = 0.0; z_max = 0.0, max = 0;
}


void rectaRegre (PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointXYZ pointJoin){

}

PointXYZ centralSection (PointCloud<PointXYZRGBA>::ConstPtr &section) {

	return;
}