#include "transformsPCL.h"


/// <summary>
/// Obtenemos el punto minimo con depth de una nube de puntos.
/// </summary>
/// <returns>PointXYZ</returns>
PointXYZ minCloudDepth(PointCloud<PointXYZRGBA>::ConstPtr &cloud) {
	float x_max = 0.0, y_max = 0.0, z_max = 0;
	int max = 0;
	PointXYZ point;

	for (size_t i= 0; i < cloud->points.size(); i++) {
		if (!pcl_isnan(cloud->points[i].x) || !pcl_isnan(cloud->points[i].y) || !pcl_isnan(cloud->points[i].z)) {			
			if (z_max < cloud->points[i].z && cloud->points[i].z > 0) {
				x_max = cloud->points[i].x; y_max = cloud->points[i].y;  z_max = cloud->points[i].z;
				max = i;
			}
		}
	}
	
	// Print point with low depth
	printf("This is Point with less depth: %d\n\t With coords: X-> %f\tY-> %f\tZ-> %f\n", max, x_max, y_max, z_max);
	//cout << "This is Point with less depth: ";
	//cout << max << " with coords: " << endl;
	//cout << "  X: "<< x_max << "  Y: "<< y_max << "  Z: "<< z_max << endl << endl;
	point.x = x_max;
	point.y = y_max;
	point.z = z_max;

	// reset variables
	x_max = 0.0; y_max = 0.0; z_max = 0.0, max = 0;

	return point;
}

/// <summary>
/// 
/// </summary>
/// <returns>PointXYZ</returns>
PointXYZ centralSection(PointCloud<PointXYZRGBA>::ConstPtr &section, float pZ) {
	float x = 0.0, y = 0.0, z = 0;
	int num = 0;
	PointXYZ point;

	for (size_t i= 0; i < section->points.size(); i++) {
		if (!pcl_isnan(section->points[i].x) || !pcl_isnan(section->points[i].y) || !pcl_isnan(section->points[i].z)) {			
			if (section->points[i].z == pZ ) {
				x += section->points[i].x; y += section->points[i].y;  z += section->points[i].z;
				num += i;
			}
		}
	}

	// Print point with low depth
	printf("This is Point has %d points\n\t With coords: X-> %f\tY-> %f\tZ-> %f\n", num, x, y, z);

	point.x = x/num;
	point.y = y/num;
	point.z = z/num;

	x = 0.0; y = 0.0; z = 0.0, num = 0;

	return;
}

/// <summary>
/// 
/// </summary>
PointCloud<PointXYZ>* rectaRegre(PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointXYZ pointJoin){
	// Quizas seria mejor prepararla los puntos Z sobre los que iremos recorriendo las secciones por una recta desde el punto de depht mas profundo al joint de la mano.
	
	float z_joint = pointJoin.z;
	PointXYZ p_aux;
	PointCloud<PointXYZ> rect;
	rect.width    = 5;
	rect.height   = 1;
	rect.is_dense = false;
	rect.points.resize (rect.width * rect.height);

	for (size_t i= 0; i < cloud->points.size(); i++) {
		if (!pcl_isnan(cloud->points[i].x) || !pcl_isnan(cloud->points[i].y) || !pcl_isnan(cloud->points[i].z)) {			
			p_aux = centralSection(cloud, z_joint);
			rect.push_back(p_aux);
		}
	}
	
	return &rect;
}