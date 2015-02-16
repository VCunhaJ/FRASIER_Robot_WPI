#ifndef SURFACE_RECONSTRUCTION_H_
#define SURFACE_RECONSTRUCTION_H_

#include<FRASIER/frasier_main.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

using namespace std;
using namespace ros;
using namespace pcl;



/*Subscriber and Publisher*/
Subscriber subSegmentedObjects;
Publisher pubSmoothedSurfaces;

/*Object for sorting clusters*/
PointCloud<PointXYZRGB>::Ptr segmentedObjectsPtr (new PointCloud<PointXYZRGB>); //object to store cloud
PointCloud<PointNormal>::Ptr smoothSurfacePtr (new PointCloud<PointNormal>); //smoothed cloud output
MovingLeastSquares<PointXYZRGB, PointNormal> reconstructFilter; //object to perform smoothing
search::KdTree<PointXYZRGB>::Ptr kdTreePtr;

sensor_msgs::PointCloud2 objectReconstructed;


/*Thresholds*/
double neighborRadius = 0.03;


#endif
