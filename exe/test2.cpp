/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

/* ******************************************************************************************** */
Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
Eigen::Matrix4d T3 = Eigen::Matrix4d::Identity();
Eigen::Matrix4d T4 = Eigen::Matrix4d::Identity();
Eigen::Matrix4d* Tc = &T2;
bool viewAll = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
		*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	static double dd = 0.05;
  if (event.getKeySym () == "r" && event.keyDown ()) {
		dd *= 2;
		printf("doubled dd to %lf\n", dd);
	}
  if (event.getKeySym () == "t" && event.keyDown ()) {
		dd /= 2.0;
		printf("halved dd to %lf\n", dd);
	}
  if (event.getKeySym () == "n" && event.keyDown ()) {
		printf("Current is 2.\n");
		Tc = &T2;
	}
  if (event.getKeySym () == "m" && event.keyDown ()) {
		printf("Current is 3.\n");
		Tc = &T3;
	}
  if (event.getKeySym () == "k" && event.keyDown ()) {
		printf("Current is 4.\n");
		Tc = &T4;
	}
  if (event.getKeySym () == "i" && event.keyDown ()) {
		printf("All is visualized.\n");
		viewAll = !viewAll;
	}
  if (event.getKeySym () == "a" && event.keyDown ()) (*Tc)(0,3) += dd;
  if (event.getKeySym () == "d" && event.keyDown ()) (*Tc)(0,3) -= dd;
  if (event.getKeySym () == "w" && event.keyDown ()) (*Tc)(1,3) += dd;
  if (event.getKeySym () == "s" && event.keyDown ()) (*Tc)(1,3) -= dd;
  if (event.getKeySym () == "j" && event.keyDown ()) (*Tc)(2,3) += dd;
  if (event.getKeySym () == "l" && event.keyDown ()) (*Tc)(2,3) -= dd;
}

/* ******************************************************************************************** */
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (Cloud::ConstPtr cloud) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

/* ******************************************************************************************** */
int main (int argc, char** argv) {

	// Compute the transformation
	T1.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix();
	T2.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix() * 
		Eigen::AngleAxis<double>(M_PI_2, Eigen::Vector3d(0,1,0)).matrix();
	T2.block<3,1>(0,3) = Eigen::Vector3d(2.05, 0.4, 2.55);
	T3.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix() * 
		Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,1,0)).matrix();
	T3.block<3,1>(0,3) = Eigen::Vector3d(0.0, 0.0, 0.0);
	T4.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix() * 
		Eigen::AngleAxis<double>(3*M_PI_2, Eigen::Vector3d(0,1,0)).matrix();
	T4.block<3,1>(0,3) = Eigen::Vector3d(0.0, 0.0, 0.0);

  Cloud::Ptr c1 (new Cloud);
  Cloud::Ptr c2 (new Cloud);
  Cloud::Ptr c3 (new Cloud);
  Cloud::Ptr c4 (new Cloud);
	assert(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *c1) != -1);

	double distLimit = 6.5;

	

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbVis(Cloud::Ptr(new Cloud));

  while (!viewer->wasStopped ()) {

		Cloud::Ptr point_cloud_ptr (new Cloud);

		// First cloud
		for (int h=0; h < c1->height; h++) {
			for (int w=0; w < c1->width; w++) {
				pcl::PointXYZRGBA point = c1->at(w, h);
				if(point.x != point.x) continue;
				if(point.z > atof(argv[2])) continue;
				if(fabs(point.x) > atof(argv[3])) continue;
				Eigen::Vector4d p (point.x, point.y, point.z, 1);
				Eigen::Vector4d Tp = T1 * p;
				point.x = Tp(0); point.y = Tp(1); point.z = Tp(2);
				point_cloud_ptr->push_back(point);
			}
		}

		viewer->removePointCloud("sample cloud");
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(point_cloud_ptr);
		viewer->addPointCloud<pcl::PointXYZRGBA> (point_cloud_ptr, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud");
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  }
}
