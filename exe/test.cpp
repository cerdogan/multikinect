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
bool visSecond = true;
bool frameOn = true;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
		*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	static double dd = 0.05/2.0;
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
  if (event.getKeySym () == "b" && event.keyDown ()) { printf("Second flipped!\n"); visSecond = !visSecond; }
  if (event.getKeySym () == "x" && event.keyDown ()) {
		if(frameOn) viewer->removeCoordinateSystem ();
		else viewer->addCoordinateSystem (1.0);
		frameOn = !frameOn;
	}
}

/* ******************************************************************************************** */
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (Cloud::ConstPtr cloud) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}

/* ******************************************************************************************** */
int main (int argc, char** argv) {

	// Compute the transformation
	T1.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix();
	T2.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix() * 
		Eigen::AngleAxis<double>(-M_PI_2, Eigen::Vector3d(0,1,0)).matrix();
	// T2.block<3,1>(0,3) = Eigen::Vector3d(1.825, 0.15, 1.975);
	T2.block<3,1>(0,3) = Eigen::Vector3d(-2.575, -0.35, 2.0375);
	T3.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix() * 
		Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,1,0)).matrix();
	// T3.block<3,1>(0,3) = Eigen::Vector3d(0.15, 0.15, 4.3);
	T3.block<3,1>(0,3) = Eigen::Vector3d(0.175, 0.175, 4.225);
	T4.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix() * 
		Eigen::AngleAxis<double>(3*M_PI_2, Eigen::Vector3d(0,1,0)).matrix();
	T4.block<3,1>(0,3) = Eigen::Vector3d(0.0, 0.0, 0.0);

  Cloud::Ptr c1 (new Cloud);
  Cloud::Ptr c2 (new Cloud);
  Cloud::Ptr c3 (new Cloud);
  Cloud::Ptr c4 (new Cloud);
	assert(pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("kin3", *c1) != -1);
	assert(pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("kin1", *c2) != -1);
	assert(pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("kin2", *c3) != -1);
//	assert(pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("kin4", *c4) != -1);

	double maxDist1 = atof(argv[1]); //6.5;
	double minDist1 = atof(argv[2]); //6.5;
	double maxDist2 = atof(argv[3]); //6.5;
	double minDist2 = atof(argv[4]); //6.5;
	double maxDist3 = atof(argv[5]); //6.5;
	double minDist3 = atof(argv[6]); //6.5;

	

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbVis(Cloud::Ptr(new Cloud));

  while (!viewer->wasStopped ()) {

		Cloud::Ptr point_cloud_ptr (new Cloud);

		// First cloud
		for (int h=0; h < c1->height; h++) {
			for (int w=0; w < c1->width; w++) {
				pcl::PointXYZRGBA point = c1->at(w, h);
				if(point.x != point.x) continue;
				if(fabs(point.x) > 0.5) continue;
				if(point.z > maxDist1) continue;
				if(point.z < minDist1) continue;
				Eigen::Vector4d p (point.x, point.y, point.z, 1);
				Eigen::Vector4d Tp = T1 * p;
				point.x = Tp(0); point.y = Tp(1); point.z = Tp(2);
				point_cloud_ptr->push_back(point);
			}
		}

		// Second cloud
		if(visSecond && (viewAll || (Tc == &T2))) {
			cout << "T2: \n" << T2 << endl;
			for (int h=0; h < c2->height; h++) {
				for (int w=0; w < c2->width; w++) {
					pcl::PointXYZRGBA point = c2->at(w, h);
					if(point.x != point.x) continue;
					if(fabs(point.x) > 0.5) continue;
					if(point.z > maxDist2) continue;
					if(point.z < minDist2) continue;
					Eigen::Vector4d p (point.x, point.y, point.z, 1);
					Eigen::Vector4d Tp = T2 * p;
					point.x = Tp(0); point.y = Tp(1); point.z = Tp(2);
					point_cloud_ptr->push_back(point);
				}
			}
		}

		// Third cloud
		if(viewAll || (Tc == &T3)) {
			cout << "T3: \n" << T3 << endl;
			for (int h=0; h < c3->height; h++) {
				for (int w=0; w < c3->width; w++) {
					pcl::PointXYZRGBA point = c3->at(w, h);
					if(point.x != point.x) continue;
					if(fabs(point.x) > 0.5) continue;
					if(point.z > maxDist3) continue;
					if(point.z < minDist3) continue;
					Eigen::Vector4d p (point.x, point.y, point.z, 1);
					Eigen::Vector4d Tp = T3 * p;
					point.x = Tp(0); point.y = Tp(1); point.z = Tp(2);
					point_cloud_ptr->push_back(point);
				}
			}
		}

		// Fourth cloud
//		if(viewAll || (Tc == &T4)) {
//			cout << "T4: \n" << T4 << endl;
//			for (int h=0; h < c4->height; h++) {
//				for (int w=0; w < c4->width; w++) {
//					pcl::PointXYZRGBA point = c4->at(w, h);
//					if(point.x != point.x) continue;
//					if(point.z > distLimit) continue;
//					Eigen::Vector4d p (point.x, point.y, point.z, 1);
//					Eigen::Vector4d Tp = T4 * p;
//					point.x = Tp(0); point.y = Tp(1); point.z = Tp(2);
//					point_cloud_ptr->push_back(point);
//				}
//			}
//		}

		viewer->removePointCloud("sample cloud");
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(point_cloud_ptr);
		viewer->addPointCloud<pcl::PointXYZRGBA> (point_cloud_ptr, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  }
}
