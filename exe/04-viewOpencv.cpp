/**
 * @file can01_labelPoints.cpp
 * @author Can Erdogan
 * @date Oct 31, 2012
 * @brief This file creates an interface for a user to label the images of a 
 * cube at a given location and outputs a file which has the registered pixel 
 * coordinates. The user has to enter a value from 1 to 8 for each corner
 * of the cube (rectangular prism). The images must be named as "%d.JPG" and
 * start from 1.
 */

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <thread>
#include <vector>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

int id;
pthread_mutex_t lock;

/* ******************************************************************************************** */
struct SimpleOpenNIViewer {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr;
	cv::Mat im;

	/* ------------------------------------------------------------------------------------------ */
	SimpleOpenNIViewer () { 
		point_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	}

	/* ------------------------------------------------------------------------------------------ */
	void makeImage () {

		// Compute the maximum depth
		double maxDepth = -1, minDepth = 10000000;
		for (int h=0; h<point_cloud_ptr->height; h++) {
			for (int w=0; w<point_cloud_ptr->width; w++) {
				pcl::PointXYZRGBA point = point_cloud_ptr->at(w, h);
				if(point.z > maxDepth) maxDepth = point.z;
				if(point.z < minDepth) minDepth = point.z;
			}
		}

		// Create the image
		im = cv::Mat(point_cloud_ptr->height, 2*point_cloud_ptr->width, CV_8UC3);
		if (point_cloud_ptr->empty()) return;

		for (int h=0; h<im.rows; h++) {
			for (int w=0; w<point_cloud_ptr->width; w++) {
				pcl::PointXYZRGBA point = point_cloud_ptr->at(w, h);
				Eigen::Vector3i rgb = point.getRGBVector3i();
				im.at<cv::Vec3b>(h,w)[0] = rgb[2];
				im.at<cv::Vec3b>(h,w)[1] = rgb[1];
				im.at<cv::Vec3b>(h,w)[2] = rgb[0];
			}
			for (int w=0; w<point_cloud_ptr->width; w++) {
				pcl::PointXYZRGBA point = point_cloud_ptr->at(w, h);
				int val = (int) (255 * ((point.z-minDepth)/(maxDepth-minDepth)));
				im.at<cv::Vec3b>(h,w+point_cloud_ptr->width)[0] = val;
				im.at<cv::Vec3b>(h,w+point_cloud_ptr->width)[1] = val;
				im.at<cv::Vec3b>(h,w+point_cloud_ptr->width)[2] = val;
			}

		}

		cv::imshow("image", im);
		cv::waitKey(1);
	}

	/* ------------------------------------------------------------------------------------------ */
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		pthread_mutex_lock(&lock);
		point_cloud_ptr = cloud->makeShared();
		pthread_mutex_unlock(&lock);
	}

	/* ------------------------------------------------------------------------------------------ */
	void run () {

		// Create the interface to the Kinect
		char buf [256];
		sprintf(buf, "#%d", id);
		pcl::Grabber* interface = new pcl::OpenNIGrabber(buf);
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		// Start the interface
		interface->registerCallback (f);
		interface->start ();

		// Wait
		while (true) { //(!viewer.wasStopped()) {
			pthread_mutex_lock(&lock);
			makeImage();
			pthread_mutex_unlock(&lock);
			usleep(1e4);
		}

		// Stop the interface
		interface->stop ();
	}

};

/* ******************************************************************************************** */
int main (int argc, char* argv[]) {
	assert(argc > 1);
	id = (atoi(argv[1]));
	pthread_mutex_init(&lock, NULL);
	SimpleOpenNIViewer v;
	v.run ();
	return 0;
}
/* ******************************************************************************************** */
