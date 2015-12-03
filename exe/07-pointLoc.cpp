#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <thread>
#include <vector>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <set>
#include <queue>

#include <Eigen/Dense>

int id;
pthread_mutex_t lock, lock2;

/* ******************************************************************************************** */
struct SimpleOpenNIViewer {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr;
	cv::Mat im;

	/* ------------------------------------------------------------------------------------------ */
	SimpleOpenNIViewer () { 
		point_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	}

	/* ------------------------------------------------------------------------------------------ */
	static void interact(int event, int x, int y, int flags, void *param) {

		if(event == CV_EVENT_LBUTTONDOWN) {

			// Get the corner
			static int counter = 0;
			pthread_mutex_lock(&lock2);
			cv::Vec3s* coords = (cv::Vec3s*) param;
			coords[0] = cv::Vec3s(x,y,counter++); 
			pthread_mutex_unlock(&lock2);
		}
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
		im = cv::Mat(2*point_cloud_ptr->height, 3*point_cloud_ptr->width, CV_8UC3);
		if (point_cloud_ptr->empty()) return;

		for (int h=0; h<2*point_cloud_ptr->height; h++) {
			for (int w=0; w<2*point_cloud_ptr->width; w++) {
				pcl::PointXYZRGBA point = point_cloud_ptr->at(w/2, h/2);
				Eigen::Vector3i rgb = point.getRGBVector3i();
				im.at<cv::Vec3b>(h,w)[0] = rgb[2];
				im.at<cv::Vec3b>(h,w)[1] = rgb[1];
				im.at<cv::Vec3b>(h,w)[2] = rgb[0];
			}
			for (int w=0; w<point_cloud_ptr->width; w++) {
				pcl::PointXYZRGBA point = point_cloud_ptr->at(w, h/2);
				int val = (int) (255 * ((point.z-minDepth)/(maxDepth-minDepth)));
				im.at<cv::Vec3b>(h/2,w+2*point_cloud_ptr->width)[0] = val;
				im.at<cv::Vec3b>(h/2,w+2*point_cloud_ptr->width)[1] = val;
				im.at<cv::Vec3b>(h/2,w+2*point_cloud_ptr->width)[2] = val;
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

		// Set the callback function to get the points
		cv::Vec3s coords (0, 0, -1);
		cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("image", interact, (void*) (&coords));
		bool update = true;

		// Wait
		int counter = 0;
		int lastMouseInput = -1;
		Eigen::Vector3d sumLoc (0,0,0);
		while (true) { //(!viewer.wasStopped()) {

			// Create the image from cloud data
			pthread_mutex_lock(&lock);
			if(update) makeImage();
			pthread_mutex_unlock(&lock);

			// Get the user input if it exists and grow a region around it
			pthread_mutex_lock(&lock2);
			if((coords[2] > 0)) {

				// Get the mouse location
				pcl::PointXYZRGBA point = point_cloud_ptr->at(coords[0]/2, coords[1]/2);
				Eigen::Vector3d loc (point.x, point.y, point.z);
				for(int i = -2; i < 3; i++) 
					for(int j = -2; j < 3; j++) 
						im.at<cv::Vec3b>(coords[1]+j, coords[0]+i) = cv::Vec3b(0,0,0);
				cv::imshow("image", im);
				cv::waitKey(1);
				// cout << "loc: " << loc.transpose() << endl;

				// Check if a new point is chosen
				if(coords[2] > lastMouseInput) {
					counter = 0;
					sumLoc = Eigen::Vector3d(0,0,0);
					cout << "----------- reset" << endl;
					lastMouseInput = coords[2];
				}

				// Check if the point is valid, if so accumulate
				if((!(loc(0) != loc(0))) && loc.norm() > 1e-2) {
					counter++;
					sumLoc += loc;
				}

				// Check if reached sufficient number of samples
				if(counter >= 25) {
					cout << "mean loc: " << (sumLoc/25).transpose() << endl;
					counter = 0;
					sumLoc = Eigen::Vector3d(0,0,0);
				}
			}
			pthread_mutex_unlock(&lock2);
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
	pthread_mutex_init(&lock2, NULL);
	SimpleOpenNIViewer v;
	v.run ();
	return 0;
}
/* ******************************************************************************************** */
