#include <thread>
#include <vector>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

int id;
pthread_mutex_t lock;

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
/* ******************************************************************************************** */
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis 
		(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) {

	char buf [256];
	sprintf(buf, "Kinect %d", id);
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer 
		(new pcl::visualization::PCLVisualizer (buf));
 viewer->setBackgroundColor (0, 0, 0);
 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
 viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
 viewer->addCoordinateSystem (1.0);
 viewer->initCameraParameters ();
 return (viewer);
}

/* ******************************************************************************************** */
struct SimpleOpenNIViewer {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr;

	/* ------------------------------------------------------------------------------------------ */
	SimpleOpenNIViewer () { 
		point_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		viewer = rgbVis(point_cloud_ptr);
	}

	/* ------------------------------------------------------------------------------------------ */
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		// if(!viewer->wasStopped()) viewer.showCloud (cloud);
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
			viewer->removePointCloud();
			 Cloud::Ptr c (new Cloud);
			Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
				T.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0,0,1)).matrix();
			for (int h=0; h < point_cloud_ptr->height; h++) {
				for (int w=0; w < point_cloud_ptr->width; w++) {
					pcl::PointXYZRGBA point = point_cloud_ptr->at(w, h);
					if(point.x != point.x) continue;
			//		if(point.z > 1.5) continue;
							Eigen::Vector4d p (point.x, point.y, point.z, 1);
							Eigen::Vector4d Tp = T * p;
							point.x = Tp(0); point.y = Tp(1); point.z = Tp(2);
					c->push_back(point);
				}
			}
		  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(c);
			viewer->addPointCloud<pcl::PointXYZRGBA> (c, rgb);
			viewer->spinOnce (1);
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
