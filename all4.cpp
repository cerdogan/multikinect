#include <thread>
#include <vector>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef pcl::visualization::PCLVisualizer Viewer;
pthread_mutex_t lock;
std::vector <Cloud> cs;

/* ******************************************************************************************** */
struct SimpleOpenNIViewer {
	int id;
	SimpleOpenNIViewer (int i_) : id(i_) { }

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		Cloud c2;
		c2 += *cloud;
		std::vector<int> indices;
    pcl::removeNaNFromPointCloud(c2,c2,indices);
    pthread_mutex_lock(&lock);
		cs[id-1] = c2;
    pthread_mutex_unlock(&lock);
	}

	void init () {
		char buf [256];
		sprintf(buf, "#%d", id);
		pcl::Grabber* interface = new pcl::OpenNIGrabber(buf);
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
		interface->registerCallback (f);
		interface->start ();
	}
};

/* ******************************************************************************************** */
int main () {
	
	for(int i = 0; i < 4; i++) cs.push_back(Cloud());
	pthread_mutex_init(&lock, NULL);
	SimpleOpenNIViewer v1(1), v2(2);
	v1.init();
	v2.init();

	// Create the viewer
	Viewer* viewer = new Viewer ("3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample");

	// Visualize
	while(true) {
		Cloud c;
    pthread_mutex_lock(&lock);
		for(int i = 0; i < 2; i++) {
			printf("%d\t", cs[i].size());
			c += (cs[i]);
		}
		printf("\n");
    pthread_mutex_unlock(&lock);

		viewer->removePointCloud();
		Cloud::Ptr cp = c.makeShared();
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cp);
		viewer->addPointCloud(cp, rgb);
		viewer->spinOnce();
	//	printf("%d\n", c.size());
	}
}
