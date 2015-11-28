#include <thread>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

class SimpleOpenNIViewer
{
	public:
		int id;
		SimpleOpenNIViewer (int i_, char* buf) : viewer(buf) {
			id = i_;
		}

		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
		{
			printf("%d\n", id);
			fflush(stdout);
//			if (!viewer.wasStopped())
//				viewer.showCloud (cloud);
		}

		void run ()
		{
		char buf [256];
		sprintf(buf, "#%d", id);
			pcl::Grabber* interface = new pcl::OpenNIGrabber(buf);

			boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
				boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

			interface->registerCallback (f);

			interface->start ();

			while(true) sleep(1);
//			while (!viewer.wasStopped()) {
//				boost::this_thread::sleep (boost::posix_time::seconds (1));
//			}

			interface->stop ();
		}

		pcl::visualization::CloudViewer viewer;
};

void threadFunc (int id) {
	char buf [256];
	sprintf(buf, "PCL OpenNI Viewer-%d", id);
	SimpleOpenNIViewer v(id, buf);
	v.run ();
}

int main () {
	
	std::thread k1 (threadFunc, 1);
	std::thread k2 (threadFunc, 2);
	k1.join();
	return 0;

}
