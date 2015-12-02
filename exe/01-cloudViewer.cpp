 #include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>

int id;

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
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
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main (int argc, char* argv[])
 {
	assert(argc > 1);
	id = (atoi(argv[1]));
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
