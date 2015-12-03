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
Eigen::Matrix4d generateFrame (const std::string& filename) {

	// Read the points
	std::ifstream file (filename);	
	Eigen::Vector3d p1, p2, p3;
	for(int i = 0; i < 3; i++) file >> p1(i);
	for(int i = 0; i < 3; i++) file >> p2(i);
	for(int i = 0; i < 3; i++) file >> p3(i);
	file.close();

	cout << "p1: " << p1.transpose() << endl;
	cout << "p2: " << p2.transpose() << endl;
	cout << "p3: " << p3.transpose() << endl;
	printf("d12: %lf, d13: %lf, d23: %lf\n", (p2-p1).norm(), (p3-p1).norm(), (p2-p3).norm());

	// Create the rotation matrix
	Eigen::Vector3d v21 = (p2 - p1).normalized(), v31 = (p3 - p1).normalized();
	Eigen::Vector3d vLast = v21.cross(v31);
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.block<3,1>(0,0) = v21;
	T.block<3,1>(0,1) = v31;
	T.block<3,1>(0,2) = vLast;

	// Set the translation
	T.block<3,1>(0,3) = p1;
	return T;
}

/* ******************************************************************************************** */
int main () {
	
	// Compute the transformation
	Eigen::Matrix4d T1 = generateFrame ("../data/tennis1");
	Eigen::Matrix4d T2 = generateFrame ("../data/tennis2");
	exit(0);
	Eigen::Matrix4d T = T2.inverse() * T1; //Eigen::Matrix4d::Identity();
//	T(2,3) = 1;

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
    pthread_mutex_lock(&lock);
		Cloud c, c2;
		c = cs[0];
		c2 = cs[1];
		
		// Transform the second cloud
		if(c2.isOrganized() && c2.size() > 0) {
			for (int h=0; h<c2.height; h++) {
				for (int w=0; w<c2.width; w++) {
					pcl::PointXYZRGBA point = c2.at(w, h);
					Eigen::Vector4d p (point.x, point.y, point.z, 1);
					Eigen::Vector4d Tp = T * p;
					point.x = p(0); point.y = p(1); point.z = p(2);
					c2.at(w,h) = point;
				}
			}
		}

		c += c2;
    pthread_mutex_unlock(&lock);

		viewer->removePointCloud();
		Cloud::Ptr cp = c.makeShared();
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cp);
		viewer->addPointCloud(cp, rgb);
		viewer->spinOnce();
	//	printf("%d\n", c.size());
	}
}
