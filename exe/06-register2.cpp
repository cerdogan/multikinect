#include <thread>
#include <vector>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef pcl::visualization::PCLVisualizer Viewer;
pthread_mutex_t lock, lock2;
std::vector <Cloud::Ptr> cs;

/* ******************************************************************************************** */
struct SimpleOpenNIViewer {
	int id;
	SimpleOpenNIViewer (int i_) : id(i_) { }

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    pthread_mutex_lock(&lock);
		cs[id-1] = cloud->makeShared();
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
Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
		*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	double dd = 0.005;
  if (event.getKeySym () == "a" && event.keyDown ()) T2(0,3) += dd;
  if (event.getKeySym () == "d" && event.keyDown ()) T2(0,3) -= dd;
  if (event.getKeySym () == "w" && event.keyDown ()) T2(1,3) += dd;
  if (event.getKeySym () == "s" && event.keyDown ()) T2(1,3) -= dd;
  if (event.getKeySym () == "j" && event.keyDown ()) { 
		printf("\nasdf\nasdf\nasdf\n"); 
		T2(2,3) += dd;
	}
  if (event.getKeySym () == "l" && event.keyDown ()) T2(2,3) -= dd;
}

/* ******************************************************************************************** */
Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
int main () {
	
	// Compute the transformation
	T.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI_2, Eigen::Vector3d(0,1,0)).matrix();
	T2.block<3,3>(0,0) = Eigen::AngleAxis<double>(M_PI_2, Eigen::Vector3d(0,1,0)).matrix();
	T2(0,3) = -0.94;
	T2(1,3) = 0.05;
	T2(2,3) = 1.07;

	for(int i = 0; i < 4; i++) cs.push_back(Cloud::Ptr(new Cloud));
	pthread_mutex_init(&lock, NULL);
	pthread_mutex_init(&lock2, NULL);
	SimpleOpenNIViewer v1(1), v2(2), v3(3), v4(4);
	v1.init();
	v2.init();
	v3.init();
	v4.init();

	// Create the viewer
	char buf [256];
	sprintf(buf, "Kinect %d", id);
	Viewer* viewer = new Viewer (buf);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample");

	// Visualize
	int counter = 0;
	while(true) {
    pthread_mutex_lock(&lock2);
		T = T2;
		if(counter++ % 4 == 0) cout << "====== " << counter << " ==============================================\nT: " << T << endl;
    pthread_mutex_unlock(&lock2);
    pthread_mutex_lock(&lock);
		Cloud::Ptr c (new Cloud);
		Cloud::Ptr c1, c2,c3,c4;
		c1 = cs[0];
		c2 = cs[1];
		c3 = cs[2];
		c4 = cs[3];
		
		for (int h=0; h < c1->height; h++) {
			for (int w=0; w < c1->width; w++) {
				pcl::PointXYZRGBA point = c1->at(w, h);
				if(point.x != point.x) continue;
				if(point.z > 1.5) continue;
				c->push_back(point);
			}
		}

	for (int h=0; h < c3->height; h++) {
			for (int w=0; w < c3->width; w++) {
				pcl::PointXYZRGBA point = c3->at(w, h);
				if(point.x != point.x) continue;
				if(point.z > 1.5) continue;
				c->push_back(point);
			}
		}
	for (int h=0; h < c4->height; h++) {
			for (int w=0; w < c4->width; w++) {
				pcl::PointXYZRGBA point = c4->at(w, h);
				if(point.x != point.x) continue;
				if(point.z > 1.5) continue;
				c->push_back(point);
			}
		}



		// Transform the second cloud
		for (int h=0; h < c2->height; h++) {
			for (int w=0; w < c2->width; w++) {
		//for (int h=50; h < 60; h++) {
		//	for (int w=50; w < 60; w++) {
				pcl::PointXYZRGBA point = c2->at(w, h);
				//printf("orig point: (%lf, %lf, %lf)\n", point.x, point.y, point.z);
				if(point.x != point.x) continue;
				if(point.z > 1.5) continue;
				Eigen::Vector4d p (point.x, point.y, point.z, 1);
				Eigen::Vector4d Tp = T * p;
				point.x = Tp(0); point.y = Tp(1); point.z = Tp(2);
				// c2.at(w,h) = point;
				c->push_back(point);
				//printf("adding point: (%lf, %lf, %lf)\n", point.x, point.y, point.z);
			}
		}

//		c += c2;
    pthread_mutex_unlock(&lock);

		viewer->removePointCloud();
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(c);
		viewer->addPointCloud(c, rgb);
		viewer->spinOnce();
	//	printf("%d\n", c.size());
	}
}
