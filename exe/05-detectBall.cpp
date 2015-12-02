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
	cv::Mat im, im_hsv;
	Eigen::Vector3d center;

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

		// Convert the image to hsv for processing later
  	cv::cvtColor(im, im_hsv, cv::COLOR_BGR2HSV);

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
	void growRegion (int px0, int py0) {

		static const int hLim = 20;
		static const int sLim = 80;
		static const int vLim = 30;
		static const double zLim = 0.40;

		// Initialize the list of visited pixels
		std::set <int> visited;
		int H = point_cloud_ptr->height, W = point_cloud_ptr->width;

		// Continue growing the region
		std::queue <int> pixels;
		pixels.push(W * py0 + px0);
		int ns [] = {0, 1, -1, 0, 0, -1, 1, 0};
		int numPixels = 0;
		std::vector <Eigen::VectorXd> points;
		while(!pixels.empty()) {


			// Get the pixel value and its color
			int val = pixels.front();
			pixels.pop();
			int pxc = val % W, pyc = val / W;
			// printf("new pix: (%d, %d)\n", pxc, pyc);
//			getchar();
			cv::Vec3b hsv_c = im_hsv.at<cv::Vec3b>(pyc, pxc);
			pcl::PointXYZRGBA pc = point_cloud_ptr->at(pxc, pyc);
			double zc = pc.z;
			numPixels++;
			if(pc.x != pc.x || pc.y != pc.y || pc.z != pc.z) continue;

			// Display the image
			im.at<cv::Vec3b>(pyc, pxc) = cv::Vec3b(0,0,255);

			// Look at the four corners
			for(int n_idx = 0; n_idx < 8; n_idx += 2) {
				
				// Get the neighbor pixel
				int px = pxc + ns[n_idx], py = pyc + ns[n_idx+1]; 
				cv::Vec3b hsv = im_hsv.at<cv::Vec3b>(py, px);
				cv::Vec3b rgb = im.at<cv::Vec3b>(py, px);
				pcl::PointXYZRGBA p = point_cloud_ptr->at(px, py);
				double z = p.z;
				if(p.x != p.x || p.y != p.y || p.z != p.z) continue;

				// Check if the neighbor is within limit
				// if((abs(hsv_c[0] - hsv[0]) < hLim) && (abs(hsv_c[1] - hsv[1]) < sLim) && 
				// 		(abs(hsv_c[2] - hsv[2]) < vLim) && (fabs(zc - z) < zLim)) {
				if((rgb[0] + rgb[1] + rgb[2] > 100) && (fabs(zc - z) < zLim)) {
					if(visited.find(W * py + px) == visited.end()) {
						visited.insert(W * py + px);
						pixels.push(W * py + px);
						Eigen::VectorXd temp (5);
						temp << p.x, p.y, p.z, px, py;
						// points.push_back(Eigen::Vector3d(p.x, p.y, p.z));
						points.push_back(temp);
						// printf("%lf\t%lf\t%lf\n", p.x, p.y, p.z);
						// printf("\tadding (%d, %d)\n", px, py);
					}
					// else printf("\tskipping (%d, %d)\n", px, py);
				}


			}

			if(numPixels > 1000) {
				printf("Reached limit...\n"); fflush(stdout);break;
			}
		}
		
		cv::imshow("image", im);
		cv::waitKey(1);
		// printf("#p: %d\n", numPixels);

		// Compute the sphere
		computeSphereCenter(points);
	}

	/* ------------------------------------------------------------------------------------------ */
	void computeSphereCenter (const std::vector <Eigen::VectorXd>& points) {

		// Create the A and b matrices
		Eigen::MatrixXd A (points.size(), 4);
		Eigen::VectorXd b (points.size());
		for(size_t i = 0; i < points.size(); i++) {
			A.block<1,3>(i,0) = points[i].block<3,1>(0,0);
			A(i,3) = 1;
			b(i) = -points[i].block<3,1>(0,0).squaredNorm();
		}

		// Compute x as in Ax=b
		Eigen::VectorXd x = ((A.transpose()*A).inverse())*(A.transpose()) * b;
		center = -x.block<3,1>(0,0)/2;

		//// Visualize the center
		//int bestx = 0, besty = 0;
		//double minDist = 1e9;
		//for(size_t i = 0; i < points.size(); i++) {
		//	double dist = (points[i].block<3,1>(0,0).normalized() - center.normalized()).squaredNorm();
		//	if(dist < minDist) {
		//		bestx = points[i](3);
		//		besty = points[i](4);
		//		minDist = dist;
		//	}
		//}
		//
		//printf("minDist: %lf, bestx: %d, besty: %d\n", minDist, bestx, besty);
		//im.at<cv::Vec3b>(besty, bestx) = cv::Vec3b(0,255,0);
		//im.at<cv::Vec3b>(besty+1, bestx-1) = cv::Vec3b(0,255,0);
		//im.at<cv::Vec3b>(besty+1, bestx+1) = cv::Vec3b(0,255,0);
		//im.at<cv::Vec3b>(besty-1, bestx-1) = cv::Vec3b(0,255,0);
		//im.at<cv::Vec3b>(besty-1, bestx+1) = cv::Vec3b(0,255,0);
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
		Eigen::Vector3d sumCenter (0,0,0);
		while (true) { //(!viewer.wasStopped()) {

			// Create the image from cloud data
			pthread_mutex_lock(&lock);
			if(update) makeImage();
			pthread_mutex_unlock(&lock);

			// Get the user input if it exists and grow a region around it
			pthread_mutex_lock(&lock2);
			if((coords[2] > 0)) {
				growRegion(coords[0], coords[1]);
				if(coords[2] > lastMouseInput) {
					counter = 0;
					sumCenter = Eigen::Vector3d(0,0,0);
					cout << "----------- reset" << endl;
					lastMouseInput = coords[2];
				}
				// coords[2] = -1;
				if((!(center(0) != center(0))) && center.norm() > 1e-2) {
					counter++;
					sumCenter += center;
				}
				if(counter >= 100) {
					counter = 0;
					cout << "mean center: " << (sumCenter/100).transpose() << endl;
					sumCenter = Eigen::Vector3d(0,0,0);
				}
				// update = false;
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
