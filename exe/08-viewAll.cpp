#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <thread>
#include <vector>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <set>
#include <queue>

#include <Eigen/Dense>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
Cloud::Ptr c1, c2, c3, c4;
cv::Mat im;
int counter = 1;
std::string folder;

/* ******************************************************************************************** */
void makeImage () {

	for (int h=0; h<2*c1->height; h++) 
		for (int w=0; w<2*c1->width; w++) 
			im.at<cv::Vec3b>(h,w) = cv::Vec3b(0,0,0);
		
	for (int h=0; h<c1->height; h++) {
		for (int w=0; w<c1->width; w++) {
			pcl::PointXYZRGBA point = c1->at(w, h);
			Eigen::Vector3i rgb = point.getRGBVector3i();
			im.at<cv::Vec3b>(h,w)[0] = rgb[2];
			im.at<cv::Vec3b>(h,w)[1] = rgb[1];
			im.at<cv::Vec3b>(h,w)[2] = rgb[0];
		}
	}

	for (int h=0; h<c2->height; h++) {
		for (int w=0; w<c2->width; w++) {
			pcl::PointXYZRGBA point = c2->at(w, h);
			Eigen::Vector3i rgb = point.getRGBVector3i();
			im.at<cv::Vec3b>(h,w+c2->width)[0] = rgb[2];
			im.at<cv::Vec3b>(h,w+c2->width)[1] = rgb[1];
			im.at<cv::Vec3b>(h,w+c2->width)[2] = rgb[0];
		}
	}

	for (int h=0; h<c3->height; h++) {
		for (int w=0; w<c3->width; w++) {
			pcl::PointXYZRGBA point = c3->at(w, h);
			Eigen::Vector3i rgb = point.getRGBVector3i();
			im.at<cv::Vec3b>(h+c3->height,w)[0] = rgb[2];
			im.at<cv::Vec3b>(h+c3->height,w)[1] = rgb[1];
			im.at<cv::Vec3b>(h+c3->height,w)[2] = rgb[0];
		}
	}

	for (int h=0; h<c4->height; h++) {
		for (int w=0; w<c4->width; w++) {
			pcl::PointXYZRGBA point = c4->at(w, h);
			Eigen::Vector3i rgb = point.getRGBVector3i();
			im.at<cv::Vec3b>(h+c4->height,w+c4->width)[0] = rgb[2];
			im.at<cv::Vec3b>(h+c4->height,w+c4->width)[1] = rgb[1];
			im.at<cv::Vec3b>(h+c4->height,w+c4->width)[2] = rgb[0];
		}
	}

	cv::imshow("image", im);
	cv::waitKey(1);
}

/* ******************************************************************************************** */
bool checkSame (Cloud::Ptr c1, Cloud::Ptr c2) {

	for (int h=0; h<8; h++) {
		for (int w=0; w<8; w++) {
			pcl::PointXYZRGBA point1 = c1->at(w, h);
			Eigen::Vector3i rgb1 = point1.getRGBVector3i();
			pcl::PointXYZRGBA point2 = c2->at(w, h);
			Eigen::Vector3i rgb2 = point2.getRGBVector3i();
			if(rgb1(0) != rgb2(0)) return false;
			if(rgb1(1) != rgb2(1)) return false;
			if(rgb1(2) != rgb2(2)) return false;
		}
	}
	return true;
}

/* ******************************************************************************************** */
void readData () {
		char buf1 [256];
		char buf2 [256];
		char buf3 [256];
		char buf4 [256];
		sprintf(buf1, "%skin1-count%04d", folder.c_str(), counter);
		sprintf(buf2, "%skin2-count%04d", folder.c_str(), counter);
		sprintf(buf3, "%skin3-count%04d", folder.c_str(), counter);
		sprintf(buf4, "%skin4-count%04d", folder.c_str(), counter);
		printf("'%s' '%s' '%s' '%s'\n", buf1, buf2, buf3, buf4);
		int res1 = pcl::io::loadPCDFile<pcl::PointXYZRGBA> (buf1, *c1);
		int res2 = pcl::io::loadPCDFile<pcl::PointXYZRGBA> (buf2, *c2);
		int res3 = pcl::io::loadPCDFile<pcl::PointXYZRGBA> (buf3, *c3);
		int res4 = pcl::io::loadPCDFile<pcl::PointXYZRGBA> (buf4, *c4);
		if((res1 == -1) || (res1 == -2) || (res1 == -3) || (res4 == -1)) {
			printf("Failed read at count %d\n", counter); fflush(stdout);
			counter++;
			readData();
			return;
		}
		if(checkSame(c1,c2) || checkSame(c1,c3) || checkSame(c1,c4) || checkSame(c2,c3) || checkSame(c2,c4) || checkSame(c3,c4)) {
			printf("Same data at count %d\n", counter); fflush(stdout);
			counter++;
			readData();
			return;
		}
}

/* ******************************************************************************************** */
static void interact(int event, int x, int y, int flags, void *param) {

	if(event == CV_EVENT_LBUTTONDOWN) {
		printf("hi\n");
		counter++;
		readData();
		makeImage();
	}
}


/* ******************************************************************************************** */
int main (int argc, char* argv[]) {

	assert(argc > 1);
	folder = std::string(argv[1]);

	// Read the clouds
  c1 = Cloud::Ptr (new Cloud);
  c2 = Cloud::Ptr (new Cloud);
  c3 = Cloud::Ptr (new Cloud);
  c4 = Cloud::Ptr (new Cloud);

	cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("image", interact, NULL);
	im = cv::Mat(960, 1280, CV_8UC3);
	readData();
	while(true) {
//		printf("hi\n");
//		if(counter++ % 1 == 0) readData();
		makeImage();
		usleep(1e4);
	}
	return 0;
}
/* ******************************************************************************************** */
