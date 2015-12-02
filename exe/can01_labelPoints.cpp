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
#include <stdio.h>

//#include "Common.h"

using namespace std;

cv::Mat im;
void interact(int event, int x, int y, int flags, void *param) {

  if(event == CV_EVENT_LBUTTONDOWN) {

		// Get the corner
		printf("The corner this click corresponds to: ");
		int key = cv::waitKey(0);
		int corner = key - 48;
		printf("%d (%d, %d)\n", corner, x, y);

		// Set the corner coordinates
		cv::Vec2s* coords = (cv::Vec2s*) param;
		coords[corner-1] = cv::Vec2s(x,y); 

		// Keep showing the image
		cv::imshow("image", im);
		cv::waitKey(0);
	}
}

int main (int argc, char* argv[]) {

	// Get the input argument for the data folder
//  if(argc != 3) throw Exception("Usage: ./can01_labelPoints <data folder> <num images>\n");
	const int numImages = atoi(argv[2]);

	// Initialize the point coordinates
	cv::Vec2s coords [numImages][8];
 
	// Show each image
	char path [1024];
	for(int img_idx = 0; img_idx < numImages; img_idx++) {
		
		// Create the file path and read it
		sprintf(path, "%s%d.JPG", argv[1], img_idx + 1);
 	 	im = cv::imread(path, CV_LOAD_IMAGE_COLOR);

		// Set the callback function to get the points
		cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("image", interact, (void*) (&coords[img_idx]));
		
		// Show the image
		cv::imshow("image", im);
		int key = cv::waitKey(0);
		printf("key out: %d\n", key);

		// Print the received coordinates
		for(size_t i = 0; i < 8; i++)
			printf("(%d, %d)\t", coords[img_idx][i][0], coords[img_idx][i][1]);
		printf("\n");
		printf("================================================\n");
		printf("next image!\n");
	}

	// Write the correspondences to a file
	FILE* file = fopen("correspondences.txt", "w");
	for(size_t i = 0; i < 8; i++) {
		for(int img_idx = 0; img_idx < numImages; img_idx++)
			fprintf(file, "%d\t%d\t", coords[img_idx][i][0], coords[img_idx][i][1]);
		fprintf(file, "\n");
	}

}
