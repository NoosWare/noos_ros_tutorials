#include "includes.ihh"
#include "vision.hpp"

/*
 * Example of vision batch 
 */
int main(int argc, char **argv) {
    // Init ros
    ros::init(argc, argv, "vision_batch");
    ros::NodeHandle n;

    /* 
     * If a camera is connected to the computer is going to capture an
     * image every second (loop of 1 Hz).
     * If the camera is not detected or there is no camera connected
     * an image is going to be read from the repository
     */
	cv::VideoCapture cap(0); // open the default camera
    bool camera = true;
    cv::Mat frame;
    if(!cap.isOpened()) {  // check if we succeeded
        std::cout << "Camera not detected" << std::endl;
        camera = false;
        /* Load an image if video option is not used 
         * NOTE: In this example, only PNG images are used.
         *       If you need to use another format check
         *       `vision.cpp` file and change the function
         *       cv::imencode in the line 6
         */
        frame = cv::imread("data/lenna.png");
	}

    /*
     * Vision is the class in charge of send the image
     * for detecting faces, and in the case that one or 
     * more faces would be found, the image is cropped
     * and it is sent again for studing age and expression
     * of that face.
     */
    vision vision_obj;

    /// Loop
	ros::Rate loop_rate(1);
    while (ros::ok()) {
        if (camera) { 
            /* Video capture */
            cap >> frame; // get a new frame from camera
        }

        vision_obj.send(frame);
        loop_rate.sleep(); 
    }
	return 0;
}
