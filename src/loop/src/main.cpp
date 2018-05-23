#include "includes.ihh"

using namespace noos::cloud;

/*
 * Do the conversion from cv::Mat to noos::object::picture
 * is not trivial. With this struct the conversion is done
 * for .png images. If another extension is needed 
 */
struct mat_to_picture
{
    noos::object::picture operator()(cv::Mat img)
    {
        std::vector<unsigned char> buf;
        cv::imencode(".png", img, buf);
        std::vector<noos::types::byte> conversion(buf.begin(), buf.end());
        return noos::object::picture(conversion);
    }
};
/*
 * Example of a simple loop
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "loop");
    ros::NodeHandle n;
   /*
     * Construct a lambda, std::function or bind your own functor.
     * In this example we'll pass an inline lambda as the callback.
     * All it does is receive a vector of noos::object::face and
     * we show the size of the vector to know how many faces have 
     * been found.
     */
    auto callback = [&](std::vector<noos::object::face> faces) { 
        std::cout << "Found " << faces.size() << " faces!" << std::endl;
    };

	/*
     * We need to create a platform object with our user and password for using 
     * the NOOS Cloud 
     * IMPORTANT: You have to change your user and password. The example doesn't work
     */
    platform node = {"demo.noos.cloud", "9001", "your_pass", "your_user"};

    /*
     * The callable object is created. There are different ways to create
     * for creating this object. In this case, because the object is going
	 * to be changing, the object is constructed only with the callback.
     * It is not necessary to create the object now, but it is COMPULSORY
     * to created it before call the method `send` 
     */
    callable<face_detection,true> callable_object(callback, 
                                                  node);

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
         *       `mat_to_picture` struct and change the function
         *       cv::imencode in the line 15
         */
        frame = cv::imread("data/lenna.png");
    }

    /// Loop
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        if (camera) { 
            /* Video capture */
            cap >> frame; // get a new frame from camera
        }
        /* 
         * The object of the callable_obj is going to be modified 
         * in every loop. Thanks to this we can update the image 
         * which is sent to the platform without create a different
         * callable object
         */
        callable_object.object = face_detection(mat_to_picture()(frame)); 
        callable_object.send();

        loop_rate.sleep(); 
    }
	return 0;
}
