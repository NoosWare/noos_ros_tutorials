#ifndef VISION_HPP
#define VISION_HPP

#include "includes.ihh"


using namespace noos::cloud;

/**
 * @brief conversion from cv::Mat to noos::object::picture
 * @struct mat_to_picture
 * @date 30.10.2017
 * @version 0.1.0
 */
struct mat_to_picture
{
    noos::object::picture operator()(cv::Mat img);
};

/**
 * @brief An image is send to detect faces, and if there any face 
 *        a vision batch will be send
 * @class vision
 * @date 30.10.2017
 * @version 0.1.0
 */
class vision
{
public:
	/// @brief constructor
    vision();

	/// @brief callback which return the number of faces found
    void callback(std::vector<noos::object::face> faces);

	/// @send the face_detection query
    void send(cv::Mat pic);

private:
    using vbatch = vision_batch<tied<face_expression>,tied<age_detection>>;

	//face expression callback
	void face_expression_cb(std::vector<std::pair<std::string,float>> data);

    //age detection callback
	void age_detection_cb(std::vector<std::pair<std::string,float>> data);

    //send vision batch object
    void batch_send(noos::object::picture new_pic);

    //crop an image into an area of interest
    noos::object::picture roi_image(const noos::object::face face);

	// callable for face_detection
    callable<face_detection, true> query__;

    // callable for vision_batch
    std::unique_ptr<callable<vbatch, true>> batch__;
	
    // save the picture 
    cv::Mat pic__;

    noos::cloud::tied<face_expression> exp_tie__;
    noos::cloud::tied<age_detection>   age_tie__;
};


#endif
