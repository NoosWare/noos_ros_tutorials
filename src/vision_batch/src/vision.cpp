#include "vision.hpp"

noos::object::picture mat_to_picture::operator()(cv::Mat img)
{
    std::vector<unsigned char> buf;
    cv::imencode(".png", img, buf);
    std::vector<noos::types::byte> conversion(buf.begin(), buf.end());
    return noos::object::picture(conversion);
}

/*
 * In the constructor we create a callable object for face_detection
 * which is going to use the method `callback` of this class to 
 * check how many faces have been found and crop the image to
 * send a vision batch for studing age and expression of these images
 */
vision::vision()
: query__(std::bind(&vision::callback, this, std::placeholders::_1),
          default_node),
  exp_tie__([&](const auto data){ this->face_expression_cb(data);}),
  age_tie__([&](const auto data) { this->age_detection_cb(data);})
{}

void vision::callback(std::vector<noos::object::face> faces)
{
    if (faces.size() != 0) {
        for (auto const face : faces) {
            std::cout << "face_callback" << std::endl;
            auto new_image = roi_image(face);
            batch_send(new_image); 
        }            
    } 
    else {
        std::cout << "Faces not found" << std::endl;
    }
}

void vision::send(cv::Mat pic)
{
    pic__ = pic;
    /*
     * The object of the callable is defined here, so in each loop
     * a different face_detection object will be assign with a 
     * different image.
     */
	query__.object = face_detection(mat_to_picture()(pic));
    query__.send();
}

void vision::face_expression_cb(std::vector<std::pair<std::string,float>> data) 
{
    if (data.size() != 0) {
        for (const auto expression : data) {
            std::cout << "Expression: " << expression.first 
            << "  Probability: " << expression.second << std::endl;	
        }
    }
    else {
        std::cout << "No expression detected" << std::endl;
    }
}

void vision::age_detection_cb(std::vector<std::pair<std::string,float>> data) 
{
    if (data.size() != 0) {
        for (const auto age : data) {
            std::cout << "Age : " << age.first
            << "  Probability: " << age.second << std::endl;	
        }
    }
    else {
        std::cout << "No age calculated" << std::endl;
    }
}

noos::object::picture vision::roi_image(const noos::object::face face)
{
    std::cout << "roi image" << std::endl;
    //Crop the image to send only the part of the image where the face has been detected
    cv::Rect roi(cv::Point(face.top_left_x, face.top_left_y),
                 cv::Point(face.bottom_right_x, face.bottom_right_y));
    auto cropped_img = pic__(roi);
    return mat_to_picture()(cropped_img);
}

void vision::batch_send(noos::object::picture new_pic)
{
    /*
     * A different callable object is created to send the vision batch.
     * The advance is that only one image is sent to use different services
     * at the same time.

    callable<vbatch, false> batch_query(new_pic,
                                       default_node,
                                       tied<face_expression>(std::bind(&vision::face_expression_cb, this, std::placeholders::_1)),
                                       tied<age_detection>(std::bind(&vision::age_detection_cb, this, std::placeholders::_1)));
     */
   
    if (!batch__) {
        batch__ = std::make_unique<callable<vbatch,true>>(new_pic, 
                                                          default_node, 
                                                          exp_tie__,
                                                          age_tie__); 
    } 
    else {
        batch__->object = vbatch(new_pic, exp_tie__, age_tie__);
    }
    assert(batch__);
    batch__->send();
}
