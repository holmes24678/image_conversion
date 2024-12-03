#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <services/srv/process_image.hpp>
#include <functional>

class ImageServiceNode : public rclcpp::Node
{
    public:
        ImageServiceNode() : Node("image_service_node")
        {
            //subscribing to /image_raw topic publishing 
            image_subscription_ = this -> create_subscription<sensor_msgs::msg::Image>(
                "/image_raw",10, std::bind(&ImageServiceNode::image_callback,this,std::placeholders::_1));
            
            //Service to
            image_service_ = this -> create_service<services::srv::ProcessImage>(
                "/process_image",
                std::bind(&ImageServiceNode::process_image_service,this,std::placeholders::_1,std::placeholders::_2));
            RCLCPP_INFO(this -> get_logger(),"Image Service Node has Started");

            // publish the processed image on /processed_image

            processed_image_publisher_ = this -> create_publisher<sensor_msgs::msg::Image>("/processed_image",10);
        }

    private:

        // image_call_back to get the image and save it in latest_image_
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            try
            {
                latest_image_ = msg;
                RCLCPP_INFO(this->get_logger(),"Recieved Image.");
            }
            catch(const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),"Failed to store the image: %s", e.what());
            }
        }

         // this callback function will used to process the image "latest_image_"
        void process_image_service(const std::shared_ptr<services::srv::ProcessImage::Request > request,
                                    std::shared_ptr<services::srv::ProcessImage::Response> response)

        {
            // Checking for image
            if(!latest_image_)
            {
                RCLCPP_WARN(this->get_logger(),"No image recieved yet");
                response -> success = false;
                response -> message = "No image available";
                return;
            }

            try
            {
                // converting ros image to cvimage using cv_bridge
                cv::Mat cv_image = cv_bridge::toCvCopy(latest_image_,"bgr8") -> image;

                cv::Mat processed_image;
                // checking for service request . if its grayscale convert it to grayscale
                if(request->mode == "grayscale")
                {
                    cv::cvtColor(cv_image,processed_image,cv::COLOR_BGR2GRAY);
                    RCLCPP_INFO(this->get_logger(),"Converted to grayscale.");
                }
                // else default is RGB
                else
                {
                    processed_image = cv_image.clone();
                    RCLCPP_INFO(this->get_logger(),"processed the image in color mode.");
                }
                //Convert the processed image back to ROS2 Image

                auto output_msg = cv_bridge::CvImage(
                                        latest_image_ -> header,
                                        (request->mode == "grayscale")?"mono8" : "bgr8",
                                        processed_image).toImageMsg();

                // Send the processed image as a part of the response
                response -> processed_image = *output_msg;
                response -> success = true;
                response -> message = "Image processed successfully.";
                // Publish the processed image
                processed_image_publisher_->publish(*output_msg);

                //display processed image

                if (!processed_image.empty())
                {
                    cv::imshow("Processed Image", processed_image);
                    cv::waitKey(1);
                }
                    else
                {
                    RCLCPP_WARN(this->get_logger(), "Processed image is empty.");
                 }

            }

            catch(const cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this -> get_logger(),"cv_bridge exception: %s", e.what());
                response -> success = false;
                response -> message = "Image process failed";
            }
            
        }




        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
        sensor_msgs::msg::Image::SharedPtr latest_image_;
        rclcpp::Service<services::srv::ProcessImage>::SharedPtr image_service_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_publisher_;


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ImageServiceNode>());
    rclcpp::shutdown();
    return 0;
}