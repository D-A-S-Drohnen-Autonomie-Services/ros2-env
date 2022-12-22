// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMAGE_PIPELINE__EDGE_DETECTOR_NODE_HPP_
#define IMAGE_PIPELINE__EDGE_DETECTOR_NODE_HPP_

#include <sstream>
#include <string>
#include <memory>

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_pipeline/common.hpp"
using namespace cv;
using namespace std;
cv::Mat src_gray;
int thresh = 100;
cv::RNG rng(12345);
void thresh_callback(int, void* );

/// Node which receives sensor_msgs/Image messages and renders them using OpenCV.
class EdgeDetector final : public rclcpp::Node
{
public:
    /// \brief Construct a new EdgeDetector for visualizing image data
    /// \param input The topic name to subscribe to
    /// \param node_name The node name to use
    /// \param watermark Whether to add a watermark to the image before displaying
    explicit EdgeDetector(
            const std::string & input, const std::string & node_name = "edge_detector",
            bool watermark = true)
            : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // Create a subscription on the input topic.
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                input,
                rclcpp::SensorDataQoS(),
                [node_name, watermark](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                    // Create a cv::Mat from the image message (without copying).
                    cv::Mat cv_mat(
                            msg->height, msg->width,
                            encoding2mat_type(msg->encoding),
                            const_cast<unsigned char *>(msg->data.data()));
                    if (watermark) {
                        // Annotate with the pid and pointer address.
                        std::stringstream ss;
                        ss << "pid: " << GETPID() << ", ptr: " << msg.get();
                        draw_on_image(cv_mat, ss.str(), 60);
                    }
                    // Show the image.
                    cv::Mat c_mat = cv_mat;
                    cv::imshow(node_name.c_str(), c_mat);

                    cvtColor( c_mat, src_gray, COLOR_BGR2GRAY );
                    blur( src_gray, src_gray, Size(3,3) );
                    const int max_thresh = 255;
                    cv::createTrackbar( "Canny thresh:", node_name.c_str(), &thresh, max_thresh, thresh_callback );
                    thresh_callback( 0, 0 );
                    char key = cv::waitKey(1);    // Look for key presses.
                    if (key == 27 /* ESC */ || key == 'q') {
                        rclcpp::shutdown();
                    }
                    if (key == ' ') {    // If <space> then pause until another <space>.
                        key = '\0';
                        while (key != ' ') {
                            key = cv::waitKey(1);
                            if (key == 27 /* ESC */ || key == 'q') {
                                rclcpp::shutdown();
                            }
                            if (!rclcpp::ok()) {
                                break;
                            }
                        }
                    }
                });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    cv::VideoCapture cap_;
    cv::Mat frame_;
};


void thresh_callback(int, void* )
{
    Mat canny_output;
    Canny( src_gray, canny_output, thresh, thresh*2 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    imshow( "Contours", drawing );
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto edge_detector_node = std::make_shared<EdgeDetector>("watermarked_image");
    rclcpp::spin(edge_detector_node);

    rclcpp::shutdown();
    return 0;
}


#endif  // IMAGE_PIPELINE__EDGE_DETECTOR_NODE_HPP_
