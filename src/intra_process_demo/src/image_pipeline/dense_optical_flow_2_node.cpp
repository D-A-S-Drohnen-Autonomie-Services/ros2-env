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

#ifndef IMAGE_PIPELINE__DENSE_OPTICAL_FLOW_NODE_HPP_
#define IMAGE_PIPELINE__DENSE_OPTICAL_FLOW_NODE_HPP_

#include <sstream>
#include <iostream>
#include <string>
#include <functional>
#include <memory>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/core/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_pipeline/common.hpp"

using namespace cv;
using namespace std;

class DenseFlow2 final : public rclcpp::Node {
public:
    Mat first_frame, prvs;
    vector<Point2f> p0;

    /// \brief Construct a new DenseFlow2 for visualizing image data
    explicit DenseFlow2()
            : Node("dense_optical_flow", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        // Create a subscription on the input topic.
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "watermarked_image",
                rclcpp::SensorDataQoS(),
                std::bind(&DenseFlow2::opticalFlowCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    void opticalFlowCallback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
// Create a cv::Mat from the image message (without copying).
        cv::Mat frame2(
                msg->height, msg->width,
                encoding2mat_type(msg->encoding),
                const_cast<unsigned char *>(msg->data.data()));
        Mat next;
        if (this->first_frame.empty()) {
            this->first_frame = frame2;
        }
        if (this->prvs.empty()) {
            cvtColor(this->first_frame, this->prvs, COLOR_BGR2GRAY);
        }

        cvtColor(frame2, next, COLOR_BGR2GRAY);
        Mat flow(prvs.size(), CV_32FC2);
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        // visualization
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
        angle *= ((1.f / 360.f) * (180.f / 255.f));
        //build hsv image
        Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cvtColor(hsv8, bgr, COLOR_HSV2BGR);
        imshow("frame2", bgr);
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
        prvs = next;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto edge_detector_node = std::make_shared<DenseFlow2>();
    rclcpp::spin(edge_detector_node);
    rclcpp::shutdown();
    return 0;
}

#endif  // IMAGE_PIPELINE__DENSE_OPTICAL_FLOW_NODE_HPP_