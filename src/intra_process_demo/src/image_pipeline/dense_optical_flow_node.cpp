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

class DenseFlow final : public rclcpp::Node {
public:
    Mat old_frame, old_gray, mask;
    vector<Point2f> p0;

    /// \brief Construct a new DenseFlow for visualizing image data
    explicit DenseFlow()
            : Node("dense_optical_flow", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        // Create a subscription on the input topic.
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "watermarked_image",
                rclcpp::SensorDataQoS(),
                std::bind(&DenseFlow::opticalFlowCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    void opticalFlowCallback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
// Create a cv::Mat from the image message (without copying).
        cv::Mat cv_mat(
                msg->height, msg->width,
                encoding2mat_type(msg->encoding),
                const_cast<unsigned char *>(msg->data.data()));
        // Create some random colors
        vector<Scalar> colors;
        RNG rng;
        for (int i = 0; i < 100; i++) {
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.push_back(Scalar(r, g, b));
        }

        if (this->old_frame.empty()) {
            this->old_frame = cv_mat;
            cvtColor(this->old_frame, this->old_gray, COLOR_BGR2GRAY);
            goodFeaturesToTrack(this->old_gray, this->p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
            this->mask = Mat::zeros(this->old_frame.size(), this->old_frame.type());
        }

        vector<Point2f> p1;
        Mat frame, frame_gray;
        frame = cv_mat;
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(this->old_gray, frame_gray, this->p0, p1, status, err, Size(15, 15), 2, criteria);
        vector<Point2f> good_new;
        for (uint i = 0; i < this->p0.size(); i++) {
            // Select good points
            if (status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(mask, p1[i], this->p0[i], colors[i], 2);
                circle(frame, p1[i], 5, colors[i], -1);
            }
        }
        Mat img;
        add(frame, mask, img);
        imshow("Frame", img);
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
        // Now update the previous frame and previous points
        this->old_gray = frame_gray.clone();
        p0 = good_new;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto edge_detector_node = std::make_shared<DenseFlow>();
    rclcpp::spin(edge_detector_node);
    rclcpp::shutdown();
    return 0;
}

#endif  // IMAGE_PIPELINE__DENSE_OPTICAL_FLOW_NODE_HPP_