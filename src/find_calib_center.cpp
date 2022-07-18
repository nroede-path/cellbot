// Finds the asymmetrical circular calibration board centers and draws lines on the board via video feed

// Include the ROS library
#include <ros/ros.h>

// Include opencv2
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <vector>
#include <iostream>

using namespace cv;

static const cv::String OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
            : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/pylon_camera_node/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow("Image window", 0);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Size patternsize(6, 15); //number of centers
        std::vector<Point2f> centers; //this will be filled by the detected centers
        bool patternfound = findCirclesGrid(cv_ptr->image, patternsize, centers, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
        drawChessboardCorners(cv_ptr->image, patternsize, Mat(centers), patternfound);

        // Generate calib board center coord vector
        std::vector<Point3f> board_points_;

        int n_rows = patternsize.height;
        int n_cols = patternsize.width;
        double spacing = 22/sqrt(2); // using diag spacing in mm
        board_points_.resize(n_cols*n_rows);
        for (int n = 0; n < board_points_.size(); ++n) {
            int row_n = n / n_cols;
            int col_n = n % n_cols;

            board_points_[n].x = (float) ((2 * col_n + row_n % 2) *spacing);
            board_points_[n].y = (float) (row_n * spacing);
            board_points_[n].z = 0.0;
        } // (int n = 0; n < n_rows*n_cols; ++n)
        ROS_INFO_STREAM(board_points_);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(2); // Delay between frames

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}