// Finds the 4x4 transformation matrix from the camera to the calibration board, then draws axes at the board origin and circles at each center point, and publishes a boolean of whether the board is found or not
// DO NOT include CALIB_CB_CLUSTERING in the findCirclesGrid, as it significantly decreases consistency

// Include the ROS library
#include <ros/ros.h>
#include <std_msgs/Bool.h>

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
    ros::Publisher pubfound;

public:
    ImageConverter()
            : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/pylon_camera_node/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        pubfound = nh_.advertise<std_msgs::Bool>("cam1/board_found", 1000);

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

        Size patternsize(5, 11); //number of centers
        std::vector<Point2f> centers; //this will be filled by the detected centers (imagePoints)
        std_msgs::Bool found_status;
        found_status.data = false;
        bool patternfound = findCirclesGrid(cv_ptr->image, patternsize, centers, CALIB_CB_ASYMMETRIC_GRID);

        if (patternfound) {
            found_status.data = true;
            drawChessboardCorners(cv_ptr->image, patternsize, Mat(centers), patternfound);

            // Generate calib board center coord vector (objectPoints)
            std::vector<Point3f> board_points_;

            int n_rows = patternsize.height;
            int n_cols = patternsize.width;
            double spacing = 12/sqrt(2); // using diag spacing in mm
            board_points_.resize(n_cols*n_rows);
            for (int n = 0; n < board_points_.size(); ++n) {
                int row_n = n / n_cols;
                int col_n = n % n_cols;

                board_points_[n].x = (float) ((2 * col_n + row_n % 2) *spacing);
                board_points_[n].y = (float) (row_n * spacing);
                board_points_[n].z = 0.0;
            } // (int n = 0; n < n_rows*n_cols; ++n)
            //ROS_INFO_STREAM(board_points_);

            // Camera intrinsics (cameraMatrix and distCoeffs)

            cv::Mat mtx = (cv::Mat_<double>(3, 3)
                    << 2424.687503985171, 0, 1201.538142079326,
                    0, 2428.130085113846, 955.6942513475894,
                    0, 0, 1);
            cv::Mat dist = (cv::Mat_<double>(5, 1) << -0.2460440432365548, 0.7593260195770797, 0.001010992458501329, 0.001776219125150247, -1.2574083896541);

            cv::Mat rotation_vector(1, 3, CV_32FC1); // (rvec)
            cv::Mat translation_vector(1, 3, CV_32FC1); // (tvec)

            bool tffound = cv::solvePnP(board_points_, centers, mtx, dist, rotation_vector, translation_vector);

            if (tffound) {
                cv::Mat rotation_matrix(3, 3, CV_32FC1);
                cv::Rodrigues(rotation_vector, rotation_matrix);
                cv::Mat camera_to_board = cv::Mat::eye(4, 4, CV_32FC1);
                cv::Rect rotation_loc = cv::Rect(0, 0, 3, 3);
                cv::Rect translation_loc = cv::Rect(3, 0, 1, 3);
                rotation_matrix.copyTo(camera_to_board(rotation_loc));
                translation_vector.copyTo(camera_to_board(translation_loc));
                camera_to_board.at<float>(3, 3) = 1;

                ROS_INFO_STREAM(camera_to_board);

                // Project board points into camera frame
//                std::vector<cv::Point3f> camera_points_in_its_frames, camera_board_points_in_current_camera_frame;
//
//                camera_board_points_in_current_camera_frame.clear();
//                camera_board_points_in_current_camera_frame.resize(board_points_.size());
//                #pragma omp parallel for
//                for (int i = 0; i < board_points_.size(); i ++ ) {
//                    cv::Mat original_point_mat(cv::Size(1,4), CV_32FC1);
//                    cv::Mat transformed_point_mat(cv::Size(1,4), CV_32FC1);
//                    cv::Point3f point_in_new_frame;
//
//                    original_point_mat.at<float>(0,0) = board_points_[i].x;
//                    original_point_mat.at<float>(1,0) = board_points_[i].y;
//                    original_point_mat.at<float>(2,0) = board_points_[i].z;
//                    original_point_mat.at<float>(3,0) = 1;
//
//                    cv::gemm(camera_to_board, original_point_mat, 1, cv::Mat(), 0, transformed_point_mat);
//
//                    point_in_new_frame.x = transformed_point_mat.at<float>(0,0);
//                    point_in_new_frame.y = transformed_point_mat.at<float>(1,0);
//                    point_in_new_frame.z = transformed_point_mat.at<float>(2,0);
//                    camera_board_points_in_current_camera_frame[i] = point_in_new_frame;
//                }
//
//                for (int i = 0; i < camera_board_points_in_current_camera_frame.size(); i++) {
//                    camera_points_in_its_frames.emplace_back(camera_board_points_in_current_camera_frame[i]);
//                }

                std::vector<Point2f> projcenters;
                cv::projectPoints(board_points_, rotation_vector, translation_vector, mtx, dist, projcenters);

                for (int j = 0; j < projcenters.size(); j++) {
                    cv::circle(cv_ptr->image, projcenters[j], 4, Scalar(255, 255, 255));
                }

                std::vector<Point3f> axes = {{0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {0, 0, -10}};
                std::vector<Point2f> imgpts;

                cv::projectPoints(axes, rotation_vector, translation_vector, mtx, dist, imgpts);

                cv::line(cv_ptr->image, imgpts[0], imgpts[1], Scalar(0, 0, 255), 2, LINE_4);
                cv::line(cv_ptr->image, imgpts[0], imgpts[2], Scalar(0, 255, 0), 2, LINE_4);
                cv::line(cv_ptr->image, imgpts[0], imgpts[3], Scalar(255, 0, 0), 2, LINE_4);
            }
        }

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(2); // Delay between frames

        // Output whether the calibration board was found
        pubfound.publish(found_status);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_find");
    ImageConverter ic;
    ros::spin();
    return 0;
}