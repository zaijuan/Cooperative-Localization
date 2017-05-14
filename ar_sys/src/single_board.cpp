/**
 * @file single_board.cpp
 * @author Hamdi Sahloul
 * @date September 2014
 * @version 0.1
 * @brief ROS version of the example named "simple_board" in the Aruco software package.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_sys/utils.h>
#include <ar_sys/Trans_Repro_Info.h>
#include <ar_sys/arsys_measurement.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace aruco;
geometry_msgs::TransformStamped transform_vec[3];

class ArSysSingleBoard
{
private:
    cv::Mat inImage, resultImg;
    aruco::CameraParameters camParam;
    bool useRectifiedImages;
    bool draw_markers;
    bool draw_markers_cube;
    bool draw_markers_axis;
    bool publish_tf;
    bool START;
    MarkerDetector mDetector;
    vector<Marker> markers;
    BoardConfiguration the_board_config;
    BoardDetector the_board_detector;
    Board the_board_detected;
    ros::Subscriber cam_info_sub;
    bool cam_info_received;
    image_transport::Publisher image_pub;
    image_transport::Publisher debug_pub;
    ros::Publisher trans_repro_pub;
    ros::Publisher transform_pub;
    std::string board_frame;
    unsigned int msg_number;
    int msg_id;

    double marker_size;
    double dist_difference;
    double noise_size;
    std::string board_config;

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    tf::TransformListener _tfListener;

public:
    ArSysSingleBoard()
        : cam_info_received(false),
          nh("~"),
          it(nh), msg_number(0), msg_id(0),dist_difference(0.0),START(false)
    {
        image_sub = it.subscribe("/image", 1, &ArSysSingleBoard::image_callback, this);
        cam_info_sub = nh.subscribe("/camera_info", 1, &ArSysSingleBoard::cam_info_callback, this);

        image_pub = it.advertise("result", 1);
        debug_pub = it.advertise("debug", 1);
        trans_repro_pub = nh.advertise<ar_sys::Trans_Repro_Info>("Trans_Repro_Info",100);
        transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);

        nh.param<double>("marker_size", marker_size, 0.151);
        nh.param<std::string>("board_config", board_config, "boardConfiguration.yml");
        nh.param<std::string>("board_frame", board_frame, "/backboardframe");
        nh.param<bool>("image_is_rectified", useRectifiedImages, true);
        nh.param<bool>("draw_markers", draw_markers, false);
        nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
        nh.param<bool>("draw_markers_axis", draw_markers_axis, false);
        nh.param<bool>("publish_tf", publish_tf, false);
        nh.param<double>("noise_size",noise_size, 0.3);

        the_board_config.readFromFile(board_config.c_str());

        ROS_INFO("ArSys node started with marker size of %f m and board configuration: %s",
                 marker_size, board_config.c_str());
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        static tf::TransformBroadcaster br;

        if(!cam_info_received) return;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            inImage = cv_ptr->image;
            resultImg = cv_ptr->image.clone();

            //detection results will go into "markers"
            markers.clear();
            //Ok, let's detect
            mDetector.detect(inImage, markers, camParam, marker_size, false);
            //Detection of the board
            double errSum = 0;
            float probDetect=the_board_detector.detect(errSum, markers, the_board_config, the_board_detected, camParam, noise_size, marker_size);

            if (probDetect > 0.0)
            {
                tf::Transform transform = ar_sys::getTf(the_board_detected.Rvec, the_board_detected.Tvec);

                tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, board_frame);

                if (publish_tf)
                    br.sendTransform(stampedTransform);

                geometry_msgs::TransformStamped transformMsg;
                tf::transformStampedTFToMsg(stampedTransform, transformMsg);
                transform_pub.publish(transformMsg);

                //add by Zaijuan, here design a very shabby low-pass filter to filter out the high-frequence noise.
                if ( !START ){
                    if( (msg_number < 3) && (msg_id < 3 ) ){
                        transform_vec[msg_id] = transformMsg;
                        msg_number++;
                        msg_id++;}

                    if (msg_number == 3){
                        double x_difference = fabs( transformMsg.transform.translation.x - transform_vec[0].transform.translation.x) +
                                fabs( transformMsg.transform.translation.x - transform_vec[1].transform.translation.x) +
                                fabs( transformMsg.transform.translation.x - transform_vec[2].transform.translation.x);

                        double y_difference = fabs( transformMsg.transform.translation.y - transform_vec[0].transform.translation.y) +
                                fabs( transformMsg.transform.translation.y - transform_vec[1].transform.translation.y) +
                                fabs( transformMsg.transform.translation.y - transform_vec[2].transform.translation.y);

                        double z_difference = fabs( transformMsg.transform.translation.z - transform_vec[0].transform.translation.z) +
                                fabs( transformMsg.transform.translation.z - transform_vec[1].transform.translation.z) +
                                fabs( transformMsg.transform.translation.z - transform_vec[2].transform.translation.z);

                        double sum_difference = x_difference + y_difference + z_difference;
                        if (sum_difference > 0.04){
                            msg_number = 0;
                            msg_id = 0;
                        } // reload transform msg from msg_id = 0;
                        if (sum_difference <= 0.04){
                            START = true; //store these three stable transform msg.
                            msg_number = 0;
                            msg_id = 0;
                        }
                    }
                }//End of !START.

                if ( START ){
                    double x_difference = fabs( transformMsg.transform.translation.x - transform_vec[0].transform.translation.x) +
                            fabs( transformMsg.transform.translation.x - transform_vec[1].transform.translation.x) +
                            fabs( transformMsg.transform.translation.x - transform_vec[2].transform.translation.x);

                    double y_difference = fabs( transformMsg.transform.translation.y - transform_vec[0].transform.translation.y) +
                            fabs( transformMsg.transform.translation.y - transform_vec[1].transform.translation.y) +
                            fabs( transformMsg.transform.translation.y - transform_vec[2].transform.translation.y);

                    double z_difference = fabs( transformMsg.transform.translation.z - transform_vec[0].transform.translation.z) +
                            fabs( transformMsg.transform.translation.z - transform_vec[1].transform.translation.z) +
                            fabs( transformMsg.transform.translation.z - transform_vec[2].transform.translation.z);

                    dist_difference = x_difference + y_difference + z_difference ;

                    if (   dist_difference < 0.15  )
                    {
                        msg_id= msg_number%3;
                        transform_vec[msg_id] = transformMsg;
                        msg_number++;

                        ar_sys::Trans_Repro_Info trans_repro_msg;
                        vector< pair <cv::Point3f,cv::Point2f> > measurements_3d_2d = the_board_detector.getMeasurements();
                        vector<double> repro_error_vec = the_board_detector.get_repj_err_vec();

                        ar_sys::arsys_measurement single_measure;
                        double single_repro;

                        trans_repro_msg.transform_.header = transformMsg.header;
                        trans_repro_msg.transform_.child_frame_id=transformMsg.child_frame_id;
                        trans_repro_msg.transform_.transform=transformMsg.transform;
                        trans_repro_msg.error= errSum;

                        for(unsigned int i=0;i<measurements_3d_2d.size();++i)
                        {
                            single_measure.objectpoint="marker_board";
                            single_measure.objx = measurements_3d_2d.at(i).first.x;
                            single_measure.objy = measurements_3d_2d.at(i).first.y;
                            single_measure.objz = measurements_3d_2d.at(i).first.z;
                            single_measure.imagepoint="Image";
                            single_measure.imgx = measurements_3d_2d.at(i).second.x;
                            single_measure.imgy = measurements_3d_2d.at(i).second.y;

                            trans_repro_msg.Measurements.push_back(single_measure);

                            single_repro = repro_error_vec.at(i);
                            trans_repro_msg.errorVec.push_back(single_repro);
                        }
                        trans_repro_pub.publish(trans_repro_msg);

                        if (msg_number == 3) msg_number = 0; //prevent msg_number from getting too large.
                    } //end of the publish message part.
                } // End of START.
            }

            //for each marker, draw info and its boundaries in the image
            for(size_t i=0; draw_markers && i < markers.size(); ++i)
            {
                markers[i].draw(resultImg,cv::Scalar(0,0,255),2);
            }


            if(camParam.isValid() && marker_size != -1)
            {
                //draw a 3d cube in each marker if there is 3d info
                for(size_t i=0; i<markers.size(); ++i)
                {
                    if (draw_markers_cube) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
                    if (draw_markers_axis) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
                }
                //draw board axis
                if (probDetect > 0.0) CvDrawingUtils::draw3dAxis(resultImg, the_board_detected, camParam);
            }

            if(image_pub.getNumSubscribers() > 0)
            {
                //show input with augmented information
                cv_bridge::CvImage out_msg;
                out_msg.header.frame_id = msg->header.frame_id;
                out_msg.header.stamp = msg->header.stamp;
                out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                out_msg.image = resultImg;
                image_pub.publish(out_msg.toImageMsg());
            }

            if(debug_pub.getNumSubscribers() > 0)
            {
                //show also the internal image resulting from the threshold operation
                cv_bridge::CvImage debug_msg;
                debug_msg.header.frame_id = msg->header.frame_id;
                debug_msg.header.stamp = msg->header.stamp;
                debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
                debug_msg.image = mDetector.getThresholdedImage();
                debug_pub.publish(debug_msg.toImageMsg());
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    // wait for one camerainfo, then shut down that subscriber
    void cam_info_callback(const sensor_msgs::CameraInfo &msg)
    {
        camParam = ar_sys::getCamParams(msg, useRectifiedImages);
        cam_info_received = true;
        cam_info_sub.shutdown();
    }
};


int main(int argc,char **argv)
{
    ros::init(argc, argv, "ar_single_board");

    ArSysSingleBoard node;

    ros::spin();
}
