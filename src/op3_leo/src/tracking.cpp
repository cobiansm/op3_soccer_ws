#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <iostream>

cv::Mat global_img_input;
int camera_fps = 30;
void usbImgCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracking");

    // Comunicacion con nodos de ROS
    ros::NodeHandle node_handler;

    // Comunicacion de imagenes
    image_transport::ImageTransport imgt(node_handler);

    // Subscriber
    image_transport::Subscriber usb_cam_sub = 
        imgt.subscribe("/usb_cam/image_raw", 1, usbImgCallback);

    // Publisher
    image_transport::Publisher track_pub =
        imgt.advertise("/tracking/image_raw", 1);

    ros::Publisher error_pub = node_handler.advertise<geometry_msgs::Point>("/ball_error", 1);

    ros::Rate loopRate(camera_fps);

    while(ros::ok())
    {
        ros::spinOnce();
        if(global_img_input.data)
        {
            cv::Mat img_output;
            cv::Mat imgHSV;
            cvtColor(global_img_input, imgHSV, cv::COLOR_BGR2HSV);
            inRange(imgHSV, cv::Scalar(20, 100, 20), cv::Scalar(40, 255, 200), img_output);
            // Se ponen los valores del rango en lugar de las variables Low y High para un color particular

            erode(img_output, img_output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
            dilate(img_output, img_output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
            dilate(img_output, img_output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
            erode(img_output, img_output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

            cv::Moments omoments = moments(img_output);

            double dm01 = omoments.m01;
            double dm10 = omoments.m10;
            double area = omoments.m00;

            if(area > 100)
            {
                int posX = dm10/area;
                int posY = dm01/area;

                int errorX = (img_output.cols/2) - posX;
                int errorY = (img_output.rows/2) - posY;
                
                // Conversion de 0º a 70º
                errorX = (errorX*70)/308;
                errorY = (errorY*70)/220;

                // Saturacion del error
                if (errorX >= 70){
                    errorX = 70;
                }
                else if (errorX <= -70){
                    errorX = -70;
                } 
                
                if (errorY >= 70){
                    errorY = 70;
                }
                else if (errorY <= -70){
                    errorY = -70;
                } 

                std::cout << "ErrorX: " << errorX << "\n";
                std::cout << "ErrorY: " << errorY << "\n";

                geometry_msgs::Point msg;
                msg.x = errorX;
                msg.y = errorY;
                error_pub.publish(msg);

                if(posX >= 0 && posY >= 0)
                {
                    circle(global_img_input, cv::Point(posX, posY), 10, cv::Scalar(0, 255, 0), 
                    cv::FILLED, cv::LINE_8);
                    img_output = global_img_input;
                }
            }

            //    Colores en HSV
            // Naranja entre 0 - 22
            // Amarillo entre 22 - 38
            // Verde entre 38 - 75
            // Azul entre 75 - 130
            // Morado entre 130 - 160
            // Rojo entre 160 - 179   

            sensor_msgs::ImagePtr track_msg;
            track_msg = cv_bridge::CvImage(std_msgs::Header(),
                sensor_msgs::image_encodings::BGR8,
                img_output).toImageMsg();

            track_msg->header.frame_id = "usb_cam_track";
            track_msg->width = img_output.cols;
            track_msg->height = img_output.rows;
            track_msg->is_bigendian = false; // Leer mensaje de derecha a izquierda
            track_msg->step = sizeof(unsigned char)*img_output.cols*3;
            track_msg->header.stamp = ros::Time::now();
            track_pub.publish(track_msg);

        }
        loopRate.sleep();
    }

    return 0;
} 

void usbImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        global_img_input = cv_bridge::toCvShare(msg, "bgr8")->image;
    }catch(cv_bridge::Exception& e)
    {
        std::cerr << e.what() << "/n";
    }
}