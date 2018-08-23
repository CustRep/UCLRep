#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <string>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <std_msgs/String.h>

using namespace std;
using namespace cv;
/*
int H_MIN = 31; 
int H_MAX = 50;
int S_MIN = 66;
int S_MAX = 122;
int V_MIN = 123;
int V_MAX = 225;
*/
// In light Calibration

int H_MIN = 30; 
int H_MAX = 64;
int S_MIN = 35; // 61
int S_MAX = 255; // 137
int V_MIN = 60; // 199
int V_MAX = 222;

// In House
/*
int H_MIN = 24; 
int H_MAX = 76;
int S_MIN = 73;
int S_MAX = 255;
int V_MIN = 97;
int V_MAX = 222;
*/
/*
int H_MIN = 0; 
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;
*/
static const string OPENCV_WINDOW = "Image window";
static const int FRAME_WIDTH = 640;
static const int FRAME_HEIGHT = 480;
static const int FORWARD_TUNE = 80;
//max number of objects to be detected in frame
static const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
static const int MIN_OBJECT_AREA = 10*10; // 40*40
static const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
static const string windowName = "Original Image";
static const string windowName1 = "HSV Image";
static const string windowName2 = "Thresholded Image";
static const string windowName3 = "After Morphological Operations";
static const string trackbarWindowName = "Trackbars";


  
void on_trackbar( int, void*)
{//This function gets called whenever a
  // trackbar position is changed
}



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher msg_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_sub_ = it_.subscribe("/zed/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    msg_pub_ = nh_.advertise<std_msgs::String>("final_vel",1000);
    // ros::Subscriber sub = nh_.subscribe("start",1000, startCallback);


    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  string intToString(int number)
  {
    std::stringstream ss;
    ss << number;
    return ss.str();
  }

  void startCallback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }
  /*
  void on_trackbar( int, void*)
  {//This function gets called whenever a
    // trackbar position is changed
  }
  */
  void createTrackbars()
  {
      // Trackbars are to be added
    namedWindow(trackbarWindowName,0);
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
  }

  void drawObject(int x,int y,Mat &frame)
  {
    cv::circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
    cv::putText(frame,intToString(x)+ " , " + intToString(y),cv::Point(x,y+20),1,1,Scalar(0,255,0));
    std::cout << "Object Found at " << "X: " << intToString(x) << " Y: " << intToString(y) << std::endl;
    
    std_msgs::String ros_msg;
    std::stringstream ss;
    
    if(x < FRAME_WIDTH/2 + FORWARD_TUNE && x > FRAME_WIDTH/2 - FORWARD_TUNE)
    {
      std::cout << "Going Forward" << std::endl;
      //ss << "fw";
    }
    else if(x > FRAME_WIDTH/2 + 30)
    {
      std::cout << "Turning Left" << std::endl;
      //ss << "left";
    }
    else
    {
      std::cout << "Turning Right" << std::endl;
      //ss << "right";
    }
    float angle_f = (x-320)/10.66;
    int angle_i = static_cast<int>(angle_f);
    ss << angle_i;

    ros_msg.data = ss.str();
    ROS_INFO("%s", ros_msg.data.c_str());
    msg_pub_.publish(ros_msg);

  
  }

  void morphOps(Mat &thresh)
  {
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5)); // 3,3
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8)); // 8,8

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);


    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
  }
  void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed)
  {
    int x,y;

    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
      int numObjects = hierarchy.size();
      //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
      if(numObjects<MAX_NUM_OBJECTS){
        for (int index = 0; index >= 0; index = hierarchy[index][0]) {

          Moments moment = moments((cv::Mat)contours[index]);
          double area = moment.m00;

          //if the area is less than 20 px by 20px then it is probably just noise
          //if the area is the same as the 3/2 of the image size, probably just a bad filter
          //we only want the object with the largest area so we safe a reference area each
          //iteration and compare it to the area in the next iteration.
          if(area>MIN_OBJECT_AREA){
            x = moment.m10/area;
            y = moment.m01/area;

          

            objectFound = true;

          }else objectFound = false;


        }
        //let user know you found an object
        if(objectFound ==true){
          //draw object location on screen
          drawObject(x,y,cameraFeed);}

      }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Variables


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

    bool calibrationMode = false;

    if(calibrationMode){
      //create slider bars for HSV filtering
      createTrackbars();
    }

    Mat cameraFeed;
    Mat threshold;
    Mat HSV;
    cameraFeed = cv_ptr->image;
    flip(cameraFeed, cameraFeed, 1);
    cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
    inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
    morphOps(threshold);
    // cv::imshow(windowName2,threshold);
    trackFilteredObject(threshold,HSV,cameraFeed);
    // Update GUI Window
    // cv::imshow(windowName, cameraFeed);
    cv::waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_proccessing");
  ImageConverter ic;
  ros::spin();
  return 0;
}
