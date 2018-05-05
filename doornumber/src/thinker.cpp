#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
/*
  cv_bridge中包含CvBridge库
*/
#include<cv_bridge/cv_bridge.h>
/*
  ROS图象类型的编码函数
*/
#include<sensor_msgs/image_encodings.h>
/*
   image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息
*/
#include<image_transport/image_transport.h>

//OpenCV2标准头文件
#include  "opencv2/opencv.hpp"
#include "opencv2/text.hpp"

#include  <vector>
#include  <iostream>
#include  <iomanip>

using namespace std;
using namespace cv;
using namespace cv::text;

void show_help_and_exit(const char *cmd);
void groups_draw(Mat &src, vector<Rect> &groups);
void er_show(vector<Mat> &channels, vector<vector<ERStat> > &regions);

static const std::string INPUT = "Input"; //定义输入窗口名称
static const std::string OUTPUT = "Output"; //定义输出窗口名称

//定义一个转换的类
class RGB_GRAY
{
private:
    ros::NodeHandle nh_; //定义ROS句柄
    image_transport::ImageTransport it_; //定义一个image_transport实例
    image_transport::Subscriber image_sub_; //定义ROS图象接收器
    //image_transport::Publisher image_pub_; //定义ROS图象发布器
public:
    RGB_GRAY()
      :it_(nh_) //构造函数
    {
        image_sub_ = it_.subscribe("cv_camera/image_raw", 1, &RGB_GRAY::convert_callback, this); //定义图象接受器，订阅话题是“camera/rgb/image_raw”
       // image_pub_ = it_.publishe("", 1); //定义图象发布器
        //初始化输入输出窗口
        cv::namedWindow(INPUT);
        cv::namedWindow(OUTPUT);
    }
    ~RGB_GRAY() //析构函数
    {
         cv::destroyWindow(INPUT);
         cv::destroyWindow(OUTPUT);
    }
    /*
      这是一个ROS和OpenCV的格式转换回调函数，将图象格式从sensor_msgs/Image  --->  cv::Mat
    */
    void convert_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例

        try
        {
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
        }
        catch(cv_bridge::Exception& e)  //异常处理
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        image_process(cv_ptr->image); //得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数
    }
    /*
       这是图象处理的主要函数，一般会把图像处理的主要程序写在这个函数中。这里的例子只是一个彩色图象到灰度图象的转化
    */
    void image_process(cv::Mat img)
    {
       //cv::Mat img_out;
       //cv::cvtColor(img, img_out, CV_RGB2GRAY);  //转换成灰度图象
       cv::imshow(INPUT, img);
       //cv::imshow(OUTPUT, img_out);
       //cv::waitKey(5);

       // TEXT OCR part
       Mat src;
       img.copyTo(src);
       //resize(src, src, Size(src.cols/4, src.rows/4));


       // Extract channels to be processed individually
       vector<Mat> channels;
       computeNMChannels(src, channels);

       int cn = (int)channels.size();
       // Append negative channels to detect ER- (bright regions over dark background)
       for (int c = 0; c < cn-1; c++)
           channels.push_back(255-channels[c]);

       // Create ERFilter objects with the 1st and 2nd stage default classifiers
       Ptr<ERFilter> er_filter1 = createERFilterNM1(loadClassifierNM1("/home/liaoziwei/researches/rosspace/src/doornumber/xml/trained_classifiernm1.xml"),16,0.00015f,0.13f,0.2f,true,0.1f);
       Ptr<ERFilter> er_filter2 = createERFilterNM2(loadClassifierNM2("/home/liaoziwei/researches/rosspace/src/doornumber/xml/trained_classifiernm2.xml"),0.5);

       vector<vector<ERStat> > regions(channels.size());
       // Apply the default cascade classifier to each independent channel (could be done in parallel)
       cout << "Extracting Class Specific Extremal Regions from " << (int)channels.size() << " channels ..." << endl;
       cout << "    (...) this may take a while (...)" << endl << endl;
       for (int c=0; c<(int)channels.size(); c++)
       {
           er_filter1->run(channels[c], regions[c]);
           er_filter2->run(channels[c], regions[c]);
           cout << "after " << c << " fors." << endl;
       }

       // Detect character groups
       cout << "Grouping extracted ERs ... ";
       vector< vector<Vec2i> > region_groups;
       vector<Rect> groups_boxes;
       erGrouping(src, channels, regions, region_groups, groups_boxes, ERGROUPING_ORIENTATION_HORIZ);
       //erGrouping(src, channels, regions, region_groups, groups_boxes, ERGROUPING_ORIENTATION_ANY, "./trained_classifier_erGrouping.xml", 0.5);

       // draw groups
       groups_draw(src, groups_boxes);
       imshow(OUTPUT,src);

       //imwrite("./output/result.jpg", src);

//       cout << "Done!" << endl << endl;
//       cout << "Press 'space' to show the extracted Extremal Regions, any other key to exit." << endl << endl;
//       if ((waitKey()&0xff) == ' ')
//           er_show(channels,regions);

       // memory clean-up
       er_filter1.release();
       er_filter2.release();
       regions.clear();
       if (!groups_boxes.empty())
       {
           groups_boxes.clear();
       }

       cv::waitKey(5);
    }
};

//主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "RGB");
    RGB_GRAY obj;
    ros::spin();
}

void show_help_and_exit(const char *cmd)
{
    cout << "    Usage: " << cmd << " <input_image> " << endl;
    cout << "    Default classifier files (trained_classifierNM*.xml) must be in current directory" << endl << endl;
    exit(-1);
}

void groups_draw(Mat &src, vector<Rect> &groups)
{
    for (int i=(int)groups.size()-1; i>=0; i--)
    {
        if (src.type() == CV_8UC3)
            rectangle(src,groups.at(i).tl(),groups.at(i).br(),Scalar( 0, 255, 255 ), 3, 8 );
        else
            rectangle(src,groups.at(i).tl(),groups.at(i).br(),Scalar( 255 ), 3, 8 );
    }
}

void er_show(vector<Mat> &channels, vector<vector<ERStat> > &regions)
{
    for (int c=0; c<(int)channels.size(); c++)
    {
        Mat dst = Mat::zeros(channels[0].rows+2,channels[0].cols+2,CV_8UC1);
        for (int r=0; r<(int)regions[c].size(); r++)
        {
            ERStat er = regions[c][r];
            if (er.parent != NULL) // deprecate the root region
            {
                int newMaskVal = 255;
                int flags = 4 + (newMaskVal << 8) + FLOODFILL_FIXED_RANGE + FLOODFILL_MASK_ONLY;
                floodFill(channels[c],dst,Point(er.pixel%channels[c].cols,er.pixel/channels[c].cols),
                          Scalar(255),0,Scalar(er.level),Scalar(0),flags);
            }
        }
        char buff[10]; char *buff_ptr = buff;
        sprintf(buff, "channel %d", c);
        imshow(buff_ptr, dst);
    }
    waitKey(-1);
}
