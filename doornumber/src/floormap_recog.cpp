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
#include<doornumber/boardArray.h>
#include<doornumber/board.h>
#include<std_msgs/Bool.h>

/*
   image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息
*/
#include<image_transport/image_transport.h>

//OpenCV2标准头文件
#include "opencv2/text.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"

#include <iostream>

/*
 * PARAMETER PART
 */
#define ONE_FRAME_ONE_TIME 1

#define PIC_SAVE_LOCATIONS "//home//liaoziwei//ros_project//images//%04d.jpg"

using namespace std;
using namespace cv;
using namespace cv::text;

static const std::string INPUT = "Input"; //定义输入窗口名称
static const std::string OUTPUT = "Output"; //定义输出窗口名称

// Global Variance
bool downsize = false;
int  REGION_TYPE = 1;
int  GROUPING_ALGORITHM = 0;
int  RECOGNITION = 0;

Mat frame, image, gray, out_img;


String region_types_str[2] = {"ERStats", "MSER"};
String grouping_algorithms_str[2] = {"exhaustive_search", "multioriented"};
String recognitions_str[2] = {"Tesseract", "NM_chain_features + KNN"};

vector<Mat> channels;
vector<vector<ERStat> > regions(2); //two channels

vector< Ptr<ERFilter> > er_filters1;
vector< Ptr<ERFilter> > er_filters2;

int num_ocrs = 10;
vector< Ptr<OCRTesseract> > ocrs;

vector< Ptr<OCRHMMDecoder> > decoders;


// for saving pictures
char filename[1000][10] = {0};
int CaptureNum = 0;

//ERStat extraction is done in parallel for different channels
class Parallel_extractCSER: public cv::ParallelLoopBody
{
private:
    vector<Mat> &channels;
    vector< vector<ERStat> > &regions;
    vector< Ptr<ERFilter> > er_filter1;
    vector< Ptr<ERFilter> > er_filter2;

public:
    Parallel_extractCSER(vector<Mat> &_channels, vector< vector<ERStat> > &_regions,
                         vector<Ptr<ERFilter> >_er_filter1, vector<Ptr<ERFilter> >_er_filter2)
        : channels(_channels),regions(_regions),er_filter1(_er_filter1),er_filter2(_er_filter2) {}

    virtual void operator()( const cv::Range &r ) const
    {
        for (int c=r.start; c < r.end; c++)
        {
            er_filter1[c]->run(channels[c], regions[c]);
            er_filter2[c]->run(channels[c], regions[c]);
        }
    }
    Parallel_extractCSER & operator=(const Parallel_extractCSER &a);
};

//OCR recognition is done in parallel for different detections
template <class T>
class Parallel_OCR: public cv::ParallelLoopBody
{
private:
    vector<Mat> &detections;
    vector<string> &outputs;
    vector< vector<Rect> > &boxes;
    vector< vector<string> > &words;
    vector< vector<float> > &confidences;
    vector< Ptr<T> > &ocrs;

public:
    Parallel_OCR(vector<Mat> &_detections, vector<string> &_outputs, vector< vector<Rect> > &_boxes,
                 vector< vector<string> > &_words, vector< vector<float> > &_confidences,
                 vector< Ptr<T> > &_ocrs)
        : detections(_detections), outputs(_outputs), boxes(_boxes), words(_words),
          confidences(_confidences), ocrs(_ocrs)
    {}

    virtual void operator()( const cv::Range &r ) const
    {
        for (int c=r.start; c < r.end; c++)
        {
            ocrs[c%ocrs.size()]->run(detections[c], outputs[c], &boxes[c], &words[c], &confidences[c], OCR_LEVEL_WORD);
        }
    }
    Parallel_OCR & operator=(const Parallel_OCR &a);
};

//Discard wrongly recognised strings
bool   isRepetitive(const string& s);
//Draw ER's in an image via floodFill
void   er_draw(vector<Mat> &channels, vector<vector<ERStat> > &regions, vector<Vec2i> group, Mat& segmentation);

const char* keys =
{
    "{@input   | 0 | camera index or video file name}"
    "{ image i |   | specify input image}"
};


bool isRepetitive(const string& s)
{
    int count  = 0;
    int count2 = 0;
    int count3 = 0;
    int first=(int)s[0];
    int last=(int)s[(int)s.size()-1];
    for (int i=0; i<(int)s.size(); i++)
    {
        if ((s[i] == 'i') ||
                (s[i] == 'l') ||
                (s[i] == 'I'))
            count++;
        if((int)s[i]==first)
            count2++;
        if((int)s[i]==last)
            count3++;
    }
    if ((count > ((int)s.size()+1)/2) || (count2 == (int)s.size()) || (count3 > ((int)s.size()*2)/3))
    {
        return true;
    }

    return false;
}

void er_draw(vector<Mat> &channels, vector<vector<ERStat> > &regions, vector<Vec2i> group, Mat& segmentation)
{
    for (int r=0; r<(int)group.size(); r++)
    {
        ERStat er = regions[group[r][0]][group[r][1]];
        if (er.parent != NULL) // deprecate the root region
        {
            int newMaskVal = 255;
            int flags = 4 + (newMaskVal << 8) + FLOODFILL_FIXED_RANGE + FLOODFILL_MASK_ONLY;
            floodFill(channels[group[r][0]],segmentation,Point(er.pixel%channels[group[r][0]].cols,er.pixel/channels[group[r][0]].cols),
                      Scalar(255),0,Scalar(er.level),Scalar(0),flags);
        }
    }
}

//定义一个转换的类
class AlphaDog
{
private:
    ros::NodeHandle nh_; //定义ROS句柄
    image_transport::ImageTransport it_; //定义一个image_transport实例
    image_transport::Subscriber image_sub_; //定义ROS图象接收器
    //image_transport::Publisher image_pub_; //定义ROS图象发布器
    /* Lzw - 11.21
     * For the topic publisher initialize
     */
    ros::Publisher doornumber_pub;
    bool analysisMode;
    ros::Subscriber command_sub;
    doornumber::boardArray boards;
    Mat imageProcessing;


    Mat g_srcImage, g_dstImage, g_grayImage, g_maskImage;//定义原始图、目标图、灰度图、掩模图
    int g_nFillMode = 1;//漫水填充的模式
    int g_nLowDifference = 40, g_nUpDifference = 40;//负差最大值、正差最大值
    int g_nConnectivity = 4;//表示floodFill函数标识符低八位的连通值
    int g_bIsColor = true;//是否为彩色图的标识符布尔值
    bool g_bUseMask = false;//是否显示掩膜窗口的布尔值
    int g_nNewMaskVal = 255;//新的重新绘制的像素值

    bool FLAG_FIRSTTIME_FLOODFILLROOM = true;

public:
    AlphaDog()
      :it_(nh_) //构造函数
    {
        /* Lzw - 11.21
         * For the topic publisher initialize
         */


        analysisMode = false;

        //初始化输入输出窗口
        cv::namedWindow(INPUT);
        cv::namedWindow(OUTPUT);
    }
    ~AlphaDog() //析构函数
    {
         cv::destroyWindow(INPUT);
         cv::destroyWindow(OUTPUT);
    }

    /*
     * new part for floormap recognition
     */

    void floodFillRoom(Point center){

        if(FLAG_FIRSTTIME_FLOODFILLROOM){
            imageProcessing.copyTo(g_srcImage);//拷贝源图到目标图
            g_srcImage.copyTo(g_dstImage);//拷贝源图到目标图
            cvtColor(g_srcImage, g_grayImage, COLOR_BGR2GRAY);//转换三通道的image0到灰度图
            g_maskImage.create(g_srcImage.rows+2, g_srcImage.cols+2, CV_8UC1);//利用image0的尺寸来初始化掩膜mask
            FLAG_FIRSTTIME_FLOODFILLROOM = false;
        }


        //-------------------【<1>调用floodFill函数之前的参数准备部分】---------------
        Point seed = center;
        int LowDifference = g_nFillMode == 0 ? 0 : g_nLowDifference;//空范围的漫水填充，此值设为0，否则设为全局的g_nLowDifference
        int UpDifference = g_nFillMode == 0 ? 0 : g_nUpDifference;//空范围的漫水填充，此值设为0，否则设为全局的g_nUpDifference
        int flags = g_nConnectivity + (g_nNewMaskVal << 8) +
            (g_nFillMode == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);//标识符的0~7位为g_nConnectivity，8~15位为g_nNewMaskVal左移8位的值，16~23位为CV_FLOODFILL_FIXED_RANGE或者0。

        //随机生成bgr值
        int b = (unsigned)theRNG() & 255;//随机返回一个0~255之间的值
        int g = (unsigned)theRNG() & 255;//随机返回一个0~255之间的值
        int r = (unsigned)theRNG() & 255;//随机返回一个0~255之间的值
        Rect ccomp;//定义重绘区域的最小边界矩形区域

        Scalar newVal = g_bIsColor ? Scalar(b, g, r) : Scalar(r*0.299 + g*0.587 + b*0.114);//在重绘区域像素的新值，若是彩色图模式，取Scalar(b, g, r)；若是灰度图模式，取Scalar(r*0.299 + g*0.587 + b*0.114)

        Mat dst = g_bIsColor ? g_dstImage : g_grayImage;//目标图的赋值
        int area;

        //--------------------【<2>正式调用floodFill函数】-----------------------------
        if( g_bUseMask )
        {
            threshold(g_maskImage, g_maskImage, 1, 128, CV_THRESH_BINARY);
            area = floodFill(dst, g_maskImage, seed, newVal, &ccomp, Scalar(LowDifference, LowDifference, LowDifference),
                Scalar(UpDifference, UpDifference, UpDifference), flags);
            imshow( "mask", g_maskImage );
        }
        else
        {
            area = floodFill(dst, seed, newVal, &ccomp, Scalar(LowDifference, LowDifference, LowDifference),
                Scalar(UpDifference, UpDifference, UpDifference), flags);
        }
        cout << area << " 个像素被重绘\n";

    }

    void getRoomAreas(){
        Mat image;
        imageProcessing.copyTo(image);

        for(int i=0; i< boards.boardArray.size(); i++){
            doornumber::board bd = boards.boardArray[i];

            printf("[bd %d] (%d, %d, %d, %d) \"%s\" %%%f \n", i, bd.tlx, bd.tly,
                   bd.brx, bd.bry, bd.text.c_str(), bd.confidence);

            // use floodfill to get every corner.
            int diffy = bd.tly - bd.bry;
            Point center(bd.tlx-diffy, bd.tly-diffy);
            floodFillRoom(center);

        }
        imshow("g_dstImage", g_dstImage);
        waitKey();
    }


    /*
      这是一个ROS和OpenCV的格式转换回调函数，将图象格式从sensor_msgs/Image  --->  cv::Mat
    */
    void convert_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        /* 11/22 Lzw
         * Here to confirm: only when Analysis Mode is on
         */
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

    void sub_dn_finder_command(const std_msgs::Bool::ConstPtr& msg)
    {
        if(msg->data == 1){
            openAnalysisMode();
            ROS_INFO("AnalysisMode: Open.");
        }
        else if(msg->data == 0){
            closeAnalysisMode();
            ROS_INFO("AnalysisMode: Close.");

        }
        else
        {
            ROS_ERROR("command signal Error: must be 1 or 2, but it's %d", msg->data);
        }
        return;
    }


    /*
       这是图象处理的主要函数，一般会把图像处理的主要程序写在这个函数中。这里的例子只是一个彩色图象到灰度图象的转化
    */
    void image_process(cv::Mat img)
    {
       cv::Mat img_out;
       cv::imshow(INPUT, img);
       img.copyTo(imageProcessing);
       img.copyTo(img_out);

       //GaussianBlur(img, img, Size(3,3) ,0 ,0);

       //cv::imshow(OUTPUT, img_out);
       cout << "this is the input image. press any key to continue..." <<endl;

       waitKey();

       cout << "start for processing..." << endl;
       //if(!analysisMode) return;

       // whether only analyze one frame each command.
       if(ONE_FRAME_ONE_TIME)
         analysisMode = false;

       img.copyTo(frame);

       double t_all = (double)getTickCount();

       if (downsize){
           resize(frame,frame,Size(320,240));
           cout << "downSize mode is on!!!" << endl;

       }

       /*Text Detection*/
       cvtColor(frame,gray,COLOR_BGR2GRAY);
       // Extract channels to be processed individually
       channels.clear();
       channels.push_back(gray);
       channels.push_back(255-gray);

       regions[0].clear();
       regions[1].clear();

       switch (REGION_TYPE)
       {
       case 0: // ERStats
           parallel_for_(cv::Range(0, (int)channels.size()), Parallel_extractCSER(channels, regions, er_filters1, er_filters2));
           break;
       case 1: // MSER
           vector<vector<Point> > contours;
           vector<Rect> bboxes;
           //Ptr<MSER> mser = MSER::create(21, (int)(0.00002*gray.cols*gray.rows), (int)(0.05*gray.cols*gray.rows), 1, 0.7);
           Ptr<MSER> mser = MSER::create(21, 150, 40000, 1, 0.7);
                      mser->detectRegions(gray, contours, bboxes);
           //Convert the output of MSER to suitable input for the grouping/recognition algorithms
           if (contours.size() > 0)
               MSERsToERStats(gray, contours, regions);
           break;
       }

       // Detect character groups
       vector< vector<Vec2i> > nm_region_groups;
       vector<Rect> nm_boxes;
       switch (GROUPING_ALGORITHM)
       {
       case 0: // exhaustive_search
           erGrouping(frame, channels, regions, nm_region_groups, nm_boxes, ERGROUPING_ORIENTATION_HORIZ);
           break;
       case 1: //multioriented
           erGrouping(frame, channels, regions, nm_region_groups, nm_boxes, ERGROUPING_ORIENTATION_ANY, "/home/liaoziwei/researches/rosspace/src/doornumber/xml/trained_classifier_erGrouping.xml", 0.5);
           break;
       }

       /*Text Recognition (OCR)*/

       int bottom_bar_height= img_out.rows/9 ;
       copyMakeBorder(frame, out_img, 0, bottom_bar_height, 0, 0, BORDER_CONSTANT, Scalar(150, 150, 150));
       float scale_font = (float)(bottom_bar_height /85.0);
       vector<string> words_detection;
       float min_confidence1 = 0.f, min_confidence2 = 0.f;

       if (RECOGNITION == 0)
       {
           min_confidence1 = 31.f;
           min_confidence2 = 40.f;
       }

       vector<Mat> detections;

       for (int i=0; i<(int)nm_boxes.size(); i++)
       {
           rectangle(out_img, nm_boxes[i].tl(), nm_boxes[i].br(), Scalar(255,255,0),3);

           Mat group_img = Mat::zeros(frame.rows+2, frame.cols+2, CV_8UC1);
           er_draw(channels, regions, nm_region_groups[i], group_img);
           group_img(nm_boxes[i]).copyTo(group_img);
           copyMakeBorder(group_img,group_img,15,15,15,15,BORDER_CONSTANT,Scalar(0));
           detections.push_back(group_img);
       }
       vector<string> outputs((int)detections.size());
       vector< vector<Rect> > boxes((int)detections.size());
       vector< vector<string> > words((int)detections.size());
       vector< vector<float> > confidences((int)detections.size());

       // parallel process detections in batches of ocrs.size() (== num_ocrs)
       for (int i=0; i<(int)detections.size(); i=i+(int)num_ocrs)
       {
           Range r;
           if (i+(int)num_ocrs <= (int)detections.size())
               r = Range(i,i+(int)num_ocrs);
           else
               r = Range(i,(int)detections.size());

           switch(RECOGNITION)
           {
           case 0: // Tesseract
               parallel_for_(r, Parallel_OCR<OCRTesseract>(detections, outputs, boxes, words, confidences, ocrs));
               break;
           case 1: // NM_chain_features + KNN
               parallel_for_(r, Parallel_OCR<OCRHMMDecoder>(detections, outputs, boxes, words, confidences, decoders));
               break;
           }
       }

       /*
        *
        * Go through a split operation, and after that
        * what is ouputs vector?  what does it mean by <3 and continue.
        */

       for (int i=0; i<(int)detections.size(); i++)
       {
           outputs[i].erase(remove(outputs[i].begin(), outputs[i].end(), '\n'), outputs[i].end());
           //cout << "OCR output = \"" << outputs[i] << "\" length = " << outputs[i].size() << endl;
           if (outputs[i].size() < 2)  // ----------------------- IMPORTANT default 3
               continue;

           for (int j=0; j<(int)boxes[i].size(); j++)
           {
               boxes[i][j].x += nm_boxes[i].x-15;
               boxes[i][j].y += nm_boxes[i].y-15;

               //cout << "  word = " << words[j] << "\t confidence = " << confidences[j] << endl;
               if ((words[i][j].size() < 2) || (confidences[i][j] < min_confidence1) ||
                       ((words[i][j].size()==2) && (words[i][j][0] == words[i][j][1])) ||
                       ((words[i][j].size()< 4) && (confidences[i][j] < min_confidence2)) ||
                       isRepetitive(words[i][j]))
                   continue;
               /*
                * Here is to publish the final result
                */
               doornumber::board newboard;
               newboard.tlx = boxes[i][j].tl().x;
               newboard.tly = boxes[i][j].tl().y;
               newboard.brx = boxes[i][j].br().x;
               newboard.bry = boxes[i][j].br().y;
               newboard.text = words[i][j];
               newboard.confidence = confidences[i][j];
               boards.boardArray.push_back(newboard);

               words_detection.push_back(words[i][j]);
               rectangle(out_img, boxes[i][j].tl(), boxes[i][j].br(), Scalar(255,0,255),3);
               Size word_size = getTextSize(words[i][j], FONT_HERSHEY_SIMPLEX, (double)scale_font, (int)(3*scale_font), NULL);
               rectangle(out_img, boxes[i][j].tl()-Point(3,word_size.height+3), boxes[i][j].tl()+Point(word_size.width,0), Scalar(255,0,255),-1);
               putText(out_img, words[i][j], boxes[i][j].tl()-Point(1,1), FONT_HERSHEY_SIMPLEX, scale_font, Scalar(255,255,255),(int)(3*scale_font));

               cout << newboard.text << endl;
           }
       }
       if((int)detections.size() < 1){
           doornumber::board emptyboard;
           boards.boardArray.push_back(emptyboard);
       }
       // doornumber_pub.publish(boards);

       t_all = ((double)getTickCount() - t_all)*1000/getTickFrequency();
       int text_thickness = 1+(out_img.rows/500);
       string fps_info = format("%2.1f Fps. %dx%d", (float)(1000 / t_all), frame.cols, frame.rows);
       putText(out_img, fps_info, Point( 10,out_img.rows-5 ), FONT_HERSHEY_DUPLEX, scale_font, Scalar(255,0,0), text_thickness);
       putText(out_img, region_types_str[REGION_TYPE], Point((int)(out_img.cols*0.5), out_img.rows - (int)(bottom_bar_height / 1.5)), FONT_HERSHEY_DUPLEX, scale_font, Scalar(255,0,0), text_thickness);
       putText(out_img, grouping_algorithms_str[GROUPING_ALGORITHM], Point((int)(out_img.cols*0.5),out_img.rows-((int)(bottom_bar_height /3)+4) ), FONT_HERSHEY_DUPLEX, scale_font, Scalar(255,0,0), text_thickness);
       putText(out_img, recognitions_str[RECOGNITION], Point((int)(out_img.cols*0.5),out_img.rows-5 ), FONT_HERSHEY_DUPLEX, scale_font, Scalar(255,0,0), text_thickness);

       imshow("recognition", out_img);

       cout <<"now have been shown. Press any key to continue..." <<endl;
       imwrite("//home//liaoziwei//Desktop//output.jpg",out_img);

       int key = waitKey(); //wait for a key press

       switch (key)
       {
       case 27: //ESC
           cout << "ESC key pressed and exited." << endl;
           return;
       case 32: //SPACE
           imwrite("recognition_alt.jpg", out_img);
           break;
       case 103: //'g'
           GROUPING_ALGORITHM = (GROUPING_ALGORITHM+1)%2;
           cout << "Grouping switched to " << grouping_algorithms_str[GROUPING_ALGORITHM] << endl;
           break;
       case 111: //'o'
           RECOGNITION = (RECOGNITION+1)%2;
           cout << "OCR switched to " << recognitions_str[RECOGNITION] << endl;
           break;
       case 114: //'r'
           REGION_TYPE = (REGION_TYPE+1)%2;
           cout << "Regions switched to " << region_types_str[REGION_TYPE] << endl;
           break;
       case 115: //'s'
           downsize = !downsize;
           if (!image.empty())
           {
               frame = image.clone();
           }
           break;
       default:
           break;
       }
    }
    void openAnalysisMode(){
        analysisMode = true;
    }
    void closeAnalysisMode(){
        analysisMode = false;
    }

};


//主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "DoorNumber");
    AlphaDog obj;

    CommandLineParser parser(argc, argv, keys);

    cout << "A demo program of End-to-end Scene Text Detection and Recognition using webcam or video." << endl << endl;
    cout << "  Keys:  " << endl;
    cout << "  Press 'r' to switch between MSER/CSER regions." << endl;
    cout << "  Press 'g' to switch between Horizontal and Arbitrary oriented grouping." << endl;
    cout << "  Press 'o' to switch between OCRTesseract/OCRHMMDecoder recognition." << endl;
    cout << "  Press 's' to scale down frame size to 320x240." << endl;
    cout << "  Press 'ESC' to exit." << endl << endl;
    parser.printMessage();

    namedWindow("recognition",WINDOW_NORMAL);
    //imshow("recognition", frame);
    waitKey(1);

    // Create ERFilter objects with the 1st and 2nd stage default classifiers
    // since er algorithm is not reentrant we need one filter for channel

    for (int i=0; i<2; i++)
    {
        Ptr<ERFilter> er_filter1 = createERFilterNM1(loadClassifierNM1("/home/liaoziwei/researches/rosspace/src/doornumber/xml/trained_classifierNM1.xml"),8,0.00015f,0.13f,0.2f,true,0.1f);
        Ptr<ERFilter> er_filter2 = createERFilterNM2(loadClassifierNM2("/home/liaoziwei/researches/rosspace/src/doornumber/xml/trained_classifierNM2.xml"),0.5);
        er_filters1.push_back(er_filter1);
        er_filters2.push_back(er_filter2);
    }

    //Initialize OCR engine (we initialize 10 instances in order to work several recognitions in parallel)
    cout << "Initializing OCR engines ... ";

    for (int o=0; o<num_ocrs; o++)
    {
        ocrs.push_back(OCRTesseract::create());
        ocrs[o]->setWhiteList("G0123456789-");
    }
    Mat transition_p;
    string filename = "/home/liaoziwei/researches/rosspace/src/doornumber/xml/OCRHMM_transitions_table.xml";
    FileStorage fs(filename, FileStorage::READ);
    fs["transition_probabilities"] >> transition_p;
    fs.release();
    Mat emission_p = Mat::eye(62,62,CV_64FC1);
    //string voc = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    string voc = "ABCDEFGHIJKL";
    //have no connection with the settings here.


    for (int o=0; o<num_ocrs; o++)
    {
        decoders.push_back(OCRHMMDecoder::create(loadOCRHMMClassifierNM("/home/liaoziwei/researches/rosspace/src/doornumber/xml/OCRHMM_knn_model_data.xml.gz"),
                           voc, transition_p, emission_p));
    }
    cout << " Done!" << endl;


    // now let's get a map
    Mat inMap = imread(argv[1]);
    printf("location: %s", argv[1]);
    if(inMap.data == NULL) cout << "error image location." <<endl;

    Mat inMap_resize;
    resize(inMap, inMap_resize, Size(inMap.cols/1, inMap.rows/1));
    obj.image_process(inMap_resize);
    obj.getRoomAreas();

    return 0;

}

