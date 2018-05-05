
/************

Important:
Sethome function has been forced to 0,0 0
*******************/
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include <ros/ros.h>
#include <doornumber/boardArray.h>
#include <doornumber/board.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
//#include "std_msgs/Float32.h"
#include "tf2_msgs/TFMessage.h"

// for atan2();
//#include <tgmath.h>
#include<math.h>


using namespace cv;  
using namespace std;  

#define PI 3.1415926
#define WINDOW_NAME "UI MAP"

#define SHOW_DEBUG_WINDOW false

// We send only one point to dn_finder
#define ONEPOINTMODE true

Mat image_highlight;

//Point startPoint(132, 778);
int start_x = 132;
int start_y = 778;

bool _flag_distance_change= false;
bool _flag_get_destination= false;

bool _flag_get_destcircle= false;

struct coordinate
{
    coordinate(float xin, float yin) {
        x = xin;
        y = yin;
    }
    coordinate() {
        x = 0;
        y = 0;
    }
    float x;
    float y;
};

void getScale(ros::NodeHandle n);
void odom2Map(vector<coordinate>& odom, vector<coordinate>& out);


class robotState{
public:
    Point home;
    Point now;
};

class odomTransfer
{
public:
    odomTransfer() {}
    void getMainMap(Mat&in, Mat&Out);
    void change2MapCoordinate(vector<Point> &searchPoints, vector<coordinate> &searchPointsOnMap);

    void humanMap2SlamMap(vector<Point> &searchPoints, vector<coordinate> &searchPointsOnMap);

    void outputVector(vector<coordinate> &points);

private:
    int leftBound;
    int rightBound;
    int upBound;
    int downBound;

    int fullMapRows;
    int fullMapCols;
};

enum MOUSE_MODES{
    MODES_NOTHING = 0,
    MODES_SELECTING_AREA = 1,
    MODES_SELECTED_AREA = 2

};

enum MOUSE_MODES_HOME{
    HOMEMODES_NOTHING = 0,
    HOMEMODES_SELECTING = 1,
    HOMEMODES_SELECTED = 2

};

enum SEARCH_STATE{
    SEARCH_WATITING_FOR_RESULT = 0,
    SEARCH_GET_RESULT = 1,
    SEARCH_FIND_GOLE = 2,
    SEARCH_STOP = 3

};

int mapCols, mapRows;

MOUSE_MODES mode = MODES_NOTHING;
MOUSE_MODES_HOME modeHome = HOMEMODES_NOTHING;
Point selected_points[2];
Point selected_points_home[2];
coordinate homePoint(0, 0);
float homeAngle = 0;

odomTransfer trans;
vector<coordinate> mapPoints;
vector<Point> searchPoints;

//For Destination Feedback
vector<Point> destinationPoints;
Point destinationPoint;


SEARCH_STATE search_result_flag = SEARCH_STOP;

ros::Publisher command_pub;
ros::Publisher scalecommand_pub;
ros::Publisher roompositioncommand_pub;

int DoorNumber = 416;
char* doornumber_string;

robotState robot;

Mat imageSelecting;
bool flag_getscale = false;
float scale = 0;
int vx, vy;
float match_rotate;

int vx_initial, vy_initial;

// when map changes, change this value
float Map_Theta0 = PI/2.0;

// for recording tf transformation   odom->map
vector< vector<float> >  tfTrans;


// restore final searching angle and points
float finalAngle = 0;
vector<coordinate> finalSearchPoints;


void drawHomeResult(Mat &matIn, Mat& matOut);

/*
 * the function declaration of class odomTransfer
 */
void odomTransfer::outputVector(vector<coordinate> &points){
    for(int i=0; i< points.size(); i++)
    {
        ROS_INFO("[%d]: (%f, %f)", i, points[i].x, points[i].y);
    }
}


// the newest function for changing pixels on human map to coordinate on slamap
void odomTransfer::humanMap2SlamMap(vector<Point> &searchPoints, vector<coordinate> &searchPointsOnMap){

    searchPointsOnMap.clear();

    for(int i=0; i<searchPoints.size(); i++){

        // translation
        int x = searchPoints[i].x + vx;
        int y = searchPoints[i].y + vy;


        // scale : compared with yaml files's settings.
        //          result: there is no connection.
        int x_scaled = int((x - 2000) * scale);
        int y_scaled = int((y - 2000) * scale);

        // rotation to zero   90
        int x_scaled_rotated = x_scaled * cos(Map_Theta0 + match_rotate) -y_scaled * sin(Map_Theta0 + match_rotate);
        int y_scaled_rotated = x_scaled * sin(Map_Theta0 + match_rotate) +y_scaled * cos(Map_Theta0 + match_rotate);


        // pixel to coordinate
        float realx,realy;
        realx = (x_scaled_rotated)/4000.0 *2.0 * 100.0;
        realy = -(y_scaled_rotated)/4000.0 *2.0 * 100.0;

        //ROS_INFO("realx:%f, realy:%f \n", realx, realy);

        coordinate temp(realx, realy);
        searchPointsOnMap.push_back(temp);

    }
}

void odomTransfer::change2MapCoordinate(vector<Point> &searchPoints, vector<coordinate> &searchPointsOnMap){

    searchPointsOnMap.clear();

    for(int i=0; i<searchPoints.size(); i++){

        int x = searchPoints[i].x + leftBound;
        int y = searchPoints[i].y + upBound;

        float realx,realy;
        realx = (x - fullMapCols/2.0)/fullMapCols *2.0 * 100.0;
        realy = -(y - fullMapRows/2.0)/fullMapRows *2.0 * 100.0;

        //ROS_INFO("realx:%f, realy:%f \n", realx, realy);

        coordinate temp(realx, realy);
        searchPointsOnMap.push_back(temp);

    }
}


void odomTransfer::getMainMap(Mat &in, Mat &Out){
    // from center to find all is 205
    fullMapCols = in.cols;
    fullMapRows = in.rows;

    Mat img;
    in.copyTo(img);


    int rows = img.rows;
    int cols = img.cols;

    bool allIsBlank = true;

    leftBound = 0;
    upBound = 0;
    rightBound = cols;
    downBound = rows;

    Point center;
    center.x = img.rows/2;
    center.y = img.cols/2;


    // To downbound
    for (int i = rows/2; i < rows; i++)
    {
        allIsBlank = true;
        for (int j = 0; j < cols; j++)
        {
            int value = img.at<uchar>(i, j);
            if(value != 205)
            {
                //cout<<"!205: " << i << "," << j << endl;
                allIsBlank = false;
                break;
            }

        }
        // if the line is all 205
        if(allIsBlank){
            downBound = i;
            break;
        }
    }

    // To upbound
    for (int i = rows/2; i > 0; i--)
    {
        allIsBlank = true;

        for (int j = 0; j < cols; j++)
        {
            int value = img.at<uchar>(i, j);
            if(value != 205)
            {
                allIsBlank = false;
                break;
            }

        }
        // if the line is all 205
        if(allIsBlank){
            upBound = i;
            break;
        }
    }

    // To rightbound
    for (int i = cols/2; i < cols; i++)
    {
        allIsBlank = true;

        for (int j = 0; j < rows; j++)
        {
            int value = img.at<uchar>(j, i);
            if(value != 205)
            {
                allIsBlank = false;
                break;
            }

        }
        // if the line is all 205
        if(allIsBlank){
            rightBound = i;
            break;
        }
    }

    // To leftbound
    for (int i = cols/2; i > 0; i--)
    {
        allIsBlank = true;

        for (int j = 0; j < rows; j++)
        {
            int value = img.at<uchar>(j, i);
            if(value != 205)
            {
                allIsBlank = false;
                break;
            }

        }
        // if the line is all 205
        if(allIsBlank){
            leftBound = i;
            break;
        }
    }

    ROS_INFO("left:%d up:%d right:%d down:%d \n", leftBound, upBound, rightBound, downBound);

    Mat matROI(in, Rect(leftBound, upBound, rightBound-leftBound, downBound-upBound));
    matROI.copyTo(Out);

    return;

}

void drawElipse(Mat &showImage, int mode = 0){
    Point center;
    center.x = (selected_points[0].x + selected_points[1].x)/2;
    center.y = (selected_points[0].y + selected_points[1].y)/2;

    double diff_x, diff_y;
    diff_x = selected_points[0].x - selected_points[1].x;
    diff_y = selected_points[0].y - selected_points[1].y;

    double distance;
    distance = powf(diff_x,2) + powf(diff_y,2);
    distance = sqrtf(distance);

    float theta;
    if(diff_x == 0 && diff_y > 0)
        theta = 90;
    else if(diff_x == 0 && diff_y < 0)
        theta = -90;

    else
        theta = atan (diff_y / diff_x) * 180 / PI;
    //    theta = atan (diff_y / diff_x);

    if(mode == 0)
    //参数为：承载的图像、圆心、长短轴、径向夹角（水平面到长轴的夹角）、起始角度（长轴到起始边沿的夹角）、结束角度（长轴到结束点的夹角）、倾斜的矩形（可选项）、颜色、粗细、线性、偏移
        ellipse(showImage,center,Size( distance, distance/5 ),theta,0,360,Scalar(0,0,0));
    else
        ellipse(showImage,center,Size( distance, distance/5 ),theta,0,360,255);


    return;
}

int corridorSearchPoints(Vec4i wallLine, Mat &maskImage, vector<Point> &wallPoints, vector<Point> &searchPoints, float dis){
    // find out which side of the wall is corridor
    Mat showImage;
    cvtColor(maskImage, showImage, CV_GRAY2BGR);


    float diff_x = wallLine[2] - wallLine[0];
    float diff_y = wallLine[3] - wallLine[1];
    float theta;
    if(diff_x == 0 && diff_y > 0)
        theta = PI/2;
    else if(diff_x == 0 && diff_y < 0)
        theta = -PI/2;
    else
        theta = atan (diff_y / diff_x);

    // NOW Get the corridor direction.
    int corridor_direction = 0;


    bool near_level = false;
    float theta_cor = theta + PI/2.0;
    if(theta_cor > PI/2.0) theta_cor -= PI;
    if(abs(theta_cor) < PI/360.0 || abs(theta_cor - PI) < PI/360.0)
    {
        // cout << "the wall is near level. This part is waited to be finished." << endl;
        cout << "the wall is near Level, but I have finished it haha." << endl;
        near_level = true;

    }

    Point center;
    center.x = (wallLine[0] + wallLine[2])/2;
    center.y = (wallLine[1] + wallLine[3])/2;

    int increase_cornum = 0;
    // get 5 points from both sides
    for(int i=0; i<10; i++)
    {
        Point temp;
        if(near_level == false){
            temp.x = center.x + 2 * i;
            temp.y = center.y + 2*i*tan(theta_cor);
        }
        else
        {
            temp.x = center.x + 2 * i;
            temp.y = center.y;
        }
        int value = static_cast<int>(image_highlight.at<uchar>(temp));
        cout << i << ", " << value << endl;
        if(value == 255)
        {
            // Unknown Area
            circle(showImage, temp, 3,Scalar(0,0,255),-1); //第五个参数我设为-1，表明这是个实点。

        }
        else if(value == 125)
        {
            increase_cornum++;
            circle(showImage, temp, 3,Scalar(0,255,255),-1); //第五个参数我设为-1，表明这是个实点。

        }
        else if(value == 0)
        {
            // still the wall
            continue;
        }
    }

    int decrease_cornum = 0;
    // get 5 points from both sides
    for(int i=0; i<10; i++)
    {
        Point temp;
        if(near_level == false){
            temp.x = center.x - 2 * i;
            temp.y = center.y - 2*i*tan(theta_cor);
        }
        else
        {
            temp.x = center.x - 2 * i;
            temp.y = center.y;
        }
        int value = static_cast<int>(image_highlight.at<uchar>(temp));
        cout << i << ", " << value << endl;

        if(value == 255)
        {
            // Unknown Area
            circle(showImage, temp, 1,Scalar(0,0,255),-1); //第五个参数我设为-1，表明这是个实点。

        }
        else if(value == 125)
        {
            decrease_cornum++;
            circle(showImage, temp, 1,Scalar(0,255,255),-1); //第五个参数我设为-1，表明这是个实点。

        }
        else if(value == 0)
        {
            // still the wall
            continue;
        }
    }

    cout << "increase: " << increase_cornum << " decrease:" << decrease_cornum << endl;
    if(increase_cornum > decrease_cornum){
        corridor_direction = 1;
    }
    else
        corridor_direction = -1;

    if(SHOW_DEBUG_WINDOW){
        cv::imshow("detecting corridor direction", showImage);
        waitKey();
    }

    // calculate the final points.
    float delta_x = dis * cos(theta_cor);
    float delta_y = dis * sin(theta_cor);

    printf("wallPoints.size = %d", int(wallPoints.size()));
    for(int i=0; i< wallPoints.size(); i++)
    {
        Point temp;
        temp.x = wallPoints[i].x + corridor_direction * delta_x;
        temp.y = wallPoints[i].y + corridor_direction * delta_y;

        searchPoints.push_back(temp);
    }


    return 1;

}

// interval: the interval of Points( Unit: cm)
void getSearchPoint(Mat &maskImage, vector<Point> &searchPoints, float interval = 100, float dist = 10, int maxPoint = 10){
    Mat Image;
    maskImage.copyTo(Image);
    //cvtColor(Image, Image, CV_BGR2GRAY);

    searchPoints.clear();

//    Mat CannyImg;
//    Canny(Image, CannyImg, 140, 250, 3);
//    if(SHOW_DEBUG_WINDOW)
//         cv::imshow("CannyImg", CannyImg);

    Mat DstImg;
    cvtColor(Image, DstImg, CV_GRAY2BGR);

    vector<Vec4i> Lines;
    HoughLinesP(maskImage, Lines, 5, CV_PI / 180, 30,30,15);

    int longest_num = 0;
    double longest_dis = 0;
    for (int i = 0; i < Lines.size(); i++)
    {
        double distance;
        distance = powf(Lines[i][0]-Lines[i][2],2) + powf(Lines[i][1]-Lines[i][3],2);
        distance = sqrtf(distance);
        ROS_INFO("L%d: distance: %.1f \n", i, distance);
        line(DstImg, Point(Lines[i][0], Lines[i][1]), Point(Lines[i][2], Lines[i][3]), Scalar(0, 0, 255), 1, 8);
        if(distance > longest_dis){
            longest_num = i;
            longest_dis = distance;
            printf("longgest_num %d, distance %f\n", i, distance);

        }
    }

    Vec4i wallLine;
    wallLine = Lines[longest_num];
    line(DstImg, Point(wallLine[0], wallLine[1]), Point(wallLine[2], wallLine[3]), Scalar(0, 255, 255), 3, 8);

    if(SHOW_DEBUG_WINDOW)
        cv::imshow("HoughLines_Detect", DstImg);
    imwrite("HoughLines_Detect.jpg", DstImg);
    //waitKey(0);

    // wallLine stores the start and end Point of one wallLine.
    // calculate these points.

    float x0, x1;
    float y0, y1;
    if(wallLine[0] < wallLine[2])
    {
        x0 = wallLine[0];
        x1 = wallLine[2];
    }
    else
    {
        x1 = wallLine[0];
        x0 = wallLine[2];
    }

    if(wallLine[1] < wallLine[3])
    {
        y0 = wallLine[1];
        y1 = wallLine[3];
    }
    else
    {
        y1 = wallLine[1];
        y0 = wallLine[3];
    }

    float diff_x = x1 - x0;
    float diff_y = y1 - y0;

    float theta;
    if( abs(diff_x) < PI/180.0 && diff_y > 0)
        theta = PI/2;
    else if(abs(diff_x) < PI/180.0 && diff_y < 0)
        theta = -PI/2;
    else
        theta = atan (diff_y / diff_x);

    float delta_x = interval * cos(theta);
    float delta_y = interval * sin(theta);

    vector<Point> wallPoints;
    for(int i=0; i < maxPoint ; i++){
        Point newPoint;
        newPoint.x = x0 + delta_x * i;
        newPoint.y = y0 + delta_y * i;

        // to avoid out of the Line
        if(newPoint.y > y1){
            printf("delta xy (%f,%f) newPoint.y %d wallLine[3] %d" , delta_x, delta_y, newPoint.y, wallLine[3]);
            break;
        }

        wallPoints.push_back(newPoint);
    }

    if(wallPoints.size() < 1){
        printf("wallPoints.size = %d, please check your params.", int(wallPoints.size()));
        //return;
    }

    corridorSearchPoints(wallLine, maskImage, wallPoints, searchPoints, dist);

    return;
}


void modes_selecting_update(int x, int y, void* param){
    Mat *im = reinterpret_cast<Mat*>(param);
    Mat showImage;
    im->copyTo(showImage);

    selected_points[1].x = x;
    selected_points[1].y = y;

    drawElipse(showImage);

    cv::imshow(WINDOW_NAME, showImage);

}

void modes_selecting_initial(int x, int y){
    cout << "Now is Selecting Mode." << endl;
    selected_points[0].x = x;
    selected_points[0].y = y;
}

void modes_selected_initial(int x, int y, void* param){
    cout << "Now is Selected Mode." << endl;

    Mat *im = reinterpret_cast<Mat*>(param);
    Mat showImage;
    cvtColor(*im,showImage, CV_GRAY2BGR);

    // begining getting ROI, and get the line.
    Mat mask = Mat::zeros(im->size(), CV_8UC1);

    // floodfill to get the mask.
    Point center;
    center.x = (selected_points[0].x + selected_points[1].x)/2;
    center.y = (selected_points[0].y + selected_points[1].y)/2;
    drawElipse(mask, 1);

    //漫水填充
    //pi的值表示为 v(pi),if  v(seed)-loDiff<v(pi)<v(seed)+upDiff,将pi的值设置为newVal
    floodFill(mask, center, 255, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);

    Mat verseImage;
    im->copyTo(verseImage);
    int height;
    int width;
    height= verseImage.rows;
    width= verseImage.cols* verseImage.channels();   // 列项要乘通道数
    //图像反转,
    for(int i= 0; i< height; i++)
    {
        for(int j=0; j< width; j++)
        {
            verseImage.at<uchar>(i, j)= 255- verseImage.at<uchar>(i, j);   // 每一个像素反转
        }
    }

    Mat maskImage;
    verseImage.copyTo(maskImage, mask);

    if(SHOW_DEBUG_WINDOW)
        cv::imshow("mask", maskImage);
    //cv::imshow("imask",mask);
    //waitKey();

    // find the line and the main point
    getSearchPoint(maskImage, searchPoints, 25, 22, 10);
    ROS_INFO("total points: %d \n", (int)searchPoints.size());

    // draw the points for test
    for(int i=0;i<searchPoints.size();i++){
        circle(showImage, searchPoints[i], 3,Scalar(0,0,255),-1); //第五个参数我设为-1，表明这是个实点。
    }

    // draw home point

    Mat matShow;
    drawHomeResult(showImage, matShow);

    cv::imshow(WINDOW_NAME, matShow);
    //waitKey();

    return;

}

void modes_nothing_initial(int x, int y, void* param){
    cout << "Now is nothing Mode." << endl;
    Mat *im = reinterpret_cast<Mat*>(param);

    cv::imshow(WINDOW_NAME, *im);


}

/*
 * For the home setting function.
 */
void drawHomeResult(Mat &matIn, Mat& matOut){
    Mat image;
    matIn.copyTo(image);
    if(modeHome == HOMEMODES_SELECTING)
    {
        circle(image, selected_points_home[0], 10,Scalar(255,0,0),-1); //第五个参数我设为-1，表明这是个实点。
        image.copyTo(matOut);
    }
    else if (modeHome == HOMEMODES_SELECTED)
    {
        circle(image, selected_points_home[0], 10,Scalar(255,0,0),-1); //第五个参数我设为-1，表明这是个实点。
        circle(image, selected_points_home[1], 10,Scalar(255,0,0),-1); //第五个参数我设为-1，表明这是个实点。
        line(image,selected_points_home[0],selected_points_home[1],Scalar(0,0,255));
        image.copyTo(matOut);

    }
    else
    {
        // have not chosen the home point
        image.copyTo(matOut);

    }
    return;
}

void modes_selecting_initial_home(int x, int y, void* param){
    cout << "Now is Selecting Mode - Home." << endl;
    selected_points_home[0].x = x;
    selected_points_home[0].y = y;

    Mat *im = reinterpret_cast<Mat*>(param);

    Mat matShow;
    cvtColor(*im, matShow, CV_GRAY2BGR);

    drawHomeResult(matShow, matShow);

    cv::imshow(WINDOW_NAME, matShow);

}

// About the home point,
// It's useless for the new application environment.
void modes_selected_initial_home(int x, int y, void* param){
    cout << "Now is Selected Mode - Home." << endl;

    selected_points_home[1].x = x;
    selected_points_home[1].y = y;

    float diff_x = selected_points_home[1].x - selected_points_home[0].x;
    float diff_y = selected_points_home[1].y - selected_points_home[0].y;
    float theta;
    diff_y = -diff_y;  // y is towarding down

    if(diff_x == 0 && diff_y > 0)
        theta = PI/2;
    else if(diff_x == 0 && diff_y < 0)
        theta = 3*PI/2;
    else if(diff_x > 0 && diff_y > 0)
        theta = atan (diff_y / diff_x);
    else if(diff_x > 0 && diff_y < 0)
        theta = 2*PI + atan (diff_y / diff_x);
    else if (diff_x < 0)
        theta = PI + atan (diff_y / diff_x);
    else theta = 0;
    homeAngle = theta;
    // angle has been ignored in ros_along_road params.

    vector<Point> temp;
    temp.push_back(selected_points_home[0]);

    vector<coordinate> temp_coor;
    //trans.change2MapCoordinate(temp, temp_coor);

    homePoint.x = temp[0].x;
    homePoint.y = temp[0].y;



    ROS_INFO("Home Position: (%d, %d) Theta:%f Coor:(%f,%f)", selected_points_home[0].x, selected_points_home[0].y, homeAngle, homePoint.x, homePoint.y);

    Mat *im = reinterpret_cast<Mat*>(param);

    Mat matShow;
    cvtColor(*im, matShow, CV_GRAY2BGR);

    drawHomeResult(matShow, matShow);

    cv::imshow(WINDOW_NAME, matShow);

    return;

}

void modes_nothing_initial_home(int x, int y, void* param){
    cout << "Now is nothing Mode." << endl;
    Mat *im = reinterpret_cast<Mat*>(param);

    cv::imshow(WINDOW_NAME, *im);


}

//1.回调函数签名  
void onMouse(int event, int x, int y, int flags, void* param);  
  
//2.定义回调函数  
void onMouse(int event, int x, int y, int flags, void* param)  
{  
    Mat *im = reinterpret_cast<Mat*>(param);  
    switch (event)  
    {  
        case CV_EVENT_LBUTTONDOWN:
        {  

            if(ONEPOINTMODE)
            {
                destinationPoint.x = x;
                destinationPoint.y = y;
                destinationPoints.clear();
                destinationPoints.push_back(destinationPoint);
                printf("Destination: x %d, y %d\n", x, y);
                Mat *im = reinterpret_cast<Mat*>(param);

                Mat matShow;
                cvtColor(*im, matShow, CV_GRAY2BGR);

                Mat image;
                matShow.copyTo(image);

                circle(image, destinationPoint, 10,Scalar(0,255,255),-1); //第五个参数我设为-1，表明这是个实点。
                circle(image, destinationPoint, 38,Scalar(0,0,255),3); //第五个参数我设为-1，表明这是个实点。

                cv::imshow(WINDOW_NAME, image);
                break;
            }

            else{
                switch (mode)
                {
                    case MODES_NOTHING:
                         mode = MODES_SELECTING_AREA;
                        modes_selecting_initial(x, y);
                        break;
                    case MODES_SELECTING_AREA:
                         mode = MODES_SELECTED_AREA;
                        modes_selected_initial(x, y, param);
                        break;
                    case MODES_SELECTED_AREA:
                         mode = MODES_NOTHING;
                        modes_nothing_initial(x, y, param);
                        break;
                 }
            }

            break;

        }
        case CV_EVENT_MBUTTONDOWN:
        {
            // for set the home point
            if(mode == MODES_NOTHING || mode == MODES_SELECTED_AREA)
            {
                switch (modeHome)
                {
                    case HOMEMODES_NOTHING:
                         modeHome = HOMEMODES_SELECTING;
                        modes_selecting_initial_home(x, y, param);
                        break;
                    case HOMEMODES_SELECTING:
                         modeHome = HOMEMODES_SELECTED;
                        modes_selected_initial_home(x, y, param);
                        break;
                    case HOMEMODES_SELECTED:
                         modeHome = HOMEMODES_NOTHING;
                        modes_nothing_initial_home(x, y, param);
                        break;
                 }
            }
            break;
        }

        case CV_EVENT_RBUTTONDOWN:
        {
            std::cout<<"at("<<x<<","<<y<<")value is:"
                <<static_cast<int>(im->at<uchar>(cv::Point(x,y)))<<std::endl;

        // the true (x,y)
            float realx,realy;
            realx = (x - mapCols/2.0)/mapCols *2.0 * 100.0;
            realy = -(y - mapRows/2.0)/mapRows *2.0 * 100.0;
            ROS_INFO("Real Point x:%f y:%f\n", realx, realy);


            break;
        }
    case CV_EVENT_MOUSEMOVE:
        if(mode == MODES_SELECTING_AREA)
            modes_selecting_update(x, y, param);
        break;
    }  
}  

void searchResult_deal(const doornumber::boardArray::ConstPtr& msg){
    ROS_INFO_STREAM( "get result. total boards number: " << msg->boardArray.size() << endl);
    search_result_flag = SEARCH_GET_RESULT;

    if(msg->boardArray.size() > 0){
        for (int i=0; i<msg->boardArray.size(); ++i)
        {
            const doornumber::board &board = msg->boardArray[i];
            ROS_INFO_STREAM("text: " << board.text << "confidence: " << board.confidence);
            search_result_flag = SEARCH_FIND_GOLE;
        }
    }
    else
        ROS_INFO("Find no destination this time. Continue on next point.");



    return;
}

void map_distance_change_deal(const std_msgs::Bool::ConstPtr& msg){

    if(msg->data == true){
        // begin a new scale calculate
        _flag_distance_change = true;
        printf("Get the command for distance change.");

    }
    else
    {
        // the search is ending, change to door plate searching.
        _flag_get_destcircle = true;
        printf("Get the command for GETTING INTO THE DESTINATION CIRCLE.");

    }

    return;
}

void map_scale_deal(const std_msgs::Float32MultiArray::ConstPtr& msg){
    flag_getscale = true;
    scale = msg->data[0];
    vx = (int)msg->data[1];
    vy = (int)msg->data[2];
    match_rotate = msg->data[3];
    printf("get scale %f, vx %d, vy %d, rotate %f\n", scale, vx, vy, match_rotate);
    return;
}

double computeDistance(Point p1, Point p2){
    double diff_x, diff_y;
    diff_x = p1.x - p2.x;
    diff_y = p1.y - p2.y;

    double distance;
    distance = powf(diff_x,2) + powf(diff_y,2);
    distance = sqrtf(distance);

    return distance;
}

// The function is for guiding robot to the goal place nearby.
int toFirstPointAndUpdateScale(int firstID, ros::NodeHandle n){
BeginICHIDO:    ros::Rate loop_rate(1);

    std_msgs::Float32MultiArray commandPoint;
    commandPoint.data.clear();
    commandPoint.data.push_back(mapPoints[firstID].x);
    commandPoint.data.push_back(mapPoints[firstID].y);
    commandPoint.data.push_back(homePoint.x);
    commandPoint.data.push_back(homePoint.y);
    commandPoint.data.push_back(homeAngle);
//    commandPoint.data.push_back(0);
//    commandPoint.data.push_back(0);
//    commandPoint.data.push_back(0.0);
    commandPoint.data.push_back(DoorNumber);

    ROS_INFO("Step1: Go to searching goal [%d] (%f, %f) ...\n", firstID, mapPoints[firstID].x, mapPoints[firstID].y);
    command_pub.publish(commandPoint);

    search_result_flag = SEARCH_WATITING_FOR_RESULT;
    ROS_INFO("[first]waitting for receiving data...\n");
    while(search_result_flag==SEARCH_WATITING_FOR_RESULT){
        ros::spinOnce();

        if(_flag_distance_change == true){
            // it's time to update a new scale and send command again
            goto NewICHIDO;
        }
        if(_flag_get_destcircle == true){
            ROS_INFO("Step1: Searching Task Succeeds! Cheeeeeeers! Then Find the Door Plate.");
            // wait for sending result. Then quit.
            _flag_get_destcircle = false;
            // change back. to wait for changing again.
            return 1;
        }
        loop_rate.sleep();
    }
    ROS_INFO("[first]get result.");
    if(search_result_flag == SEARCH_FIND_GOLE )
    {
        search_result_flag = SEARCH_STOP;
        return 1;
    }
NewICHIDO:    getScale(n);
    //vector<coordinate> odomPoints;
    trans.humanMap2SlamMap(destinationPoints, mapPoints);
    //odom2Map(odomPoints, mapPoints);

    if(_flag_distance_change){
        _flag_distance_change = false;
        goto BeginICHIDO;
    }
}

// Old Version: When adding search and go, its structure becomes confusing, so just
// create a new one.
//void startSearching(int argc, char** argv, ros::NodeHandle n){


//    ros::Rate loop_rate(1);


//    ROS_INFO("[Searching]begin sending searching goal. \n");

//    // to make sure robots search from the nearest point.
//    float distance_toStart = computeDistance(selected_points_home[0], searchPoints[0]);
//    float distance_toFinal = computeDistance(selected_points_home[0], searchPoints[searchPoints.size() - 1]);
//    if(distance_toStart < distance_toFinal){
//        // start from point 0
//        if(toFirstPointAndUpdateScale(0, n)) goto FINISH;
//        for(int i=0;i<mapPoints.size(); i++){

//            std_msgs::Float32MultiArray commandPoint;
//            commandPoint.data.push_back(mapPoints[i].x);
//            commandPoint.data.push_back(mapPoints[i].y);
//            commandPoint.data.push_back(homePoint.x);
//            commandPoint.data.push_back(homePoint.y);
//            commandPoint.data.push_back(homeAngle);
////                commandPoint.data.push_back(0);
////                commandPoint.data.push_back(0);
////                commandPoint.data.push_back(0.0);
//            commandPoint.data.push_back(DoorNumber);


//            ROS_INFO("Sending, searching goal [%d] (%f, %f) ...\n", i, mapPoints[i].x, mapPoints[i].y);
//            command_pub.publish(commandPoint);

//            search_result_flag = SEARCH_WATITING_FOR_RESULT;
//            ROS_INFO("[Searching ++]waitting for receiving data...\n");

//            while(search_result_flag==SEARCH_WATITING_FOR_RESULT){
//                ros::spinOnce();
//                loop_rate.sleep();
//            }
//            ROS_INFO("[Searching]get result.");
//            if(search_result_flag == SEARCH_FIND_GOLE )
//            {
//                search_result_flag = SEARCH_STOP;
//                break;
//            }
//            // deal with the result

//        }
//    }
//    else
//    {
//        // go to the first point and then update the points using newest scale.
//        if(toFirstPointAndUpdateScale(mapPoints.size()-1, n)) goto FINISH;
//        for(int i=mapPoints.size()-1;i>0; i--){

//            std_msgs::Float32MultiArray commandPoint;
//            commandPoint.data.push_back(mapPoints[i].x);
//            commandPoint.data.push_back(mapPoints[i].y);
//                commandPoint.data.push_back(homePoint.x);
//                commandPoint.data.push_back(homePoint.y);
//                commandPoint.data.push_back(homeAngle);
////                commandPoint.data.push_back(0);
////                commandPoint.data.push_back(0);
////                commandPoint.data.push_back(0.0);
//                commandPoint.data.push_back(DoorNumber);


//            ROS_INFO("Sending, searching goal [%d] (%f, %f) ...\n", i, mapPoints[i].x, mapPoints[i].y);
//            command_pub.publish(commandPoint);

//            search_result_flag = SEARCH_WATITING_FOR_RESULT;
//            ROS_INFO("[Searching --]waitting for receiving data...\n");

//            while(search_result_flag==SEARCH_WATITING_FOR_RESULT){
//                ros::spinOnce();
//                loop_rate.sleep();
//            }
//            ROS_INFO("get result.");
//            if(search_result_flag == SEARCH_FIND_GOLE )
//            {
//                search_result_flag = SEARCH_STOP;
//                break;
//            }
//            // deal with the result

//        }
//    }


//FINISH:    ROS_INFO("Step1: Searching Task Succeeds! Cheeeeeeers! Then Find the Door Plate.");

//    // wait until getting the search points.
//    while(_flag_get_destcircle == false) ;



//    std_msgs::Float32MultiArray commandPoint;
//    commandPoint.data.push_back(0);
//    commandPoint.data.push_back(0);
//    commandPoint.data.push_back(homePoint.x);
//    commandPoint.data.push_back(homePoint.y);
//    commandPoint.data.push_back(homeAngle);
////                commandPoint.data.push_back(0);
////                commandPoint.data.push_back(0);
////                commandPoint.data.push_back(0.0);
//    commandPoint.data.push_back(DoorNumber);


//    command_pub.publish(commandPoint);

//    return;

//}

void stopRobots(){
    std_msgs::Float32MultiArray commandPoint;
    commandPoint.data.clear();
    commandPoint.data.push_back(0.0);
    commandPoint.data.push_back(0.0);
    commandPoint.data.push_back(homePoint.x);
    commandPoint.data.push_back(homePoint.y);
    commandPoint.data.push_back(homeAngle);
//    commandPoint.data.push_back(0);
//    commandPoint.data.push_back(0);
//    commandPoint.data.push_back(0.0);
    commandPoint.data.push_back(DoorNumber);

    ROS_INFO("STOP ROBOTS COMMAND SENT.");
    command_pub.publish(commandPoint);
}

void startSearching_GoAndSearch(int argc, char** argv, ros::NodeHandle n){


    ros::Rate loop_rate(1);


    ROS_INFO("[Searching]Step1: begin sending searching goal. \n");

    if(!toFirstPointAndUpdateScale(0, n)) return;
    ROS_INFO("[Searching]Step2: Searching Task Succeeds! Cheeeeeeers! Wait for points.");

    stopRobots();

    // wait until getting the search points.
    while(_flag_get_destcircle == false) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("[Searching]Step3: Get Points. Start searching...");

    for(int i=0;i<finalSearchPoints.size(); i++){

        std_msgs::Float32MultiArray commandPoint;
        commandPoint.data.push_back(finalSearchPoints[i].x);
        commandPoint.data.push_back(finalSearchPoints[i].y);
        commandPoint.data.push_back(homePoint.x);
        commandPoint.data.push_back(homePoint.y);
        //commandPoint.data.push_back(homeAngle);
        commandPoint.data.push_back(finalAngle);

        commandPoint.data.push_back(DoorNumber);


        ROS_INFO("Sending goal [%d] (%f, %f) Angle %f ...\n", i, finalSearchPoints[i].x, finalSearchPoints[i].y, finalAngle);
        command_pub.publish(commandPoint);

        search_result_flag = SEARCH_WATITING_FOR_RESULT;
        ROS_INFO("[Searching ++]waitting for receiving data...\n");

        while(search_result_flag==SEARCH_WATITING_FOR_RESULT){
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_INFO("[Searching]get result.");
        if(search_result_flag == SEARCH_FIND_GOLE )
        {
            search_result_flag = SEARCH_STOP;
            break;
        }
        // deal with the result

    }

    return;

}

void getScale(ros::NodeHandle n){

    std_msgs::Float32MultiArray command;
    command.data.push_back(homePoint.x);
    command.data.push_back(homePoint.y);
    command.data.push_back(homeAngle);

    scalecommand_pub.publish(command);

    ros::Rate loop_rate(1);
    printf("wait for getting scale result. continue on sending...\n");

    while(!flag_getscale){
        scalecommand_pub.publish(command);

        ros::spinOnce();
        loop_rate.sleep();
    }
    flag_getscale = false;


    printf("Get Scale: %f angle: %f\n", scale, match_rotate);

    return;
}

void subroomPosition(const std_msgs::Float32MultiArray::ConstPtr& msg){
    printf("Get Position.\n");
}

void subsearchCommand_points(const std_msgs::Float32MultiArray::ConstPtr& msg){
    printf("Get Final Searching Points. number: %d\n", int(msg->data.size()));
    _flag_get_destcircle = true;

    // data Lists:
    // 0 : angle
    // 1 - n : (x,y)...

    if(msg->data.size() < 3){
        printf("Data result is not correct.\n");
        return;
    }

    finalAngle = msg->data[0] - 3.14159/2.0;
    finalSearchPoints.clear();
    for(int i=0; i<(msg->data.size()/2);i++){
        coordinate p;
        p.x = msg->data[i*2+1];
        p.y = msg->data[i*2+2];
        finalSearchPoints.push_back(p);
    }



}

void subtf(const tf2_msgs::TFMessage::ConstPtr& data){
    if (data->transforms[0].header.frame_id == "map" && data->transforms[0].child_frame_id == "odom"){
       float     x = data->transforms[0].transform.translation.x;
       float     y = data->transforms[0].transform.translation.y;
       float     qz = data->transforms[0].transform.rotation.z;
       float     qw = data->transforms[0].transform.rotation.w;
       float angle = atan2(2*qw*qz, 1-2*qz*qz);
       vector<float> tf;
       tf.push_back(x);
       tf.push_back(y);
       tf.push_back(angle);
       tfTrans.push_back(tf);
       printf("get new tf trans  map->odom. (%f,%f,%f),total:%d\n", x, y, angle, int(tfTrans.size()));
    }
}

void odom2Map(vector<coordinate>& odom, vector<coordinate>& out){
    coordinate orgin(0, 0);
    orgin.x = odom[0].x;
    orgin.y = odom[0].y;

    coordinate mapCoor(0, 0);

    vector<float> tf(3, 0);
    if(tfTrans.size() > 0)
        tf = tfTrans.back();
    mapCoor.x = orgin.x - tf[0];
    mapCoor.y = orgin.y - tf[1];

    mapCoor.x = mapCoor.x * cos(tf[2]) + mapCoor.y * sin(tf[2]);
    mapCoor.y = -mapCoor.x * sin(tf[2]) + mapCoor.y * cos(tf[2]);

    printf("UsingTF: (x %f, y %f, theta %f)\n", tf[0], tf[1], tf[2]);

    out.clear();
    out.push_back(mapCoor);
}

int main(int argc, char** argv){
    //Mat image = imread("/home/liaoziwei/ros_project/map/corridor/3L_Full.pgm", 0);
    Mat image = imread(argv[1], 0);


    // image_highlight is used for judging the corridor direction..
    // **** Now, the mechanism is not perfect. Wait to be finished.
    image_highlight = imread( "/home/liaoziwei/Desktop/runSpace/map_highlight.jpg", 0);
    //cvtColor(image_highlight, image_highlight, CV_BGR2GRAY);

//        if (argc > 1){
//        printf("argc: %d, the start point position is x: %d, y: %d\n", argc, *argv[2], *argv[3]);
//    }
//    else
//    {
//        printf("Please input the start point: x y\n");
//        return 0;
//    }

//    if (argc > 1){
//        doornumber_string = argv[2];
//        printf("argc: %d, change the door number to %s\n", argc, doornumber_string);
//    }
//    else
    //{
        printf("The door number: %d\n", DoorNumber);
    //}

    if(image.data == NULL)
    {
        ROS_INFO("error in reading image.");
	return 0;
    }

    cout << "Image Information: COLS:" << image.cols << " ROWS:" << image.rows << endl;

    Mat imageTrans;
    image.copyTo(imageTrans);
    //trans.getMainMap(image, imageTrans);
//    //resize(image, image, Size(image.cols/4, image.rows/4));

//    mapCols = imageTrans.cols;
//    mapRows = imageTrans.rows;

//    cv::imshow(WINDOW_NAME, imageTrans);

//    imageTrans.copyTo(imageSelecting);

//    Mat imageTrans;
//    image.copyTo(imageTrans);
//    //pi的值表示为 v(pi),if  v(seed)-loDiff<v(pi)<v(seed)+upDiff,将pi的值设置为newVal
//    floodFill(imageTrans, startPoint, 128, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);

    imageTrans.copyTo(imageSelecting);

    Mat showImage;
    imageTrans.copyTo(showImage);
    circle(showImage, Point(start_x, start_y), 3,Scalar(0,0,255),-1); //第五个参数我设为-1，表明这是个实点。
    namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    resizeWindow(WINDOW_NAME, 640, 480);

    cv::imshow(WINDOW_NAME, showImage);
    //cv::imshow("highlight pic", image_highlight);


    cv::setMouseCallback(WINDOW_NAME,onMouse,reinterpret_cast<void*> (&imageTrans));

    ROS_INFO("Initialize start searching module...\n");
    ROS_INFO("Connecting with dn_finder node...\n");
    ros::init(argc, argv, "UImap");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);


    /* get a door infor*/
    ros::Subscriber sub = n.subscribe("searchResult", 1, searchResult_deal);
//    while(sub.getNumPublishers() < 1){
//        ROS_INFO("wait for dn_finder node's searchResult topic. Publisher:%d", sub.getNumPublishers());
//        loop_rate.sleep();
//        sub = n.subscribe("searchResult", 1, searchResult_deal);
//    }
//    ROS_INFO("Connecting with dn_finder node...successful.SubNum:%d\n", sub.getNumPublishers());

    command_pub = n.advertise<std_msgs::Float32MultiArray>("searchCommand", 1);
    scalecommand_pub = n.advertise<std_msgs::Float32MultiArray>("command_getscale", 1);
    roompositioncommand_pub = n.advertise<std_msgs::String>("roomNumber", 1);

    // scale sub
    ros::Subscriber sub_scale = n.subscribe("map_scale", 1, map_scale_deal);

    // distance sub
    ros::Subscriber sub_distance = n.subscribe("map_distance_change", 1, map_distance_change_deal);
    ros::Subscriber sub_roomPosition = n.subscribe("roomPosition", 1, subroomPosition);
    ros::Subscriber sub_tf = n.subscribe("tf", 1, subtf);
    ros::Subscriber sub_searchCommand_points = n.subscribe("searchCommand_points", 1, subsearchCommand_points);
    // CONTINUE ON COMMUNICATION

    while(ros::ok()){
        char key = waitKey(30);
        switch (key) {
        case 's':
            if(mode == MODES_SELECTED_AREA)
            {
                if(searchPoints.size() < 1){
                    ROS_INFO("there is no searching point now.");
                    break;
                }
                if(modeHome != HOMEMODES_SELECTED){
                    ROS_INFO("Please select the home point.");
                    break;
                }
                getScale(n);
                // change to the coordinate system on map
                trans.humanMap2SlamMap(searchPoints, mapPoints);
                odom2Map(mapPoints, mapPoints);

                trans.outputVector(mapPoints);

                startSearching_GoAndSearch(argc, argv, n);
            }
            else
                ROS_INFO("please select an area first. \n");
            break;
        case 'g':
            if(modeHome == HOMEMODES_SELECTED)
            {
//                std_msgs::String pubdata;
//                pubdata.data = argv[1]; // how to transform.
//                roompositioncommand_pub.publish(pubdata);
//                _flag_get_destination = false;
//                printf("wait for the destination result.\n");
//                while(_flag_get_destination == false){
//                    waitKey(500);
//                }


                if(destinationPoints.size() < 1){
                    printf("Please select the point first.\n");

                    break;
                }
                printf("Begin getting the scale...\n");

                getScale(n);
                // change to the coordinate system on map
                //vector<coordinate> odomPoints;
                trans.humanMap2SlamMap(destinationPoints, mapPoints);
                //odom2Map(odomPoints, mapPoints);
                trans.outputVector(mapPoints);
                searchPoints.clear();
                searchPoints.push_back(destinationPoint);
                startSearching_GoAndSearch(argc, argv, n);
            }
            else
                ROS_INFO("please select an area first. \n");
            break;
        case 'q':
            return 0;
            break;
        default:
            break;
        }
    }

    return 0;
}
