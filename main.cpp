#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdlib.h>
#include <time.h>

#include<vector>
#include "detect_ball.h"
#include "path_planning.h"
#include "serialport.h"
#include "CRC_Check.h"


SerialPort port("/dev/ttyUSB0");        //串口通信的类
VisionData vdata;                       //创建}传数据的结构体（7个）


int mode = 1;
int buff_four = -1;
int shoot_speed = -1;
int COLOR = 1;    //控制识别颜色  red = 0   |   blue = 1;

#define TESTMODE 1   //0表示图片测试模式，1表示视频测试模式

#define VIDEO_OR_TEST 0   //0:跑视频模式   1：相机模式

#define EACH_FRAME_DEBUG 0    //1：逐帧调试模式：按空格才到下一帧

#define SAVE_VIDEO 2    //0：不保存视频  1：根据时间保存视频 2：既保存原视频又保存有结果的视频

void image_test();
void video_test();
string get_time(){
    time_t t;
       struct tm *tmp;
       char buf2[64];

       /* 获取时间 */
       time(&t);
       tmp = localtime(&t);

       /* 转化时间 */
       if (strftime(buf2, 64, "%r, %a %b %d, %Y", tmp) == 0) {
           printf("buffer length 64 is too small\n");
       } else {
           printf("%s\n", buf2);
       }
       return buf2;
}

int main()
{



    if(TESTMODE == 0){
      image_test();
    }

    if(TESTMODE == 1){
        video_test();
    }



}


void image_test(){
    vector<Point2i> ball_message;
    NanoDet nanonet(320, 0.8, 0.6);

    string imgpath = "/home/nuc/lzh_dachuang/nanodet/nanodet/测试素材/ball3-0.png";
    Mat srcimg = imread(imgpath);   //检测图片

    double t1 = getTickCount();
    ball_message = nanonet.detect(srcimg);
    cout << ball_message.size() << "个球" ;
    double t2 = getTickCount();
    cout<<" : time: "<<(t2 - t1) /getTickFrequency() * 1000 <<"ms"<<endl;

    //更新球信息
    Path Direction_decision(180,srcimg,ball_message,srcimg.size().width,srcimg.size().height,300,600,pi/10,pi*9/10,float(2)/9);
    Direction_decision.init(180);

   Direction_decision.last_direction();   //六个方向的决策

    static const string kWinName = "Deep learning object detection in OpenCV";
    //namedWindow(kWinName, WINDOW_NORMAL);
    imshow(kWinName, srcimg);
    waitKey(0);
    destroyAllWindows();

}
int jilu = 0;

void video_test(){

 VideoWriter writer,writer_result;
    //保存视频
    if(SAVE_VIDEO == 1 || SAVE_VIDEO == 2){
         writer.open("../ORI_" + get_time() + ".avi", VideoWriter::fourcc('M','J','P','G'), 25.0, Size(960, 448));
        if(!writer.isOpened())
         {
         cout<< "Error : fail to open video writer\n"<<endl;
         return ;
         }
    }
    if(SAVE_VIDEO == 2){
         writer_result.open("../SHOW_" + get_time() + ".avi", VideoWriter::fourcc('M','J','P','G'), 25.0, Size(960, 448));
    }

    vector<Point2i> ball_message;
    NanoDet nanonet(320, 0.2, 0.4);

    //视频路径：   yolo-test-video2.mp4  floorVideo1.mp4 yolo-test-video3.mp4
    cv::VideoCapture capture;
    if(VIDEO_OR_TEST == 0){
        //home/nuc/lzh_dachuang/最后测试视频/last_show_luzhi.mp4
        //"/home/nuc/lzh_dachuang/nanodet/nanodet/测试素材/floorVideo1.mp4"
        //home/nuc/lzh_dachuang/最后测试视频/last_show.avi
         capture.open("/home/nuc/lzh_dachuang/nanodet/nanodet/测试素材/floorVideo1.mp4");
    }
    else{
         capture.open(1);       //相机模式

    }
 port.initSerialPort(); //初始化串口


     int frame_num;
     cv::Mat srcimg;

      if (!capture.isOpened())
      {
          std::cout << "Read video Failed !" << std::endl;
          return ;
      }


     if(VIDEO_OR_TEST == 0){
         frame_num = capture.get(cv::CAP_PROP_FRAME_COUNT);     //获取
         std::cout << "total frame number is: " << frame_num << std::endl;
     }





    capture >> srcimg;      //第一帧额外处理

    Path Direction_decision(180,srcimg,ball_message,srcimg.size().width,srcimg.size().height,200,440,pi/10,pi*9/10,float(2)/9);
    Direction_decision.init(180);

    int i=0;

    bool mode_shi;

   while(1)
     {



        //按下ESC或者到达指定的结束帧后退出读取视频

        if(VIDEO_OR_TEST == 0){
            if(i == frame_num - 1){
                break;
            }
            if(i>0 ){
                if ( cv::waitKey(30) == 'q'  || i == 5180)
                {
                    break;
                }
            }

        }
        else{
            mode_shi = port.get_Mode(mode,buff_four,shoot_speed,COLOR); //每一帧都不断获取电控的数据
            cout << "串口测试：" << mode << "   " << buff_four << "   " << shoot_speed << "   " << COLOR << endl;
        }
 cout << "666666677777888 " << endl ;
        if(i>0){
             cout << "01010101 " << endl ;
         capture >> srcimg;
          cout << "2323232323232 " << endl ;
        }
        i++;

        if(i <0){
            cout << i << endl;
            continue;
        }
        if(SAVE_VIDEO == 1 || SAVE_VIDEO == 2){     //保存原视频模式
              writer << srcimg;
        }
        cout << "2323232323232 " << endl ;
         cout << "图片的高 ： " << srcimg.size().height << "   图片的宽 ： " << srcimg.size().width <<  endl;
         //capture.read(frame); 第二种方式
         double t1 = getTickCount();
          cout << "55555555555 " << endl ;
         ball_message = nanonet.detect(srcimg);
         cout << "1010101010101 " << endl ;
         cout << "第 " <<  i << " 帧有 " << ball_message.size() << "个球" ;
         double t2 = getTickCount();
         cout<<" : time: "<<(t2 - t1) /getTickFrequency() * 1000 <<"ms"<<endl;

         //更新球信息



        Direction_decision.change_ball_message(ball_message,srcimg);
        Direction_decision.last_direction();   //六个方向的决策

        static const string kWinName = "Deep learning object detection in OpenCV";

       //namedWindow(kWinName, WINDOW_NORMAL);
       imshow(kWinName, srcimg);


       if(SAVE_VIDEO == 1 || SAVE_VIDEO == 2){     //保存原视频模式
             writer_result << srcimg;
       }


       if(EACH_FRAME_DEBUG == 1){   //逐帧调试模式
         while(1){
             waitKey(0);
             break;
         }
       }


       //每一帧处理完发数据给电控   加上判断条件（如果电控告诉这边已经走完了，需要下一次的决策数据了，才进行发送）
       vdata = {0,0,0,0,0,0,0,0,0};
       port.TransformData(vdata);
       port.send();

     }
    // waitKey(0);
     cv::destroyWindow("video test");
     capture.release();

     if(SAVE_VIDEO == 1){
         writer.release();      //保存视频
     }

}



