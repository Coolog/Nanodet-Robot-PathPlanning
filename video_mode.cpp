//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <iostream>

//#define CV_CAP_PROP_FRAME_WIDTH  3

//using namespace std;
//using namespace cv;

//int main()
//{
//    //打开视频文件：其实就是建立一个VideoCapture结构
//    VideoCapture capture;
//    capture.open("/home/nuc/lzh_dachuang/nanodet/nanodet/测试素材/floorVideo1.mp4");

//    //检测是否正常打开:成功打开时，isOpened返回ture
//    if (!capture.isOpened())
//        cout << "fail to open!" << endl;
//    //获取整个帧数
//    long totalFrameNumber = capture.get(cv::CAP_PROP_FRAME_COUNT);
//    cout << "整个视频共" << totalFrameNumber << "帧" << endl;

//    //设置开始帧()
//    long frameToStart = 0;
//    capture.set(cv::CAP_PROP_POS_FRAMES, frameToStart);
//    cout << "从第" << frameToStart << "帧开始读" << endl;

////    //设置结束帧
////    int frameToStop = 400;

////    if (frameToStop < frameToStart)
////    {
////        cout << "结束帧小于开始帧，程序错误，即将退出！" << endl;
////        return -1;
////    }
////    else
////    {
////        cout << "结束帧为：第" << frameToStop << "帧" << endl;
////    }

//    //获取帧率
//    double rate = capture.get(cv::CAP_PROP_FPS);
//    cout << "帧率为:" << rate << endl;

//    //定义一个用来控制读取视频循环结束的变量
//    bool stop = false;
//    //承载每一帧的图像
//    Mat frame;
//    //显示每一帧的窗口
//    namedWindow("Extracted frame");
//    //两帧间的间隔时间:
//    //int delay = 1000/rate;
//    int delay = 1000 / rate;

//    //利用while循环读取帧
//    //currentFrame是在循环体中控制读取到指定的帧后循环结束的变量
//    long currentFrame = frameToStart;



//    while (!stop)
//    {
//        //读取下一帧
//        if (!capture.read(frame))
//        {
//            cout << "读取视频失败" << endl;
//            return -1;
//        }


//        imshow("after filter", frame);
//        cout << "正在读取第" << currentFrame << "帧" << endl;
//        //waitKey(int delay=0)当delay ≤ 0时会永远等待；当delay>0时会等待delay毫秒
//        //当时间结束前没有按键按下时，返回值为-1；否则返回按键

//        int c = waitKey(delay);
//        //按下ESC或者到达指定的结束帧后退出读取视频
//        if ((char)c == 27)
//        {
//            stop = true;
//        }
//        //按下按键后会停留在当前帧，等待下一次按键
//        if (c >= 0)
//        {
//            waitKey(0);
//        }
//        currentFrame++;

//    }
//    //关闭视频文件
//    capture.release();
//    waitKey(0);
//    return 0;
//}
