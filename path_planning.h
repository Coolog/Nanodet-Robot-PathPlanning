#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#endif // PATH_PLANNING_H
#include<opencv2/opencv.hpp>
#include<iostream>
#include <fstream>
#include <assert.h>
#include<math.h>
#include<vector>
#include<sstream>
using namespace cv;
using namespace std;


#define pi 3.1415926

class Line{
    public:
        bool mode;                     //是否是特殊模式 0：普通，1：垂直线
        //正常模式
        Line(){}
        Line(float _k_left,float _k_right,float _b_left,float _b_right,
             int _line_shovel_left,int _line_shovel_right, bool _mode){
            mode = _mode;
            k_left = _k_left;
            k_right = _k_right;
            b_left = _b_left;
            b_right = _b_right;
            line_shovel_left = _line_shovel_left;
            line_shovel_right = _line_shovel_right;
        }

        //垂直模式
        Line(float _b_left,float _b_right,bool _mode){
            mode = _mode;
            b_left = _b_left;
            b_right = _b_right;
        }
    int allnum_contain(vector<Point2i> ball_message, bool special, int &max_y, float &_ori_value, int X, int Y, int value_num);  //判断有多少个球在两条线里面
    double value_contain(vector<Point2i> ball_message);             //距离价值计算函数
     float y_to_value_num(float dis, int num, int height);                     //距离转换价值函数

private:
        vector<Point2i> line_ball_message;                  //球的信息
        float k_left,k_right,b_left,b_right;                //两条线的信息
        int contain_normal(Point2i p);                     //普通线判断一个点是否在两条线里面
        int contain_special(Point2i p);                    //垂直线的判断

        int line_shovel_left,line_shovel_right;

};

class Path{
public:
    /*******************************电控联调数据*******************************/
    int state_mode;             //状态模式：0：前进   1：拐弯   2：巡游
    int judge_mode_num,judge_mode;          //上一次的模式和数值
    int last_pos_index,max_ball_number,last_angle;//当前帧方向索引，数目  ，last_angle:告诉伟明要走的方向
    float last_distance;        //告诉伟明最后走的距离
    int send_angle;
    int change_rotated_num,max_change_angle_num;            //已经旋转了几次，最大旋转几次
    int mode_3_num,max_mode_3_num;              //随机走的次数
    int pre_state;              //先前状态，如果没更新数值，则表示马哥还没做完
    float distance_rate;        //距离系数
    int *line_max_y;
    int mage_state,mage_num;
    float trans_angle_rate;
    int mode_3_send_angle;
    float mode_3_send_distance;
    Path(){}
    Path(int _num_direction,Mat & _img,vector<Point2i> _ball_pos,int _img_width,int _img_height,
         int _shovel_left, int _shovel_right,float _left_blind_angle,float _right_blind_angle,float _trans_angle_rate){
        img = _img;
        ball_pos = _ball_pos;
        img_width = _img_width;
        img_height = _img_height;
        shovel_left  = _shovel_left;
        shovel_right = _shovel_right;
        left_blind_angle = _left_blind_angle;
        right_blind_angle = _right_blind_angle;

        //cout << "_num:  " << _num_direction << endl;
        num_direction = _num_direction;
        trans_angle_rate = _trans_angle_rate;       //角度转换比率
        delta_blind_angle =40 ;
        distance_rate = 0.3;
    }

    /*******************电控联调函数族************************/
    void set_mode(int mode, int mage_state);
  float judge_distance();     //返回最后距离
   void last_direction();                   //平行线决策法选出来的最终方向
   void change_ball_message(vector<Point2i> message,Mat src);        //每一帧更新球信息
   void draw_line(Point2i A, Point2i B, string name, Mat img, Scalar _color, int mode);//画线函数

   void init(int _num_direction);                                                //初始化变量函数

   void origin_process_ball(int mode);                  //原始处理球的思路

   void circle_process_ball(int mode);                  //原始处理球的思路
   int *circle_number;
   float *circle_value,max_ball_value;                         //当前方向球的价值
   float circle_distance,last_circle_distance;                       //设置的圆心距离是多少
   void set_circle_distance(float circle_dis);                  //设置圆心距离

   int circle_max_index;

 private:

    vector<Point2i> ball_pos;                                //小球的信息
    Mat img;                                                 //图片
    float left_blind_angle,right_blind_angle,delta_blind_angle; //左右盲角（需要测量）
    int img_width,img_height;                                //图像宽度和高度
    int shovel_left,shovel_right;                            //铲子的左边和右边的坐标位置





    float last_pos,last_b1,last_b2;   //最终决斜率和截距

    Point2i draw_left_point,draw_right_point;             //画右边的点
    void find_drawline_point(float k, float b1, float b2);                         //找到画线的点
    Point2i Line_left_1,Line_right_1;                   //画线的下端点
    int *number;                                             //每一条线的球的数目
    Scalar *color;                                      //线的颜色
    int num_direction;                                  //线的条数

    /**********距离价值函数**********/
    float optimal_value;                                //最优解
    float *ori_value;

    /*
     * pre_pos:上一帧斜率
     * pre_pos_index:上一帧最终方向索引
     * pre_max_num：上一帧最大小球数目
     * pre_b1，pre_b2：上一帧两条线的截距
     */
    int pre_pos_index,pre_max_num,pre_angle;
    float pre_pos,pre_b1,pre_b2;


    //方向改变损失函数
    void direction_loss_judge();        //方向损失函数


    /**********************************图像显示函数族**********************************/
    void DRAW_TEXT();                   //显示每帧的信息
    void DRAW_CIRCLE_TEXT();                   //显示每帧的信息

    /**********************************DEBUG函数族************************************/
    void DUBUG_LINE_POINT();            //输出画的两条线的位置信息
    void DEBUG_MESSAGE_PRE();           //输出前一帧的信息
    void DEBUG_MESSAGE_NOW();           //输出当前帧的信息
    void DEBUG_DRAW_EACH_LINE(int i , float last_k, float b1, float b2);        //画每一条线

};
