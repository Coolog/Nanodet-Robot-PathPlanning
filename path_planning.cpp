#include <path_planning.h>

/*********************** DEBUG函数宏定义***********************/
//#define DUBUG_LINE              //输出画的线的位置信息
//#define DEBUG_PRE_MESSAGE       //输出前一帧的信息
//#define DEBUG_NOW_MESSAGE       //输出当前帧的信息
//#define DEBUG_EACH_LINE           //画每一条线

float Path::judge_distance(){
    if(line_max_y[last_pos_index]> 360){
        return 0.35;
    }
    else if(line_max_y[last_pos_index]>240){
        return 0.45;
}
    else if(line_max_y[last_pos_index]>120){
        return 0.6;
    }
    else{
        return 0.7;
    }
}

void Path::set_mode(int mode,int _mage_state){
    mage_num = mode;
    mage_state =_mage_state;
    if(mode != judge_mode_num){
         judge_mode_num = mode;      //设置当前状态下的决策次数
         pre_state = 1;         //表示马哥做完了
//pre_state ++;
//if(pre_state >3)
//    pre_state = 3;
    }
    else{
        pre_state = 0;      //0表示马哥还没做完
    }

}

//初始化参数信息
void Path::init(int _num_direction){
    mode_3_send_angle = 90;
    mode_3_send_distance = 0.3;
    num_direction = _num_direction;
    mage_state = -1;
    cout << "num_direction:" <<  num_direction << endl;
    judge_mode_num = -1;       //电控：开始设置为0，表示不做决策，等待电控需要数据再决策
    pre_state = 1;              //等马哥发送1
    judge_mode = -1;            //初始状态
    state_mode = -1;     //初始化为前进模式



    mode_3_num = 0;     //模式三的个数为0
    send_angle = 0;
    last_distance = 0;
    pre_max_num = -1;
    pre_pos = -1;
    pre_b1 = -1;
    pre_b2 = -1;
    pre_angle = -1;     //开始的先前角度为-1
    last_angle = -1;
    last_pos_index = -1;
    Line_left_1.x = shovel_left;
    Line_left_1.y = img.size().height;
    Line_right_1.x = shovel_right;
    Line_right_1.y = img.size().height;
    last_pos = 0;
    last_b1=0;last_b2=0;
    cout << "left_blind_angleL: " << left_blind_angle << "  ,delta_blind_angle: "  << delta_blind_angle << endl;
    cout << num_direction << endl;
    color = new Scalar[1000];

    number = new int[num_direction + 1];
    line_max_y = new int[num_direction + 1];
    circle_number = new int[ball_pos.size() + 1];
    max_change_angle_num = ceil(360/(delta_blind_angle) - 1);
    cout << "最大旋转次数为" << max_change_angle_num << endl;
  //  srand(time(0));
 cout << 665 << endl;
    for(int i=0;i<num_direction;i++){
        int a=rand()%256,b=rand()%256,c=rand()%256;

        //cout << "第" << i << "条线的颜色为：" << a << "  " << b << "  " << c << endl;
        color[i] = Scalar(a,b,c);
        number[i] = 0;

    }
    for(int i=0;i<1000;i++){
        int a=rand()%256,b=rand()%256,c=rand()%256;

        //cout << "第" << i << "条线的颜色为：" << a << "  " << b << "  " << c << endl;
        color[i] = Scalar(a,b,c);
    }

    number[num_direction] = 0;
    set_circle_distance((shovel_right - shovel_left)/2);
}


////平行线选最终方向法
///*
// * 1.划分为num_direction个方向
// * 2.for循环，构造出当前方向的那一条平行线段
// * 3.对于每一个点，判断其是否在平行线内
// * 4.对于每条线的xita，k=tan(左死角 + （180-死角差）/（n-1）*i)
// *
// *circle_value /

void Path::last_direction(){
    cout << 22222 << endl;
    //init();             //初始化参数
    last_pos_index = -1;
#ifdef DEBUG_PRE_MESSAGE
    DEBUG_MESSAGE_PRE();
#endif
    for(int i=0;i<= num_direction;i++){
        line_max_y[i] = 0;          //初始化y
    }
     cout << "最大旋转次数为" << max_change_angle_num << endl;
    pre_state = 1;
    if(ball_pos.size()>0 &&(pre_state == 1 || judge_mode == 3)){      //有球的情况：  需要做决策 或者 找球状态需要实时检测
        circle_process_ball(1);
//        origin_process_ball(1);        //原始；处理球的思路

    }
    else{           //没球

        if(pre_state == 0 && judge_mode != 3){     //没球+马哥没做完+状态1，2（原子性，此时等待做完）
            if(state_mode == -1){
                cout << "开机状态，等马哥发决策需求！！！" << endl;
            }
            else{
                cout << "状态" << state_mode << " ，等马哥发决策需求！！！" << endl;

            }

            DRAW_TEXT();        //显示图像信息
           imshow("Cool", img);
            return;     //返回结果，此时不用发送    初始开机状态也return
        }
        else{       //没球 + 马哥做完了   或者  没球+第三状态
            state_mode = 2; //原地旋转模式
            if(judge_mode == 2){        //若先前是第二状态，更新旋转次数
                change_rotated_num = judge_mode_num;     //表示马哥做完了之前的动作，此时更新状态次数
            }

            if(change_rotated_num >= (max_change_angle_num) || judge_mode == 3){     //转了360度还是没球 或者先前就是第三状态
                cout << "---------------------------------------------------------if manzu2-->3:" <<( change_rotated_num >= (max_change_angle_num) )<< endl;
                if(judge_mode != 3){

                    state_mode = 3;         //进入随机巡游状态
                }
                if(judge_mode == 3){
                    mode_3_num = judge_mode_num;
                }

                if(mode_3_num > 10){
                    state_mode = 4;//关机模式
                    send_angle = 0;
                    last_distance = 0;
                }
                else{
                    //1-2回头走模式
//                    if(mode_3_num == 1 || mode_3_num == 0){
//                        send_angle = 0;
//                        last_distance = distance_rate;
//                    }
//                    if(mode_3_num == 2){
//                        send_angle = 90;
//                        last_distance = distance_rate;
//                    }
//                    if(mode_3_num == 3){
//                        send_angle = 90;
//                        last_distance =   distance_rate;
//                    }
//                    if(mode_3_num >3){
//                        send_angle = 90;
//                        last_distance = float((mode_3_num-1))/2 * distance_rate;
//                    }
                    if(mode_3_num == 0){
                        send_angle = delta_blind_angle;
                        last_distance = distance_rate;
                        mode_3_send_angle = delta_blind_angle;
                        mode_3_send_distance = distance_rate;
                    }
                    else{
                        send_angle = 45;
                        last_distance = (mode_3_num+2)/2 * distance_rate;
                        mode_3_send_angle = 90;
                        mode_3_send_distance = last_distance;
                    }
                }
            }
            else{
                //原地旋转模式
                cout << "原地旋转模式！！从" << endl;
                    send_angle = 40; //广角角度
                    last_distance = 0;
                    //原地动，不走；


            }
        }
        DRAW_TEXT();        //显示图像信息
         imshow("Cool", img);
    }


    if(pre_state == 1){
        judge_mode = state_mode;        //记录前面一次做的动作的状态
    }
    //pre_state = 0;              //每做完一次决策就自己设置为0，直到马哥下一次发送不同的数值和状态

//    DRAW_TEXT();        //显示图像信息
//   imshow("Cool", img);




#ifdef DEBUG_NOW_MESSAGE
    DEBUG_MESSAGE_NOW();        //输出当前帧决策信息
#endif
}

//储存当前帧变量，用于下一帧的方向损失函数判断
//    pre_pos = last_pos;
//    pre_b1 = last_b1;
//    pre_b2 = last_b2;
//    pre_max_num = max_ball_number;
//    pre_pos_index = last_pos_index;
//    pre_angle = last_angle;

//方向损失函数判断（最终方向决策）
void Path::direction_loss_judge(){
     pre_max_num = number[num_direction];       //前进的球的个数

    //从第二帧开始考虑方向偏移误差
    if(pre_angle != -1){
        cout << "角度差：" << abs(last_angle-90) << "最优解球数： " << max_ball_number <<
                " 更换方向需要的个数差：" << log(abs(last_angle-90) + 1)
                << "  两帧最优解的个数差：" <<
                max_ball_number - pre_max_num << endl;
        DEBUG_MESSAGE_NOW();
       // DEBUG_MESSAGE_PRE();
        //直走更好
        if(0.2*log(abs(last_angle-90) + 1) >= (max_ball_number - number[num_direction])){
            state_mode = 0;
            last_pos = pre_pos;
            last_b1 = pre_b1;
            last_b2 = pre_b2;

            max_ball_number = number[num_direction];
            last_angle = 90;

            last_pos_index = num_direction;
        }
        //拐弯更好
        else{
            state_mode = 1;     //拐弯
            pre_angle = 1;
           // last_angle -= 90;        //发给电控的角度
            if(last_pos< 0){
                 last_angle =  180 + atan(last_pos)*180/pi ;
              }
            else{
                 last_angle =  atan(last_pos)*180/pi ;

            }

        }

//        state_mode = 1;     //拐弯
//       // last_angle -= 90;        //发给电控的角度
//        if(last_pos< 0){
//             last_angle =  180 + atan(last_pos)*180/pi ;
//          }
//        else{
//             last_angle =  atan(last_pos)*180/pi ;

//        }


    }
    else{
        pre_angle = 90;
        state_mode = 1;     //拐弯
    }
}



//更新每一帧的小球信息
void Path::change_ball_message(vector<Point2i> message, Mat src){
    img = src;
    ball_pos = message;
}


void Path::draw_line(Point2i A, Point2i B,string name,Mat img,Scalar _color,int mode){
//    line(img, A,B, Scalar(255,255,0), 3);
    line(img, A,B, _color, mode);
//    imshow("Cool", img);

}


void Path::find_drawline_point(float k,float b1,float b2){
    cout << "k : " << k << "  b1:" << b1 << " b2: " << b2 << endl;
    if(k > 0){
//        cout << "k > 0" << endl;
        if(b1 >= 0 && b1 <=img_height){
//            cout << "b1 >= 0 && b1 <=img_height" << endl;
            draw_left_point = Point2i(0,b1);
//            cout << "draw_left_point: " <<draw_left_point << endl;
        }
        else{
//            cout << "不是b1 >= 0 && b1 <=img_height" << endl;
//            cout << "-b1/k" << -b1/k << endl;
            draw_left_point = Point2i(-b1/k,0);
//            cout << "draw_left_point: " <<draw_left_point << endl;
        }

        if(b2 >= 0 && b2 <=img_height){
//            cout << "b2 >= 0 && b2 <=img_height" << endl;
            draw_right_point = Point2i(0,b2);
//             cout << "draw_right_point: " <<draw_right_point << endl;
        }
        else{
//            cout << "不是b2 >= 0 && b2 <=img_height" << endl;
//            cout << "-k/b2" << -b2/k << endl;
            draw_right_point = Point2i(-b2/k,0);
//             cout << "draw_right_point: " <<draw_right_point << endl;
        }
    }
    else{
//        cout << "k < 0" << endl;
        if((k*img_width + b1) >= 0 && (k*img_width + b1) <=img_height){
            draw_left_point = Point2i(img_width,(k*img_width + b1));
        }
        else{
            draw_left_point = Point2i(-b1/k,0);
        }

        if((k*img_width + b2) >= 0 && (k*img_width + b2) <=img_height){
             draw_right_point = Point2i(img_width,(k*img_width + b2));
        }
        else{
            draw_right_point = Point2i(-b2/k,0);
        }
    }
}

//普通线：判断一个点是否在两条线里面
int Line::contain_normal(Point2i p){
    if((k_left < 0 && p.x < line_shovel_left) || (k_right > 0 && p.x > line_shovel_right)){
        return 0;
    }

    if(k_left > 0){
        if(p.x >= line_shovel_left){
             if((k_right * p.x + b_right < p.y)){return 1;}
             else{return 0;}
        }
        else{
            if((k_left * p.x + b_left > p.y) && (k_right * p.x + b_right < p.y)) {return 1;}
            else{return 0;}
        }
    }
    else{
        if(p.x <= line_shovel_right){
            if((k_left * p.x + b_left < p.y)){return 1;}
            else{return 0;}
        }
        else{
            if((k_left * p.x + b_left < p.y) && (k_right * p.x + b_right > p.y)){return 1;}
            else{return 0;}
        }
    }
}

//垂直线：判断一个点是否在两条线里面
int Line::contain_special(Point2i p){
    if(p.x>b_left && p.x<b_right){
        return 1;
    }
    return 0;
}




//判断有多少个球在两条线里面
/*
 * specicl: 为0则直线正常，为1则直线垂直于x轴
 */

int Line::allnum_contain(vector<Point2i> ball_message, bool special, int &max_y,float &ori_value,int X,int Y,int value_num){
    int sum = 0;
    if(special == 0){
        for(int i=0;i<ball_message.size();i++){
           sum += contain_normal(ball_message[i]);
           //ori_value += 1/pow(sqrt(pow(ball_message[i].x-X,2) + pow(ball_message[i].y-Y,2)),0.9);
           ori_value += y_to_value_num(ball_message[i].y,value_num,Y);
           if(max_y < ball_message[i].y){
               max_y = ball_message[i].y;
           }

        }
    }
    else{
        for(int i=0;i<ball_message.size();i++){
           sum += contain_special(ball_message[i]);
//            ori_value += 1/pow(sqrt(pow(ball_message[i].x-X,2) + pow(ball_message[i].y-Y,2)),0.9);
            ori_value += y_to_value_num(ball_message[i].y,value_num,Y);
           if(max_y < ball_message[i].y){
               max_y = ball_message[i].y;
           }
        }
    }


    return sum;     //返回小球在线里面的总数
}


//double Path::value_contain(vector<Point2i> ball_message){

//}

/************************************DUBUG函数族****************************************/

void Path::DUBUG_LINE_POINT(){
    cout << img.size() << endl;
    cout <<"x: " << shovel_left << "   ,y:" << Line_left_1.y << endl;
    cout << "高 ： " << img_height << " , 宽： " << img_width << endl;
    cout <<"draw_left_pointx: " << draw_left_point.x << "   draw_left_point,y:" << draw_left_point.y << endl;
    cout <<"Line_left_1x: " << Line_left_1.x << "   ,Line_left_1y:" << Line_left_1.y << endl;
    cout << endl;
    cout <<"draw_right_pointx: " << draw_right_point.x << "   ,draw_right_pointy:" << draw_right_point.y << endl;
    cout <<"Line_right_1x: " << Line_right_1.x << "   ,Line_right_1y:" << Line_right_1.y << endl;
}



void Path::DEBUG_MESSAGE_PRE(){
    cout << "上一帧的球的个数：" << pre_max_num << " 上一帧索引： " << pre_pos_index << " 上一帧角度：" << pre_angle << endl;
    cout << "上一帧k:" << pre_pos << "   b1:" << pre_b1 << "   b2:" << pre_b2 << endl;
}

void Path::DEBUG_MESSAGE_NOW(){

    //最终决策出来的方向
    cout << endl;
    if(last_pos< 0){
         cout << "当前决策度数： " << 180 + atan(last_pos)*180/pi << endl;
      }
    else{
        cout << "当前决策度数： " << atan(last_pos)*180/pi << endl;
    }

    cout << "当前帧的球的个数：" << max_ball_number << " 当前帧索引： " << last_pos_index << "  当前帧角度:  " << last_angle << endl;
    cout << "当前帧k:" << last_pos << "   b1:" << last_b1 << "   b2:" << last_b2 << endl;
}

void Path::DEBUG_DRAW_EACH_LINE(int i ,float last_k,float b1,float b2){

    //cout << "当前决策度数： " << atan(last_k)*180/pi << "  ,个数为: " << number[i] << endl;
//    cout <<"draw_left_pointx: " << draw_left_point.x << "   draw_left_point,y:" << draw_left_point.y << endl;
//    cout <<"Line_left_1x: " << Line_left_1.x << "   ,Line_left_1y:" << Line_left_1.y << endl;
//    cout << endl;
//    cout <<"draw_right_pointx: " << draw_right_point.x << "   ,draw_right_pointy:" << draw_right_point.y << endl;
//    cout <<"Line_right_1x: " << Line_right_1.x << "   ,Line_right_1y:" << Line_right_1.y << endl;

    find_drawline_point(last_k,b1,b2);
    if(last_pos*57.3 < 0){
        stringstream st;
        st<< 180 + atan(last_pos)*180/pi  ;
         draw_line(Line_left_1, draw_left_point,"度数为" + st.str(),img,color[i],1);
         draw_line(Line_right_1, draw_right_point,"度数为" + st.str(),img,color[i],1);
    }
    else{
        stringstream st;
        st<<atan(last_pos)*180/pi;
        draw_line(Line_left_1, draw_left_point,"度数为" + st.str(),img,color[i],1);
        draw_line(Line_right_1, draw_right_point,"度数为" + st.str(),img,color[i],1);
    }
}

/************************************END——DUBUG函数族****************************************/


void Path::origin_process_ball(int mode){
    cout << "变成状态1了,开始捡球！！！" << endl;
     change_rotated_num = 0;     //转了0次，对状态2和3的计数器进行归位
     mode_3_num = 0;

     max_ball_number = 0;
     optimal_value = 0;
     ori_value =   new float[num_direction + 1];
     for(int i=0;i<ball_pos.size();i++){
         ori_value[i] = 0;
     }
    // int number[num_direction];
     float last_k,b1,b2;
     Line temp_line(shovel_left,shovel_right,1);
     number[num_direction] = temp_line.allnum_contain(ball_pos,1,line_max_y[num_direction],ori_value[num_direction],(shovel_left+shovel_right)/2,img.size().height,4);
        for(int i=0;i<=num_direction;i++){
         if(i == num_direction){
             break;          //最后一个是特殊情况
         }
         //判断平行线垂直于x轴的情况
         last_k = tan(left_blind_angle + (right_blind_angle - left_blind_angle)*(float(i)/(num_direction-1)));
         b1 = img.size().height - last_k * shovel_left;
         b2 = img.size().height - last_k * shovel_right;

 //        cout << "第" << i << " 条直线， " <<"当前斜率为：" << last_k << " b1 = "<<b1 << " b2= " << b2 << endl;

         Line now_line(last_k,last_k,b1,b2,shovel_left,shovel_right,0);
         number[i] = now_line.allnum_contain(ball_pos,0,line_max_y[i],ori_value[i],(shovel_left+shovel_right)/2,img.size().height,4);     //正常模式

         //单纯靠个数判断，不加入距离价值
         if(mode == 1){
             if(number[i] > max_ball_number){        //记录最大的个数
                 optimal_value = ori_value[i];
                 max_ball_number = number[i];
                 //cout << "当前最大球： " << max_ball_number << endl;
                 last_pos = last_k;
                 last_pos_index = i;
                 last_b1 = b1;
                 last_b2 = b2;
             }
         }
         if(mode == 2){
             if(ori_value[i] > optimal_value){        //记录最大的个数
                 optimal_value = ori_value[i];
                 max_ball_number = number[i];
                 //cout << "当前最大球： " << max_ball_number << endl;
                 last_pos = last_k;
                 last_pos_index = i;
                 last_b1 = b1;
                 last_b2 = b2;
             }
         }


 #ifdef DEBUG_EACH_LINE
         DEBUG_DRAW_EACH_LINE(i,last_k,b1,b2);       //画出每一条线
 #endif
     }



 #ifdef DUBUG_LINE
     DUBUG_LINE_POINT();     //对画线的点进行DEBUG
 #endif


         //得到最终角度
         if(last_pos < 0){
             last_angle = 180 + atan(last_pos)*180/pi;
         }
         else{
             last_angle = atan(last_pos)*180/pi;
         }

         direction_loss_judge();


         //找到最后画的线的点
         if(state_mode == 1){
             find_drawline_point(last_pos,last_b1,last_b2);
             send_angle = (last_angle - 90) * trans_angle_rate;       //拐弯捡球模式
             cout<< "当前决策角度-------------------------" << send_angle << endl;
             last_distance = judge_distance();
         }
         else{ //直走模式
             draw_left_point = Point2i(shovel_left,0);
             draw_right_point = Point2i(shovel_right,0);
             send_angle = 0;
             last_distance = judge_distance();

             cout << "旋转角度为" << send_angle << endl;
         }
         state_mode = 1;
          stringstream st;
          st<< last_angle;           //最终的角度

          draw_line(Line_left_1, draw_left_point,"度数为" + st.str(),img,color[last_pos_index],5);
          draw_line(Line_right_1, draw_right_point,"度数为" + st.str(),img,color[last_pos_index],5);
          DRAW_TEXT();        //显示图像信息
          imshow("Cool", img);
}

//设置圆心距离
void Path::set_circle_distance(float circle_dis){
    circle_distance = circle_dis;   //设置圆心距离
}

void Path::circle_process_ball(int mode){
    cout << "变成状态1了,开始捡球！！！" << endl;
     circle_max_index = 0;
     max_ball_number = 0;
     max_ball_value = 0;
     circle_number = new int[ball_pos.size() + 1];
     circle_value =   new float[ball_pos.size() + 1];
     for(int i=0;i<ball_pos.size();i++){
         circle_number[i] = 0;
         circle_value[i] = 0;
     }
     float temp_k,temp_b1,temp_b2,temp_ridus;
     for(int i=0;i<ball_pos.size();i++){

         temp_k = float(img.size().height - ball_pos[i].y)/(float(shovel_right + shovel_left)/2 - ball_pos[i].x);
         temp_b1 = img.size().height - temp_k * shovel_left;
         temp_b2 = img.size().height - temp_k * shovel_right;
         temp_ridus = (1 * fabs(temp_k * (float(shovel_right + shovel_left)/2) - img.size().height + temp_b1)) / (sqrt(1 + temp_k * temp_k));
         set_circle_distance(temp_ridus);
         for(int j=0;j<ball_pos.size();j++){
             if(i == j){
                 continue;
             }
             else{      //判断距离
                 if(sqrt(pow(ball_pos[i].x-ball_pos[j].x,2) + pow(ball_pos[i].y-ball_pos[j].y,2)) < temp_ridus){
                     circle_number[i]++;
                     circle_value[i] += 1/pow(sqrt(pow(ball_pos[i].x-ball_pos[j].x,2) + pow(ball_pos[i].y-ball_pos[j].y,2)),1.0);
                 }
             }
         }
         if(circle_number[i] > 0){
             circle_value[i]/=circle_number[i];
         }
         else{
             circle_value[i] = 0;
         }
         if(mode == 1){
             if(circle_number[i] > max_ball_number){
                 max_ball_number = circle_number[i] + 1;
                 max_ball_value = circle_value[i];
                 circle_max_index = i;
                 last_pos = float(img.size().height - ball_pos[i].y)/(float(shovel_right + shovel_left)/2 - ball_pos[i].x);
                 last_b1 = img.size().height - last_pos * shovel_left;
                 last_b2 = img.size().height - last_pos * shovel_right;
                 last_circle_distance = circle_distance;
                 find_drawline_point(last_pos,last_b1,last_b2);
             }
         }
         else{
             if(max_ball_value<circle_value[i]){
                 max_ball_number = circle_number[i] + 1;
                 max_ball_value = circle_value[i];
                 circle_max_index = i;
                 last_pos = float(img.size().height - ball_pos[i].y)/(float(shovel_right + shovel_left)/2 - ball_pos[i].x);
                 last_b1 = img.size().height - last_pos * shovel_left;
                 last_b2 = img.size().height - last_pos * shovel_right;
                 last_circle_distance = circle_distance;
                 find_drawline_point(last_pos,last_b1,last_b2);
             }
         }

     }
     if(last_pos < 0){
         last_angle = 180 + atan(last_pos)*180/pi;
     }
     else{
         last_angle = atan(last_pos)*180/pi;
     }
     send_angle = (last_angle - 90) * trans_angle_rate;       //拐弯捡球模式
      cout << 44444 << endl;
     draw_line(Line_left_1, draw_left_point,"度数为" ,img,color[last_pos_index],5);
     draw_line(Line_right_1, draw_right_point,"度数为" ,img,color[last_pos_index],5);
      cout << 55555 << endl;
     circle(img,ball_pos[circle_max_index],last_circle_distance,Scalar(0,255,255),3);
     cout << 66666 << endl;
     state_mode = 1;
     DRAW_CIRCLE_TEXT();        //显示图像信息
     cout << 77777 << endl;
     imshow("Cool", img);
    cout << 88888 << endl;
}


float Line::y_to_value_num(float dis,int num,int height)
{
    for(int i=num-1;i>=1;i--){
        if(dis > float(height) * i/num){
            return (i + 1);
        }
    }
    return 1;

}
void Path::DRAW_CIRCLE_TEXT(){
    string label_angle_text = "JUDGE ANGLE:  " ;
    stringstream angle_text;
    angle_text<< label_angle_text;
    angle_text << int(send_angle) ;
    angle_text << " degrees";

    string label_num = "JUDGE BALL NUMBER:" + format("%d", max_ball_number);
    putText(img, angle_text.str(), Point(10,30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    putText(img, label_num, Point(10,60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    if(state_mode == -1){
        putText(img, "WAIT JUDGE NEED", Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    }
    else if(state_mode == 1){
            putText(img, "PICK UP BALL", Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    }
    else if(state_mode == 2){
        putText(img, "SPIN AROUND,MAX_NUM:" + to_string(max_change_angle_num) + "  NOW_NUM: " + to_string(change_rotated_num), Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
 }
    else if(state_mode == 3){
        putText(img, "FIND BALL: FIND_NUM:" + to_string(mode_3_num), Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
 }
    else if(state_mode == 4){
        putText(img, "GO HOME", Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
 }
}

//显示图像信息
void Path::DRAW_TEXT(){
    //显示图像的文字信息
    string label_angle_text = "JUDGE ANGLE:  " ;
    stringstream angle_text;
    angle_text<< label_angle_text;
    angle_text << int(send_angle) ;
    angle_text << " degrees";

    cout << "最终角度" << last_angle << endl;

   string label_num = "JUDGE BALL NUMBER:" + format("%d", max_ball_number);
   putText(img, angle_text.str(), Point(10,30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
   putText(img, label_num, Point(10,60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
   if(state_mode == -1){
       putText(img, "WAIT JUDGE NEED", Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
   }
   else if(state_mode == 1){
           putText(img, "PICK UP BALL", Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
   }
   else if(state_mode == 2){
       putText(img, "SPIN AROUND,MAX_NUM:" + to_string(max_change_angle_num) + "  NOW_NUM: " + to_string(change_rotated_num), Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
}
   else if(state_mode == 3){
       putText(img, "FIND BALL: FIND_NUM:" + to_string(mode_3_num), Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
}
   else if(state_mode == 4){
       putText(img, "GO HOME", Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
}
    putText(img, "CONTINUE NUM:" + to_string(judge_mode_num), Point(10,120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    putText(img, "SEND DIS:" + to_string(last_distance), Point(10,150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    putText(img, "Mage  STATE:" + to_string(mage_state) + "  NUM:" + to_string(mage_num), Point(10,180), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);

//   stringstream st;
//   st<<number[num_direction];
//   string str= "Straght Ball Num: " + st.str();
//   putText(img, str, Point(10,90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
}
