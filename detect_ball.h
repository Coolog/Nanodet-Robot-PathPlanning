#ifndef DETECT_BALL_H
#define DETECT_BALL_H

#endif // DETECT_BALL_H

#include<opencv2/opencv.hpp>
#include<iostream>
#include <fstream>
#include <assert.h>

#include<vector>


using namespace cv;
using namespace dnn;
using namespace std;

class NanoDet
{
    public:
        NanoDet(int input_shape, float confThreshold, float nmsThreshold);
        vector<Point2i> detect(Mat srcimg);

    private:
        const int stride[3] = { 8, 16, 32 };
        const string classesFile = "/home/nuc/lzh_dachuang/coco.names";
        int input_shape[2];   //// height, width
        const float mean[3] = { 103.53, 116.28, 123.675 };
        const float std[3] = { 57.375, 57.12, 58.395 };
        const int reg_max = 7;
        float prob_threshold;
        float iou_threshold;
        vector<string> classes;
        int num_class;
        Net net;

        Mat resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left);
        void normalize(Mat& srcimg);
        void softmax(float* x, int length);
        vector<Point2i> post_process(vector<Mat> outs, Mat& frame, int newh, int neww, int top, int left);
        void generate_proposal(vector<int>& classIds, vector<float>& confidences, vector<Rect>& boxes, const int stride_, Mat out_score, Mat out_box);
        const bool keep_ratio = true;

        vector<Point2i> ball_position;

};
