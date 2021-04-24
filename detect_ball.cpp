#include"detect_ball.h"

NanoDet::NanoDet(int input_shape, float confThreshold, float nmsThreshold)
{
    assert(input_shape==320 || input_shape==416);
    this->input_shape[0] = input_shape;
    this->input_shape[1] = input_shape;
    this->prob_threshold = confThreshold;
    this->iou_threshold = nmsThreshold;

    ifstream ifs(this->classesFile.c_str());
    string line;
    while (getline(ifs, line)) this->classes.push_back(line);
    this->num_class = this->classes.size();
    if(input_shape==320)
    {


         this->net = readNet("/home/nuc/lzh_dachuang/BallDetection_Demo/simplify_second_ball_model.onnx");
       // this->net = readNetFromModelOptimizer("model_second_m_ball_hongjie.xml","model_second_m_ball_hongjie.bin");

    }
    else
    {
      //  this->net = readNetFromModelOptimizer("model_second_m_ball_hongjie.xml","model_second_m_ball_hongjie.bin");

        this->net = readNet("/home/nuc/lzh_dachuang/BallDetection_Demo/simplify_second_ball_model.onnx");
            }
}

Mat NanoDet::resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left)
{
    int srch = srcimg.rows, srcw = srcimg.cols;
    *newh = this->input_shape[0];
    *neww = this->input_shape[1];
    Mat dstimg;
    if (this->keep_ratio && srch != srcw)
    {
        float hw_scale = (float)srch / srcw;
        if (hw_scale > 1)
        {
            *newh = this->input_shape[0];
            *neww = int(this->input_shape[1] / hw_scale);
            resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
            *left = int((this->input_shape[1] - *neww) * 0.5);
            copyMakeBorder(dstimg, dstimg, 0, 0, *left, this->input_shape[1] - *neww - *left, BORDER_CONSTANT, 0);
        }
        else
        {
            *newh = (int)this->input_shape[0] * hw_scale;
            *neww = this->input_shape[1];
            resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
            *top = (int)(this->input_shape[0] - *newh) * 0.5;
            copyMakeBorder(dstimg, dstimg, *top, this->input_shape[0] - *newh - *top, 0, 0, BORDER_CONSTANT, 0);
        }
    }
    else
    {
        resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
    }
    return dstimg;
}

void NanoDet::normalize(Mat& img)
{
    img.convertTo(img, CV_32F);
    int i = 0, j = 0;
    for (i = 0; i < img.rows; i++)
    {
        float* pdata = (float*)(img.data + i * img.step);
        for (j = 0; j < img.cols; j++)
        {
            pdata[0] = (pdata[0] - this->mean[0]) / this->std[0];
            pdata[1] = (pdata[1] - this->mean[1]) / this->std[1];
            pdata[2] = (pdata[2] - this->mean[2]) / this->std[2];
            pdata += 3;
        }

    }
}

vector<Point2i> NanoDet::detect(Mat srcimg)
{
    int newh = 0, neww = 0, top = 0, left = 0;
    Mat dstimg = this->resize_image(srcimg, &newh, &neww, &top, &left);
    this->normalize(dstimg);
    Mat blob = blobFromImage(dstimg);
    this->net.setInput(blob);


    vector<Mat> outs;
    this->net.forward(outs, this->net.getUnconnectedOutLayersNames());

    return this->post_process(outs, srcimg, newh, neww, top, left);
}

void NanoDet::softmax(float* x, int length)
{
    float sum = 0;
    int i = 0;
    for (i = 0; i < length; i++)
    {
        x[i] = exp(x[i]);
        sum += x[i];
    }
    for (i = 0; i < length; i++)
    {
        x[i] /= sum;
    }
}

void NanoDet::generate_proposal(vector<int>& classIds, vector<float>& confidences, vector<Rect>& boxes, const int stride_, Mat out_score, Mat out_box)
{
    const int num_grid_y = (int)this->input_shape[0]/stride_;
    const int num_grid_x = (int)this->input_shape[1]/stride_;
    const int reg_1max = this->reg_max + 1;

    if(out_score.dims==3)
    {
        out_score = out_score.reshape(0, num_grid_x*num_grid_y);
    }
    if(out_box.dims==3)
    {
        out_box = out_box.reshape(0, num_grid_x*num_grid_y);
    }
    for (int i = 0; i < num_grid_y; i++)
    {
        for (int j = 0; j < num_grid_x; j++)
        {
            const int idx = i * num_grid_x + j;
            Mat scores = out_score.row(idx).colRange(0, num_class);
            Point classIdPoint;
            double score;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &score, 0, &classIdPoint);
            if (score >= this->prob_threshold)
            {
                float* pbox = (float*)out_box.data + idx * reg_1max * 4;
                float dis_pred[4];
                for (int k = 0; k < 4; k++)
                {
                    this->softmax(pbox, reg_1max);
                    float dis = 0.f;
                    for (int l = 0; l < reg_1max; l++)
                    {
                        dis += l * pbox[l];
                    }
                    dis_pred[k] = dis * stride_;
                    pbox += reg_1max;
                }

                float pb_cx = (j + 0.5f) * stride_ - 0.5;
                float pb_cy = (i + 0.5f) * stride_ - 0.5;
                float x0 = pb_cx - dis_pred[0];
                float y0 = pb_cy - dis_pred[1];
                float x1 = pb_cx + dis_pred[2];
                float y1 = pb_cy + dis_pred[3];

                classIds.push_back(classIdPoint.x);
                confidences.push_back(score);
                boxes.push_back(Rect((int)x0, (int)y0, (int)(x1 - x0), (int)(y1 - y0)));
            }
        }
    }
}

vector<Point2i> NanoDet::post_process(vector<Mat> outs, Mat& frame, int newh, int neww, int top, int left)
{
    /////generate proposals
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    this->generate_proposal(classIds, confidences, boxes, this->stride[0], outs[0], outs[1]);
    this->generate_proposal(classIds, confidences, boxes, this->stride[1], outs[2], outs[3]);
    this->generate_proposal(classIds, confidences, boxes, this->stride[2], outs[4], outs[5]);

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, this->prob_threshold, this->iou_threshold, indices);
    float ratioh = (float)frame.rows / newh;
    float ratiow = (float)frame.cols / neww;

    ball_position.clear();      //清空上一帧的小球

    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        int xmin = (int)max((box.x - left)*ratiow, 0.f);
        int ymin = (int)max((box.y - top)*ratioh, 0.f);
        int xmax = (int)min((box.x - left + box.width)*ratiow, (float)frame.cols);
        int ymax = (int)min((box.y - top + box.height)*ratioh, (float)frame.rows);



        ball_position.push_back(Point2i((xmin + xmax)/2,(ymin + ymax)/2));    //将其中小球位置存起来

        rectangle(frame, Point(xmin+5, ymin+5), Point(xmax+5, ymax+5), Scalar(162,32,240), 3);

        //Get the label for the class name and its confidence
        string label = format("%.2f", confidences[idx]);
        label = classes[classIds[idx]] + ":" + label;

        //Display the label at the top of the bounding box
        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        ymin = max(ymin, labelSize.height);

        putText(frame, label, Point(xmin, ymin), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,20), 2);


    }
    return ball_position;   //返回球的信息
}


