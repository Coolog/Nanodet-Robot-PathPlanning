
DATA_DIR = "E:/大三上/大创/视频素材/ball_video_1.mp4" #视频数据主目录
SAVE_DIR = "E:/大三上/大创/第二次训练图片" #帧文件保存目录

#*************************************************************
#*********************视频转图片*******************************
#*************************************************************
import cv2  #OpenCV库
import os 

def getphoto(video_in, video_save):
    number = 80
    cap = cv2.VideoCapture(video_in)  # 打开视频文件
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # 视频的帧数
    fps = cap.get(cv2.CAP_PROP_FPS)  # 视频的帧率
    dur = n_frames / fps  # 视频的时间
    num_frame = 0
    judge = cap.isOpened()
    while judge:
        flag, frame = cap.read()  # flag是读取状态，frame下一帧
        if cv2.waitKey(0) == 27:
            break
        if flag:
            num_frame += 1
            if num_frame % 10 == 0: 
                print("正在保存第%d张照片" % number)
                cv2.imwrite('./save/' + str(number) + '.jpg', frame)  # cv2.imwrite(‘路径’ + ‘名字’ + ‘后缀’， 要存的帧)
                number += 1
        else:
            break

    print("视频时长: %d 秒" % dur)
    print("视频共有帧数: %d 保存帧数为: %d" % (n_frames, number))
    print("每秒的帧数(FPS): %.1lf" % fps)
def main_1(path):
    video_in = path
    video_save = '2th'
    getphoto(video_in, video_save)
    
if __name__=='__main__':
     paht='./视频素材/ball_video_2.mp4'#视频路径
     main_1(paht)
