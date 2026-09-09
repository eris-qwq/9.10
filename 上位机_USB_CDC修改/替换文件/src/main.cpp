#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


#include "apps.h"
#include "VCOM.h"
#include "QRCore.h"


constexpr bool is_open_qr = true;        // 是否开启二维码扫描
constexpr bool is_open_port = true;     // 是否打开串口
constexpr bool is_open_debug = true;    // 是否开启调试窗口（图像弹窗）

constexpr int x_center_offset = 25;    // X坐标偏移，校准摄像头机械安装偏差
constexpr int y_center_offset = 50;    // Y坐标偏移
constexpr int delta_max = 1000;        // 坐标突变过滤阈值，防跳变

std::string align_port = "ttyACM15";    // MC02 Type-C 原生 USB CDC，由 udev 规则固定命名
std::string screen_port = "ttyUSB16";   // 屏幕串口

cv::VideoCapture QRCoreCap("/dev/df100_cam", cv::CAP_V4L2); //二维码相机
cv::VideoCapture CrawlCap("/dev/h65_cam", cv::CAP_V4L2);    //物料识别相机
// 1. `cv::`：OpenCV 库的命名空间，所有 OpenCV 对象前面都带`cv::`
// 2. `VideoCapture`：OpenCV 视频捕获类，负责读取摄像头 / 视频流
// 3. `QRCoreCap`：对象名字（自定义，这里代表二维码相机）
// 4. `"/dev/df100_cam"`：Linux 设备节点路径，软链接指向真实摄像头（也可以写`/dev/video0`）
// 5. `cv::CAP_V4L2`：**强制使用 V4L2 驱动**（Linux 专用，非常关键，不写会自动选别的后端，经常打不开、帧率异常）

//==================== 相机参数 编译期常量 ====================
// constexpr：编译阶段就确定数值，运行时不可修改，减少运行时计算，适合竞赛写死配置
// 二维码识别相机：分辨率、帧率
constexpr int qr_core_cap_width  = 1280;   // 二维码相机 图像宽度(像素)
constexpr int qr_core_cap_height = 960;    // 二维码相机 图像高度(像素)
constexpr int qr_core_cap_fps    = 30;     // 二维码相机 目标帧率

// 对位/目标识别相机：分辨率、帧率
constexpr int align_cap_width  = 1280;     // 对位相机 图像宽度(像素)
constexpr int align_cap_height = 960;      // 对位相机 图像高度(像素)
constexpr int align_cap_fps    = 30;       // 对位相机 目标帧率

//==================== HSV颜色阈值定义 ====================
// cv::Scalar：OpenCV容器，此处存 H S V 三个通道数值
// OpenCV HSV范围： H:0~179   S:0~255   V:0~255
// _h 后缀 high：阈值上限；_l 后缀 low：阈值下限
// cv::inRange(输入hsv图,下限,上限,输出掩码) 会保留在此区间内的颜色

// --------普通物体颜色阈值（分拣物体：红、蓝、绿）--------
cv::Scalar obj_red_h   {11,  255, 255};  // 物体红色 HSV高阈值
cv::Scalar obj_red_l   {0,   83,  100};  // 物体红色 HSV低阈值

cv::Scalar obj_blue_h  {138, 255, 245};  // 物体蓝色 HSV高阈值
cv::Scalar obj_blue_l  {95,  90,  100};  // 物体蓝色 HSV低阈值

cv::Scalar obj_green_h {91,  255, 255};  // 物体绿色 HSV高阈值
cv::Scalar obj_green_l {55,  82,  48};   // 物体绿色 HSV低阈值

// --------圆形标记色块阈值（场地圆形标识、标记点：红、蓝、绿）--------
cv::Scalar circle_red_h   {10,  255, 255}; // 圆形标记红色 HSV高阈值
cv::Scalar circle_red_l   {0,   9,   160}; // 圆形标记红色 HSV低阈值

cv::Scalar circle_blue_h  {110, 255, 255}; // 圆形标记蓝色 HSV高阈值
cv::Scalar circle_blue_l  {96,  40,  120}; // 圆形标记蓝色 HSV低阈值

cv::Scalar circle_green_h {90,  255, 255}; // 圆形标记绿色 HSV高阈值
cv::Scalar circle_green_l {40,  30,  100}; // 圆形标记绿色 HSV低阈值


int screen_port_fd;
VCOM::VCOM com; // 串口
VCOM::VCOM screen_com; // 屏幕串口
cv::Mat cap_img;

struct AlignmentData {
    int16_t x;
    int16_t y;
};

struct ReceiveData {    // 全为0无效为扫描二维码
    uint8_t color;      // 0无效, 1 R, 2 G, 3 B
    uint8_t align_type; // 0无效, 1物料, 2色环
};

struct QRcoreData {
    uint8_t first[3]{0};
    uint8_t second[3]{0};
};

enum COLOR{
    FALTE_COLOR = 0,//false无效
    RED_COLOR = 1,
    GREEN_COLOR = 2,
    BLUE_COLOR = 3,
};

enum ALIGNTYPE{
    FALTE = 0,//false无效
    OBJECT = 1,//object普通物体
    CIRCLE = 2,//circle圆形目标
};

QRcoreData order;
std::string order_string = "";


//二维码扫码函数
void findQRCore(TrackbarWindow &qr_window, bool is_debug) {

    QRCoreCap.read(cap_img);
    if (cap_img.empty()) return;

    cv::Mat gray_img;
    cv::cvtColor(cap_img, gray_img, cv::COLOR_BGR2GRAY);
    std::vector<std::string> QRCores;
    scanQRCode(gray_img, QRCores);

    if (is_debug) qr_window.imgShow(cap_img);

    for (const std::string &qr_data : QRCores) {

        if (qr_data.size() != 7) continue; // e.g. 123+213
        if (qr_data.at(3) != '+') continue;

        std::cout << "扫描结果 : " << qr_data << std::endl;
        order_string = qr_data;
        for (int i = 0; i < 7; i++) {
            if (i < 3) {
                if (qr_data[i] > '3' || qr_data[i] < '0') continue;
                order.first[i] = qr_data[i] - '0';
            }
            if (i > 3) {
                if (qr_data[i] > '3' || qr_data[i] < '0') continue;
                order.second[i - 4] = qr_data[i] - '0';
            }
        }
        com.transmit<QRcoreData>(order, 1, 0, true);
        return;
    }
}

//OpenCV 颜色色块跟踪函数
void crawlObj(ColorRange &obj_red, ColorRange &obj_green, ColorRange &obj_blue,
                            TrackbarWindow &obj_window, COLOR color, bool is_debug) {
    // static静态局部变量：函数多次调用，值不会清零，保存上一帧的目标坐标，用来防抖
    static int last_x = 0, last_y = 0;

    // 读摄像头一帧图像，CrawlCap是VideoCapture摄像头句柄
    CrawlCap.read(cap_img);
    // 读帧失败直接退出
    if (cap_img.empty()) return;

    std::vector<cv::RotatedRect> obj(3);
    // 5×5高斯模糊，降噪，减少画面噪点产生的小色块干扰
    cv::blur(cap_img, cap_img, cv::Size(5, 5));

    cv::Mat red_ranged_img, green_ranged_img, blue_ranged_img;
    // getObj()：自定义类方法，做HSV颜色阈值、形态学操作、轮廓检测，返回色块的旋转外接矩形RotatedRect
    obj[0] = obj_red.getObj(cap_img, red_ranged_img, is_debug);   // 红色色块
    obj[1] = obj_green.getObj(cap_img, green_ranged_img, is_debug); // 绿色色块
    obj[2] = obj_blue.getObj(cap_img, blue_ranged_img, is_debug);   // 蓝色色块

    cv::RotatedRect aim_rect;
    // 根据传入color参数，选择我们要跟踪哪一个颜色
    if (color == RED_COLOR) {
        aim_rect = obj.at(0);
    } else if (color == GREEN_COLOR) {
        aim_rect = obj.at(1);
    } else if (color == BLUE_COLOR) {
        aim_rect = obj.at(2);
    }

    AlignmentData aim;
    // 判断：找到了有效色块（中心点不为0）
    if (aim_rect.center.x != 0 && aim_rect.center.y != 0) {
        // =========计算偏移量=========
        // 图像坐标系转“中心为原点”的坐标系：画面中心点(align_cap_width/2 , align_cap_height/2)
        // aim.x = 色块x − 画面半宽 +偏移补偿；向左为负，向右为正
        aim.x = aim_rect.center.x - align_cap_width / 2 + x_center_offset;
        // OpenCV图像Y轴向下，所以做翻转：向上为正，向下为负
        aim.y = align_cap_height / 2 - aim_rect.center.y + y_center_offset;

        // ----------------【防抖逻辑】----------------
        // delta_max是阈值：如果本帧坐标和上一帧坐标跳变太大，判定为噪点，直接丢弃，沿用旧坐标last_x、last_y
        if (std::abs(last_x - aim.x) > delta_max || std::abs(last_y - aim.y) > delta_max) {
            aim.x = last_x;
            aim.y = last_y;
        } else {
            // 变化正常，更新保存当前坐标
            last_x = aim.x;
            last_y = aim.y;
        }
        // 串口发送AlignmentData结构体给STM32，下发偏移，用来PID闭环对准色块
        com.transmit<AlignmentData>(aim, 0, 0);
        std::cout << "Aim" << color << ": " << aim.x << ", " << aim.y << std::endl;
    } else {
        // 没有检测到色块：发送 {0,0} 坐标，额外发一条空标记包，通知下位机“目标丢失”
        com.transmit<AlignmentData>({0, 0}, 0, 0);
        com.transmit<void*>(nullptr, 0, 1);
    }
}

// 函数名拼写笔误：应该是 `alignCircle`，作用：**检测画面里 3 个成一条直线的圆形标记，拟合出组合中心点，输出相对于画面中心的偏移坐标，串口下发给 STM32 做对准闭环**。
// 电赛场景：场上有三个圆孔 / 圆形靶标，三点近似排成一条直线，机械臂要对准这三个圆的**合成中心**。
void alginCircle(bool is_debug) {
    // static保存上一帧有效坐标，用于防抖
    static int last_x = 0, last_y = 0;
    std::vector<cv::Point> centers;

    // 读取摄像头帧
    CrawlCap.read(cap_img);
    if (cap_img.empty()) return;

    // ==========【图像预处理】整套增强，专门提升圆边缘效果 ==========
    cv::Mat eroded;
    // 腐蚀：把小噪点、细小毛刺消掉
    cv::erode(cap_img, eroded, cv::Mat(), cv::Point(-1, -1), 2);

    cv::Mat dilated;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    // 膨胀：把物体边缘缺口补全
    cv::dilate(eroded, dilated, kernel, cv::Point(-1, -1), 1);

    cv::Mat gray;
    cv::cvtColor(dilated, gray, cv::COLOR_BGR2GRAY);

    // CLAHE自适应直方图均衡：光照不均匀的时候，增强图像局部对比度
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(5.0, cv::Size(8, 8));
    cv::Mat clahed;
    clahe->apply(gray, clahed);

    // MORPH_GRADIENT 形态学梯度 = 膨胀‑腐蚀，专门提取物体边缘
    cv::Mat morphGrad;
    cv::Mat gradKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(clahed, morphGrad, cv::MORPH_GRADIENT, gradKernel);

    cv::Mat blurred, enhanced;
    cv::GaussianBlur(morphGrad, blurred, cv::Size(7, 7), 3);
    // convertScaleAbs：放大边缘对比度，4倍增益，把边缘变亮
    cv::convertScaleAbs(blurred, enhanced, 4.0, 0);
    cv::GaussianBlur(enhanced, enhanced, cv::Size(7, 7), 3);

    // 二值化，边缘变成纯白，背景黑色
    cv::Mat thresh;
    cv::threshold(enhanced, thresh, 50, 255, cv::THRESH_BINARY);
    cv::GaussianBlur(thresh, thresh, cv::Size(9, 9), 3);

    // ==========【霍夫圆检测 HoughCircles】找出画面全部圆形 ==========
    std::vector<cv::Vec3f> circles;
    // HOUGH_GRADIENT_ALT新版本霍夫圆，更抗噪声
    // 参数：最小半径35，最大半径75，只检测这个尺寸范围的圆，过滤太小太大的干扰圆
    cv::HoughCircles(thresh, circles, cv::HOUGH_GRADIENT_ALT, 1.5, 20, 50, 0.85, 35, 75);

    AlignmentData circle = {0, 0};

    // is_debug开启：把检测到的全部圆画在图像上
    if (is_debug) {
        for (const auto& c : circles) {
            cv::Point center(cvRound(c[0]), cvRound(c[1]));
            int radius = cvRound(c[2]);
            cv::circle(cap_img, center, radius, cv::Scalar(100, 255, 100), 1);
            cv::circle(cap_img, center, 2, cv::Scalar(255, 100, 100), -1);
            // 在图上打印圆心坐标、半径
            std::ostringstream label;
            label << "(" << center.x << "," << center.y << ") r=" << radius;
            cv::putText(cap_img, label.str(), center + cv::Point(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        }
    }

    // 必须至少检测出 >=3 个圆，才进入逻辑，我们要找3个一组的标记
    if (circles.size() >= 3) {
        float best_score = 1e9;
        cv::Point2f best_center;
        std::vector<cv::Point> best_group;

        // 三重循环：枚举所有3个圆的组合 C(n,3)
        for (size_t i = 0; i < circles.size(); ++i) {
            for (size_t j = i + 1; j < circles.size(); ++j) {
                for (size_t k = j + 1; k < circles.size(); ++k) {
                    // 取出这一组的3个圆心、3个半径
                    std::vector<cv::Point> centers = {
                            cv::Point(cvRound(circles[i][0]), cvRound(circles[i][1])),
                            cv::Point(cvRound(circles[j][0]), cvRound(circles[j][1])),
                            cv::Point(cvRound(circles[k][0]), cvRound(circles[k][1]))
                    };
                    std::vector<float> radii = { circles[i][2], circles[j][2], circles[k][2] };

                    // 条件1：三个圆半径不能差太大，过滤大小悬殊的圆组合
                    float r_min = *std::min_element(radii.begin(), radii.end());
                    float r_max = *std::max_element(radii.begin(), radii.end());
                    if (r_max - r_min > 15.0f) continue;

                    // fitLine：对3个圆心做直线拟合，判断三点是否近似共线
                    cv::Vec4f line;
                    cv::fitLine(centers, line, cv::DIST_L2, 0, 0.01, 0.01);
                    float vx = line[0], vy = line[1]; // 直线方向向量
                    float x0 = line[2], y0 = line[3]; // 直线上一点

                    // 计算三个点到拟合直线的最大垂直距离
                    float max_dist = 0.0;
                    for (const auto& pt : centers) {
                        float dx = pt.x - x0;
                        float dy = pt.y - y0;
                        float dist = std::abs(vx * dy - vy * dx);
                        max_dist = std::max(max_dist, dist);
                    }
                    // 条件2：三点不能偏离直线太远，超过15像素直接舍弃这组
                    if (max_dist > 15.0) continue;

                    // 打分：分数越小，这一组越理想（共线好 + 半径接近）
                    float score = max_dist + (r_max - r_min);
                    if (score < best_score) {
                        best_score = score;
                        best_group = centers;

                        // 把三个圆心投影到拟合的直线上，求投影点的平均，得到最终合成目标中心点 best_center
                        std::vector<cv::Point2f> projections;
                        for (const auto& pt : centers) {
                            float t = (pt.x - x0) * vx + (pt.y - y0) * vy;
                            projections.emplace_back(x0 + t * vx, y0 + t * vy);
                        }
                        best_center = std::accumulate(projections.begin(), projections.end(), cv::Point2f(0, 0)) * (1.0f / 3.0f);
                    }
                }
            }
        }

        // 找到了符合条件的三点圆组
        if (!best_group.empty()) {
            // 像素坐标 → 相对画面中心偏移坐标系，和crawlObj完全一样
            circle.x = best_center.x - align_cap_width / 2 + x_center_offset;
            circle.y = align_cap_height / 2 - best_center.y + y_center_offset;

            // 和色块函数一模一样的跳变防抖：坐标跳太大就用上一帧
            if (std::abs(last_x - circle.x) > delta_max || std::abs(last_y - circle.y) > delta_max) {
                circle.x = last_x;
                circle.y = last_y;
            } else {
                last_x = circle.x;
                last_y = circle.y;
            }

            // debug绘制：蓝色小圆标记3个有效圆心，黄色实心圆标记合成目标点
            if (is_debug) {
                for (const auto& pt : best_group)
                    cv::circle(cap_img, pt, 3, cv::Scalar(255, 0, 0), 2);
                cv::circle(cap_img, best_center, 4, cv::Scalar(0, 255, 255), -1);
            }

            // 串口发送偏移结构体 AlignmentData，STM32PID直接复用
            com.transmit<AlignmentData>(circle, 0, 0);
            std::cout << "Circle (fitted): " << circle.x << ", " << circle.y << std::endl;
        }
    } else {
        // 找不到满足条件的3圆组合，发送0,0，外加标记包通知下位机目标丢失
        com.transmit<AlignmentData>({0, 0}, 0, 0);
        com.transmit<void*>(nullptr, 0, 1);
    }

    // 调试弹窗显示中间图，❗❗MaixCam2上运行必须删掉 imshow，板子没有X11，会崩溃
    if (is_debug) {
        cv::imshow("gray", gray);
        cv::imshow("thresh", thresh);
        cv::imshow("result", cap_img);
    }
}


//控制屏幕（向 **串口屏（TFT 液晶、USART HMI 淘晶驰串口屏）** 发送指令：）
void control_screen() {
    // 拼接命令字符串 main.t0.txt="二维码原始字符串"
    std::string cmd = "main.t0.txt=\"" + order_string + "\"";
    // string → uint8_t字节数组，把字符串转成发送的字节流
    std::vector<uint8_t> data(cmd.begin(), cmd.end());
    // 追加3字节 0xFF 作为帧结束标记
    data.push_back(0xFF);
    data.push_back(0xFF);
    data.push_back(0xFF);

    // 通过串口把整包发给显示屏
    screen_com.transmit(data);
}

int main() {
    std::cout << "视觉程序启动！" << std::endl;

    // --------------------------
    // 1、配置摄像头参数：MJPG格式、分辨率、帧率
    // --------------------------
    // 二维码摄像头参数设置
    QRCoreCap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    QRCoreCap.set(cv::CAP_PROP_FRAME_WIDTH, qr_core_cap_width);
    QRCoreCap.set(cv::CAP_PROP_FRAME_HEIGHT, qr_core_cap_height);
    QRCoreCap.set(cv::CAP_PROP_FPS, qr_core_cap_fps);

    // 对准任务摄像头（色块、三点圆检测）
    CrawlCap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    CrawlCap.set(cv::CAP_PROP_FRAME_WIDTH, align_cap_width);
    CrawlCap.set(cv::CAP_PROP_FRAME_HEIGHT, align_cap_height);
    CrawlCap.set(cv::CAP_PROP_FPS, align_cap_fps);

    // --------------------------
    // 2、打开两个串口
    // com：和STM32通信；screen_com：接淘晶驰串口屏，波特率9600
    // --------------------------
    if (is_open_port) {
        // 打开和STM32的串口，115200，8N1
        com.openDevice(align_port, {-1, 115200, 8, 1,  'S'});
        com.start(300);
        com.setDataBufferSize(0, 0, 1);

        // 打开串口屏串口，9600，8N1
        screen_com.openDevice(screen_port, {-1, 9600, 8, 1,  'S'});
        screen_com.start(300);
    }

    // --------------------------
    // 3、PC端调试用：滑动条窗口，调HSV阈值
    // 【重要】MaixCam2板子上要全部删掉 TrackbarWindow！板子没有GUI窗口系统
    // --------------------------
    TrackbarWindow qr_window("qr_scan");

    TrackbarWindow obj_window("OBJ");
    ColorRange obj_red("OBJ_RED", obj_red_h, obj_red_l);
    obj_red.addColorRange(160, 56, 53, 180, 255, 255);
    obj_red.setED(2, 4, 0, 2, 3, 2);

    ColorRange obj_green("OBJ_GREEN", obj_green_h, obj_green_l);
    obj_green.setED(2, 4, 0, 2, 3, 2);

    ColorRange obj_blue("OBJ_BLUE", obj_blue_h, obj_blue_l);
    obj_blue.setED(2, 4, 0, 2, 3, 2);
    obj_window.tackBarInit();

    auto start = std::chrono::steady_clock::now();

    // 主循环：只要对准摄像头打开，就一直跑
    while (CrawlCap.isOpened()) {

        // 每一轮循环都发送，把全局order_string刷新到串口屏
        control_screen();

        // 从串口com读取STM32下发的控制指令包 ReceiveData
        std::vector<ReceiveData> data;
        com.getData<ReceiveData>(data, 0, 0);

        if (!data.empty()) {
            // 收到STM32指令，1秒打印一次日志，防止刷屏
            auto now = std::chrono::steady_clock::now();
            auto e = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            if (e.count() > 1000) {
                start = now;
                std::cout << "收到控制命令: " << (int)(data.at(0).color) << " " << (int)(data.at(0).align_type) << std::endl;
            }
        } else {
            // 没有收到下位机指令，打印等待提示，跳过本轮循环
            auto now = std::chrono::steady_clock::now();
            auto e = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            if (e.count() > 1000) {
                start = now;
                std::cout << "未收到控制命令，等待中..." << std::endl;
            }
            continue;
        }

        // --------------------------
        // 解析STM32下发的指令
        // ReceiveData结构体包含两个关键字段：
        // color：0无效，1红色，2绿色，3蓝色
        // align_type：0无效，1色块OBJECT，2三点圆CIRCLE
        // --------------------------
        COLOR color = FALTE_COLOR;
        if (data.at(0).color == 1) color = RED_COLOR;
        if (data.at(0).color == 2) color = GREEN_COLOR;
        if (data.at(0).color == 3) color = BLUE_COLOR;

        ALIGNTYPE align_type = FALTE;
        if (data.at(0).align_type == 1) align_type = OBJECT;
        if (data.at(0).align_type == 2) align_type = CIRCLE;

        // ============任务分发逻辑============
        // ① align_type无效，color无效 → 执行【二维码识别 findQRCore】
        if (align_type == FALTE && color == FALTE_COLOR) {
            findQRCore(qr_window, is_open_debug);
        }
        // ② align_type=OBJECT色块 + 指定颜色 → 执行【色块跟踪 crawlObj】
        if (align_type == OBJECT && color != FALTE_COLOR) {
            crawlObj(obj_red, obj_green, obj_blue, obj_window, color, is_open_debug);
        }
        // ③ align_type=CIRCLE → 执行【三点共线圆检测 alginCircle】
        if (align_type == CIRCLE) {
            alginCircle(is_open_debug);
        }

        // PC调试：等待按键；MaixCam2删除这一行
        cv::waitKey(1);
    }

    // 跳出循环，摄像头断开，报错退出
    if ((is_open_qr && !QRCoreCap.isOpened())) {
        std::cerr << "二维码摄像头断开连接！" << std::endl;
        return -1;
    }
    if (!CrawlCap.isOpened()) {
        std::cerr << "抓取摄像头断开连接！" << std::endl;
        return -1;
    }
    return 0;
}
