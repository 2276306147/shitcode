#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <chrono>

struct point_time
{
    Point pot; 
    long long time;
};

using namespace cv;
using namespace std;
vector<point_time> p_data;//储存点+时间戳
long long start_time;
Point r(3,4);//r坐标
Point target;//装甲板
double rid;//半径
double a, b;//运动方程参数
double bullet_time;//子弹飞行时间
vector<Point> fit_pot;//用于对比拟合数据
// 角速度函数：v = a * sin(b * t) + c
double hanshu(double t) {
    double c = 2.090 - a;
    return a * sin(b * t) + c;
}
int time_start()//记录开始时间
{
    auto timestamp = chrono::system_clock::now().time_since_epoch();
    start_time = chrono::duration_cast<chrono::milliseconds>(timestamp).count();

    return 0;

}
long long time_count()//记录当前毫秒数
{
    auto timestamp = chrono::system_clock::now().time_since_epoch();
    auto milliseconds = chrono::duration_cast<chrono::milliseconds>(timestamp).count()- start_time;

    return milliseconds;
}
//获取用于对比的点集与其对应时间戳，该函数不应当影响读帧
vector<point_time> get_Point()
{    
    Point OKpoint;//已经减去r坐标的装甲板坐标
    Point points;//收集观测数据
    long long times;//收集对应时间戳
    vector<point_time> temp;
    OKpoint.x = target.x - r.x;
    OKpoint.y = target.y - r.y;
    if (p_data.size() < 10)
    {
        points=(OKpoint);
        times=time_count();
        if (p_data.size() < 10)
        {
            p_data[p_data.size()-1].pot = points;
            p_data[p_data.size() - 1].time = times;
        }

    }
    if(p_data.size() == 10)
    {
        temp = p_data;
        p_data.clear();
        return temp;
    }
}


// 梯形法则数值积分函数，a，b为上下界
//输入：函数，上，下界
//输出：积分值
double trapezoidalIntegration(double (*function)(double), double upper, double lower) {
    int numSteps = 1000;
    double stepSize = (lower - upper) / numSteps;
    double integral = 0.0;

    for (int i = 0; i < numSteps; ++i) {
        double x1 = upper + i * stepSize;
        double x2 = upper + (i + 1) * stepSize;
        integral += ((*function)(x1) + (*function)(x2)) * 0.5 * stepSize; // 梯形面积的计算公式
    }

    return integral;
}


// 计算坐标变化数据
vector<Point2d> computeCoordinateData(  Point target, Point center ) {
    int f_m=300000;//五分钟的毫秒数
    int c_m = 75000;//一分十五秒的毫秒数
    vector<long long> time_counts;
    Point p_1;//储存预测起始点
    double x1;//虚拟预测点x坐标
    double y1;//虚拟预测点y坐标
    vector<Point2d> coordinateData;//储存预测点集
    vector<long long> t_interval;//时间间隔
    vector<double> jifen;//记录积分结果
    

    for(int i=0;i< get_Point().size();i++)
    {
        time_counts.push_back(get_Point()[i].time);//收集所有时间戳(未处理)
    }

    for (int i = 1; i < time_counts.size(); i++)
    {
        t_interval[0] = 0.0;//记录间隔
        t_interval.push_back(time_counts[i] - time_counts[i - 1]);
    }

    for (int i=1; i < time_counts.size()-1; i++)//处理数据
    {
        jifen.push_back(trapezoidalIntegration(hanshu, time_counts[1] - f_m, time_counts[i] - f_m));//收集对应时刻的速度积分
        if (jifen[i] / CV_PI <= 1)
        {
            x1 = rid * cos(jifen[i]);
            y1 = rid * sin(jifen[i]);
        }
        else
        {
            x1 = rid * cos(fmod(jifen[i] ,CV_PI));
            y1 = rid * sin(fmod(jifen[i], CV_PI));
        }
        Point2d temp;//临时储存预测点
        temp.x = x1;
        temp.y = y1;
        coordinateData.emplace_back(temp);

    }

    return coordinateData;
}

// 定义误差函数，用于拟合
double errorFunction(const vector<Point2d>& predictedData, const vector<Point2d>& observedData) {
    double error = 0.0;

    for (size_t i = 0; i < predictedData.size(); ++i) {
        double diff_x = predictedData[i].x - observedData[i].x;
        double diff_y = predictedData[i].y - observedData[i].y;
        error += diff_x * diff_x + diff_y * diff_y;
    }

    return error;
}
int B_prediction(double radius) {
    radius = 5.0;
    double deltaTime = 0.001;
    double targetTime = 3.0;
    vector<Point2d> observedData;//储存实际测量点

    // 假设已知的坐标变化数据（这里仅作示例，实际应根据具体数据填充）
    for(int i=0;i< get_Point().size();i++)
    {
        observedData.emplace_back(get_Point()[i].pot);
    }

    // 在指定范围内搜索最优的a、b、c值
    double bestA = 0.0, bestB = 0.0, bestC = 0.0;
    double smallestError = numeric_limits<double>::max();

    for (a = 0.780; a <= 1.045; a += 0.001) {
        for (b = 1.884; b <= 2.000; b += 0.001) {
            double c = 2.090 - a;
            vector<double> params = { a, b, c };
            vector<Point2d> predictedData = computeCoordinateData(target,r);
            double error = errorFunction(predictedData, observedData);

            if (error < smallestError) {
                smallestError = error;
                bestA = a;
                bestB = b;
                bestC = c;
            }
        }
    }
    a = bestA;
    b = bestB;
    trapezoidalIntegration(hanshu,time_count(),bullet_time);

    return 0;
}

//#include <iostream>
//#include <chrono>
//
//int main()
//{
//    auto timestamp = std::chrono::system_clock::now().time_since_epoch();
//    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp).count();
//
//    std::cout << "Timestamp in milliseconds: " << milliseconds << std::endl;
//
//    return 0;
//}
