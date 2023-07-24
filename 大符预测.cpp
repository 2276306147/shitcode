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
vector<point_time> p_data;//�����+ʱ���
long long start_time;
Point r(3,4);//r����
Point target;//װ�װ�
double rid;//�뾶
double a, b;//�˶����̲���
double bullet_time;//�ӵ�����ʱ��
vector<Point> fit_pot;//���ڶԱ��������
// ���ٶȺ�����v = a * sin(b * t) + c
double hanshu(double t) {
    double c = 2.090 - a;
    return a * sin(b * t) + c;
}
int time_start()//��¼��ʼʱ��
{
    auto timestamp = chrono::system_clock::now().time_since_epoch();
    start_time = chrono::duration_cast<chrono::milliseconds>(timestamp).count();

    return 0;

}
long long time_count()//��¼��ǰ������
{
    auto timestamp = chrono::system_clock::now().time_since_epoch();
    auto milliseconds = chrono::duration_cast<chrono::milliseconds>(timestamp).count()- start_time;

    return milliseconds;
}
//��ȡ���ڶԱȵĵ㼯�����Ӧʱ������ú�����Ӧ��Ӱ���֡
vector<point_time> get_Point()
{    
    Point OKpoint;//�Ѿ���ȥr�����װ�װ�����
    Point points;//�ռ��۲�����
    long long times;//�ռ���Ӧʱ���
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


// ���η�����ֵ���ֺ�����a��bΪ���½�
//���룺�������ϣ��½�
//���������ֵ
double trapezoidalIntegration(double (*function)(double), double upper, double lower) {
    int numSteps = 1000;
    double stepSize = (lower - upper) / numSteps;
    double integral = 0.0;

    for (int i = 0; i < numSteps; ++i) {
        double x1 = upper + i * stepSize;
        double x2 = upper + (i + 1) * stepSize;
        integral += ((*function)(x1) + (*function)(x2)) * 0.5 * stepSize; // ��������ļ��㹫ʽ
    }

    return integral;
}


// ��������仯����
vector<Point2d> computeCoordinateData(  Point target, Point center ) {
    int f_m=300000;//����ӵĺ�����
    int c_m = 75000;//һ��ʮ����ĺ�����
    vector<long long> time_counts;
    Point p_1;//����Ԥ����ʼ��
    double x1;//����Ԥ���x����
    double y1;//����Ԥ���y����
    vector<Point2d> coordinateData;//����Ԥ��㼯
    vector<long long> t_interval;//ʱ����
    vector<double> jifen;//��¼���ֽ��
    

    for(int i=0;i< get_Point().size();i++)
    {
        time_counts.push_back(get_Point()[i].time);//�ռ�����ʱ���(δ����)
    }

    for (int i = 1; i < time_counts.size(); i++)
    {
        t_interval[0] = 0.0;//��¼���
        t_interval.push_back(time_counts[i] - time_counts[i - 1]);
    }

    for (int i=1; i < time_counts.size()-1; i++)//��������
    {
        jifen.push_back(trapezoidalIntegration(hanshu, time_counts[1] - f_m, time_counts[i] - f_m));//�ռ���Ӧʱ�̵��ٶȻ���
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
        Point2d temp;//��ʱ����Ԥ���
        temp.x = x1;
        temp.y = y1;
        coordinateData.emplace_back(temp);

    }

    return coordinateData;
}

// �����������������
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
    vector<Point2d> observedData;//����ʵ�ʲ�����

    // ������֪������仯���ݣ��������ʾ����ʵ��Ӧ���ݾ���������䣩
    for(int i=0;i< get_Point().size();i++)
    {
        observedData.emplace_back(get_Point()[i].pot);
    }

    // ��ָ����Χ���������ŵ�a��b��cֵ
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
