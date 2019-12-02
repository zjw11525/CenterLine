#ifndef __OPTIMIZER_H_
#define __OPTIMIZER_H_

#include <iostream>
#include <vector>
#include "Point.h"

using std::vector;

class Optimizer
{
public:
    Optimizer() {}
    Optimizer(vector<Point> &r_result) : result(r_result) {}
    ~Optimizer() {}
    vector<Point> PointOptimize();
    vector<Point> PointOptimize(vector<Point> boundary, vector<vector<int>> edge, bool is_snake_route);
    void MultiRoadOptimizer(vector<vector<Point>> &points);
    void BoundaryOptimize(vector<Point> &boundary, vector<vector<int>> &edge);
    void RemoveOddPoint(vector<Point> boundary, vector<vector<Point>> &result_multi_road, double edge_width);

private:
    const double Cross_Minus_Min_Threshold = 0.5;     //小图拐点距离差值阈值
    const double Cross_Distance_Min_Threshold = 1.5;  //小图拐点距离阈值
    const double Cross_Minus_Threshold = 1;           //大图拐点距离差值阈值
    const double Cross_Distance_Threshold = 5;        //大图拐点距离阈值
    const double Snake_Cross_Minus_Threshold = 3;     //蛇型走线拐点距离差值阈值
    const double Snake_Cross_Distance_Threshold = 10; //蛇型走线拐点距离阈值
    const int Cross_Optimize_Threshold = 100000;      //拐点优化点数阈值
    const int Cross_Threshold_Big = 30;               //大图点数阈值
    const double Distance_Optimize_Threshold = 0.25;  //相似点优化阈值
    const double Line_Optimize_min_Threshold = 0.2;   //小图直线优化阈值
    const double Line_Optimize_max_Threshold = 3.3;   //大图直线优化阈值
    const double MultiRoad_Optimize_Threshold = 0.5;  //多条路降重复点阈值

    vector<Point> result;
    double GetAngle(vector<Point> &P, int index);
    double GetDistanceFrom2(Point a, Point b);
    vector<Point> LinePointOptimize(vector<Point> points);
    void GetMaxMin(vector<Point> boundary, double &xmax, double &xmin, double &ymax, double &ymin);
    double GetEdgeAngle(vector<Point> boundary, vector<int> edge);
    double GetAnglePoint(Point a, Point b);
    void GetCrossPoint(const Point &p1, const Point &p2, const Point &q1, const Point &q2, double &x, double &y);
    vector<Point> UniquePointOptimize(vector<Point> points, vector<Point> centerline, vector<vector<int>> edge);
    vector<Point> CrossPointOptimize(vector<vector<int>> index, vector<Point> points);
    vector<vector<int>> GetIndexFromInflectionPoint(vector<Point> v, bool is_snake_route);
    void GetWidthLength(double &w, double &l, vector<Point> result);
};

#endif