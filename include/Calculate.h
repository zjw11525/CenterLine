#ifndef _CALCULATE_H_
#define _CALCULATE_H_

#include <iostream>
#include <vector>
#include "Point.h"

using std::vector;

class Calculate
{
public:
    Calculate() {}
    Calculate(vector<Point> boundary)
    {
        this->boundary = boundary;
    }
    ~Calculate() {}
    vector<Point> boundary;
    vector<Point> calculte(vector<vector<int>> edge);
    vector<vector<Point>> MultiRoad_calculte(vector<vector<int>> edge, bool is_alpha_route);
    vector<vector<int>> GetEdge(); //针对弧形边
    vector<vector<int>> GetEdge(bool &Is_Snake_Route);
    vector<vector<int>> GetEdge(bool &Is_Snake_Route, bool &Is_Alpha_Route, double &edge_width);
    void GetBorderLine(vector<vector<int>> edge, vector<Point> &border1, vector<Point> &border2);

private:
    const double Search_Width = 1.2;     //路径搜索宽度
    const double Search_Width_Alpha = 2; //字母路径搜索宽度
    const double Snake_Width = 40;       //蛇型走线宽度
    vector<vector<Point>> RoadSplit(vector<vector<int>> edge, bool is_alpha_route);
    vector<Point> RoadReshape(vector<Point> points);
    double GetDistance2(Point a, Point b);
    double GetDistanseFromId(int id_from, int id_to);
    void GetWidthLength(double &w, double &l);
    vector<Point> GetRouteFromRouteId(vector<int> id_list);
    double GetMostElement(vector<double> list, int &sum);
};

#endif