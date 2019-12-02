#include "Optimizer.h"
#include "Calculate.h"
#include <algorithm>
#include <cmath>

vector<Point> Optimizer::PointOptimize()
{
    int n = this->result.size();
    vector<Point> new_result = {this->result[0]};

    std::cout << "original points sum:" << this->result.size() << std::endl;
    //去除相邻接近点
    for (int i = 1; i < n; i++)
    {
        Point Point1 = this->result[i - 1];
        Point Point2 = this->result[i];

        if (fabs(Point1.x - Point2.x) < Distance_Optimize_Threshold && fabs(Point1.y - Point2.y) < Distance_Optimize_Threshold)
            continue;
        new_result.push_back(Point2);
    }
    std::cout << "after distance check sum:" << new_result.size() << std::endl;

    //接近直线的点
    this->result = this->LinePointOptimize(new_result);
    std::cout << "after line optimize sum:" << result.size() << std::endl;

    return this->result;
}

vector<Point> Optimizer::PointOptimize(vector<Point> boundary, vector<vector<int>> edge, bool is_snake_route)
{
    int n = this->result.size();
    vector<Point> new_result = {this->result[0]};

    std::cout << "original points sum:" << this->result.size() << std::endl;
    //去除相邻接近点
    for (int i = 1; i < n; i++)
    {
        Point Point1 = this->result[i - 1];
        Point Point2 = this->result[i];

        if (fabs(Point1.x - Point2.x) < Distance_Optimize_Threshold && fabs(Point1.y - Point2.y) < Distance_Optimize_Threshold)
            continue;
        new_result.push_back(Point2);
    }
    std::cout << "after distance check sum:" << new_result.size() << std::endl;

    //去除特殊点 case3
    if (new_result.size() < 15)
        new_result = this->UniquePointOptimize(boundary, new_result, edge);

    new_result = this->LinePointOptimize(new_result);
    std::cout << "after line optimize sum:" << new_result.size() << std::endl;

    //新方法，优化拐点
    if (new_result.size() < Cross_Optimize_Threshold || is_snake_route)
    {
        vector<vector<int>> indexs = this->GetIndexFromInflectionPoint(new_result, is_snake_route);
        new_result = this->CrossPointOptimize(indexs, new_result);
    }

    this->result = new_result;

    return this->result;
}

vector<Point> Optimizer::UniquePointOptimize(vector<Point> points, vector<Point> centerline, vector<vector<int>> edge)
{
    Calculate calculate(points);
    vector<Point> r1, r2;
    calculate.GetBorderLine(edge, r1, r2);

    for (Point p : centerline)
    {
        vector<double> dis_list_1;
        vector<double> dis_list_2;
        for (int i = 0; i < r1.size(); i++)
        {
            dis_list_1.push_back(this->GetDistanceFrom2(p, r1[i]));
        }
        auto min1 = std::min_element(dis_list_1.begin(), dis_list_1.end());

        for (int i = 0; i < r2.size(); i++)
        {
            dis_list_2.push_back(this->GetDistanceFrom2(p, r2[i]));
        }
        auto min2 = std::min_element(dis_list_2.begin(), dis_list_2.end());

        double scale;
        *min1 > *min2 ? scale = *min1 - *min2 : scale = *min2 - *min1;
        if (scale > 0.4 && this->GetAngle(centerline, p.id) < 15) //角度小于20的特殊点
        {
            std::cout << scale << std::endl;
            centerline.erase(centerline.begin() + p.id);
        }
        //TODO只考虑有一个特殊点
    }
    return centerline;
}

vector<Point> Optimizer::LinePointOptimize(vector<Point> points)
{
    vector<Point> new_result;
    new_result.push_back(points[0]);
    int n = points.size();
    double val = Line_Optimize_min_Threshold;

    for (int i = 1; i < n - 1; i++)
    {
        double angle_last = this->GetAngle(points, i - 1);
        double angle = this->GetAngle(points, i);
        double angle_next = this->GetAngle(points, i + 1);
        double length = this->GetDistanceFrom2(points[i - 1], points[i + 1]);

        if (points.size() < 15 && length < 50) //小图
        {
            val = Line_Optimize_max_Threshold;
        }

        if (points.size() > 50 && length < 50 && angle_last > 20 && angle_next > 20) //大图波浪线
        {
            val = Line_Optimize_max_Threshold;
        }

        if (fabs(angle) > val)
        {
            //std::cout << "angle:" << angle << std::endl;
            new_result.push_back(points[i]); //两条线接近直线
            val = Line_Optimize_min_Threshold;
        }
    }
    new_result.push_back(points.back());

    for (int i = 0; i < new_result.size(); i++)
    {
        new_result[i].id = i;
    }
    return new_result;
}

void Optimizer::MultiRoadOptimizer(vector<vector<Point>> &points)
{
    //单个优化
    vector<vector<Point>> multi_result;
    for (int i = 0; i < points.size(); i++)
    {
        this->result = points[i];
        multi_result.push_back(PointOptimize());
    }

    points = multi_result;

    //整体优化
    vector<Point> result(points[0]);
    vector<double> dis_list;
    vector<Point> mid_result;

    for (int i = 1; i < points.size(); i++)
    {
        int index = 0;
        //std::cout << "M road size:" << points[i].size() << std::endl;
        for (auto Point1 : points[i])
        {
            for (auto Point2 : result) //路点到result每个点的距离
                dis_list.push_back(this->GetDistanceFrom2(Point1, Point2));

            auto dis_min = std::min_element(dis_list.begin(), dis_list.end()); //路点到result的最小距离

            if (*dis_min > MultiRoad_Optimize_Threshold)
            {
                mid_result.push_back(Point1); //距离大于设定值的点保留，其余删除
            }
            else
            {
                index = Point1.id; //记录第一个不重合的点的index,删除index之前的Point
            }
            dis_list.clear();
        }
        for (auto r : mid_result)
            result.push_back(r);

        mid_result.clear();

        points[i].erase(points[i].begin(), points[i].begin() + index);
        //std::cout << "R road size:" << points[i].size() << std::endl;
    }
    std::cout << "after multi road optimize:" << result.size() << std::endl;
}

void Optimizer::BoundaryOptimize(vector<Point> &boundary, vector<vector<int>> &edge)
{
    double angle_begin = this->GetEdgeAngle(boundary, edge[0]);
    double angle_end = this->GetEdgeAngle(boundary, edge[1]);
    if (fabs(angle_begin - 180) < 0.5 && fabs(angle_end - 90) < 0.5) //L型
    {
        vector<Point> border;
        int number_angle = 0;
        for (Point p : boundary) //找出特殊图
        {
            if (fabs(p.angle - 90.0) < 0.3)
                number_angle++;
        }
        if (number_angle > 20)
        {
            int max1 = std::min(edge[0][0], edge[0][1]);
            int min1 = std::max(edge[0][0], edge[0][1]);
            int min2 = std::min(edge[1][0], edge[1][1]);
            int max2 = std::max(edge[1][0], edge[1][1]);
            //border.push_back(calculate.boundary[min1]);
            border.push_back(Point(0, boundary[min1].x, boundary[min1].y));
            border.push_back(Point(1, boundary[min1].x, boundary[min2].y));
            border.push_back(Point(2, boundary[min2].x, boundary[min2].y));
            border.push_back(Point(3, boundary[max2].x, boundary[max2].y));
            border.push_back(Point(4, boundary[max1].x, boundary[max2].y));
            border.push_back(Point(5, boundary[max1].x, boundary[max1].y));

            boundary = border;

            edge.clear();
            edge.push_back({0, 5});
            edge.push_back({2, 3});
        }
    }
    else if (fabs(angle_begin - 90) > 0.5 && fabs(angle_end - 180) > 0.5) //倒L型
    {
    }
    else if ((fabs(angle_begin - 180) > 0.5 && fabs(angle_end - 180) > 0.5) || (fabs(angle_begin - 90) > 0.5 && fabs(angle_end - 90) > 0.5))
    {
    }
}

double Optimizer::GetEdgeAngle(vector<Point> boundary, vector<int> edge)
{
    double x1 = boundary[edge[0]].x;
    double x2 = boundary[edge[1]].x;
    double y1 = boundary[edge[0]].y;
    double y2 = boundary[edge[1]].y;
    return (std::atan2(y2 - y1, x2 - x1) * 57.3);
}

double Optimizer::GetAnglePoint(Point a, Point b)
{
    return (std::atan2(b.y - a.y, b.x - a.x) * 57.3);
}

void Optimizer::GetCrossPoint(const Point &p1, const Point &p2, const Point &q1, const Point &q2, double &x, double &y)
{
    //求交点
    double tmpLeft, tmpRight;
    tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
    tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

    x = ((double)tmpRight / (double)tmpLeft);

    tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
    tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x - p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
    y = ((double)tmpRight / (double)tmpLeft);
}

void Optimizer::GetMaxMin(vector<Point> boundary, double &xmax, double &xmin, double &ymax, double &ymin)
{
    vector<double> x_list;
    vector<double> y_list;
    for (auto p : boundary)
    {
        x_list.push_back(p.x);
        y_list.push_back(p.y);
    }
    auto x_max = std::max_element(x_list.begin(), x_list.end());
    auto x_min = std::min_element(x_list.begin(), x_list.end());
    auto y_max = std::max_element(y_list.begin(), y_list.end());
    auto y_min = std::min_element(y_list.begin(), y_list.end());

    xmax = *x_max;
    xmin = *x_min;
    ymax = *y_max;
    ymin = *y_min;
}

double Optimizer::GetDistanceFrom2(Point a, Point b)
{
    //return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double Optimizer::GetAngle(vector<Point> &P, int index)
{
    int n = P.size();
    Point from = P[(index + n - 1) % n];
    Point cur = P[index];
    Point to = P[(index + 1) % n];
    double x1 = cur.x - from.x;
    double y1 = cur.y - from.y;
    double x2 = to.x - cur.x;
    double y2 = to.y - cur.y;
    double ang = (x1 * x2 + y1 * y2) / (sqrt((x1 * x1 + y1 * y1) * (x2 * x2 + y2 * y2)));
    return std::acos(ang) * (180 / 3.1415);
}

vector<vector<int>> Optimizer::GetIndexFromInflectionPoint(vector<Point> v, bool is_snake_route)
{
    vector<int> single_index;
    vector<vector<int>> indexs;
    double start_ds = this->GetDistanceFrom2(v[1], v[0]);
    double w, l;
    this->GetWidthLength(w, l, v);
    std::cout << w << "," << l << std::endl;

    double val1 = Cross_Minus_Min_Threshold;
    double val2 = Cross_Distance_Min_Threshold;

    if (w > 1000 || l > 1000)
    {
        val1 = Cross_Minus_Threshold;
        val2 = Cross_Distance_Threshold;
    }

    if (v.size() > Cross_Threshold_Big) //大图
    {
        val1 = Cross_Minus_Threshold;
        val2 = Cross_Distance_Threshold;
    }

    if (is_snake_route) //蛇型走线
    {
        val1 = Snake_Cross_Minus_Threshold;
        val2 = Snake_Cross_Distance_Threshold;
    }

    //std::cout << val1 << " " << val2 << std::endl;
    for (int i = 2; i < v.size(); ++i)
    {
        double ds = this->GetDistanceFrom2(v[i], v[i - 1]);
        double resdes = ds - start_ds;

        if (resdes < val1 && ds < val2 && start_ds < val2)
        {
            single_index.push_back(i);
            single_index.push_back(i - 1);
            single_index.push_back(i - 2);
        }
        else
        {
            if (single_index.empty() == false)
                indexs.push_back(single_index);
            single_index.clear();
        }
        start_ds = ds;
    }
    std::cout << "indexs size:" << indexs.size() << std::endl;

    return indexs;
}

vector<Point> Optimizer::CrossPointOptimize(vector<vector<int>> index, vector<Point> points)
{
    vector<Point> result;
    vector<Point> P(points);
    vector<int> id_list;
    vector<Point> new_point;

    for (int i = 0; i < index.size(); ++i)
    {
        sort(index[i].begin(), index[i].end());
        index[i].erase(unique(index[i].begin(), index[i].end()), index[i].end());
        for (int j = 0; j < index[i].size() - 1; j++)
            id_list.push_back(index[i][j]);

        double x, y;
        GetCrossPoint(P[index[i][0]], P[index[i][1]], P[index[i][index[i].size() - 2]], P[index[i][index[i].size() - 1]], x, y);

        P[index[i][index[i].size() - 1]].x = x;
        P[index[i][index[i].size() - 1]].y = y;
    }

    for (Point p : P)
    {
        vector<int>::iterator a = std::find(id_list.begin(), id_list.end(), p.id); //有相同id说明两条边相邻

        if (a == id_list.end())
            result.push_back(p);
    }

    std::cout << "after cross point optimize sum:" << result.size() << std::endl;
    return result;
}

void Optimizer::RemoveOddPoint(vector<Point> boundary, vector<vector<Point>> &result_multi_road, double edge_width)
{
    vector<vector<Point>> road_temp(result_multi_road);
    vector<double> dis_list;

    for (int i = 0; i < road_temp.size(); i++)
    {
        for (int j = 0; j < road_temp[i].size(); j++)
        {
            for (auto p : boundary)
            {
                double distance = this->GetDistanceFrom2(p, road_temp[i][j]);
                dis_list.push_back(distance);
            }
            if (dis_list.size() > 0)
            {
                auto min = std::min_element(dis_list.begin(), dis_list.end());
                if (*min > (edge_width * 1.5))
                {
                    result_multi_road[i].erase(result_multi_road[i].begin() + j);
                }
            }
            dis_list.clear();
        }
    }
}

void Optimizer::GetWidthLength(double &w, double &l, vector<Point> result)
{
    vector<double> x_list;
    vector<double> y_list;
    for (auto p : result)
    {
        x_list.push_back(p.x);
        y_list.push_back(p.y);
    }
    auto x_max = std::max_element(x_list.begin(), x_list.end());
    auto x_min = std::min_element(x_list.begin(), x_list.end());
    auto y_max = std::max_element(y_list.begin(), y_list.end());
    auto y_min = std::min_element(y_list.begin(), y_list.end());

    w = fabs(*x_max - *x_min);
    l = fabs(*y_max - *y_min);
}