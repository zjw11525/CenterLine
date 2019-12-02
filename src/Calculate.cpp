#include "Calculate.h"
#include <algorithm>
#include <cmath>

vector<Point> Calculate::calculte(vector<vector<int>> edge)
{
    vector<Point> result;
    vector<Point> border1;
    vector<Point> border2;
    this->GetBorderLine(edge, border1, border2); //获取用于迭代计算中心线的两条边

    //中心线起始点
    Point start(0, (border1[0].x + border2[0].x) / 2, (border1[0].y + border2[0].y) / 2);
    result.push_back(start);

    int rc1 = 0;
    int rc2 = 0;
    //迭代三角网计算
    while (rc1 + 1 < border1.size() && rc2 + 1 < border2.size())
    {
        Point a1 = border1[rc1];
        Point a2 = border1[rc1 + 1];
        Point b1 = border2[rc2];
        Point b2 = border2[rc2 + 1];
        double disa1_b2 = this->GetDistance2(a1, b2);
        double disa2_b1 = this->GetDistance2(a2, b1);
        if (disa1_b2 <= disa2_b1)
        {
            rc2++;
            Point temp(0, (a1.x + b2.x) / 2, (a1.y + b2.y) / 2);
            result.push_back(temp);
        }
        else
        {
            rc1++;
            Point temp(0, (a2.x + b1.x) / 2, (a2.y + b1.y) / 2);
            result.push_back(temp);
        }
    }

    //中心线终点
    Point end(0, (border1.back().x + border2.back().x) / 2, (border1.back().y + border2.back().y) / 2);
    result.push_back(end);

    //每个点赋予id
    for (int i = 0; i < result.size(); i++)
    {
        result[i].id = i;
    }

    return result;
}

vector<vector<Point>> Calculate::MultiRoad_calculte(vector<vector<int>> edge, bool is_alpha_route)
{
    vector<vector<Point>> split_road = this->RoadSplit(edge, is_alpha_route); //路径拆分
    vector<vector<Point>> result_multi_road;
    std::cout << "split complete! road size: " << split_road.size() << std::endl;
    vector<vector<int>> new_edge;
    bool is_snake_route;

    for (int i = 0; i < split_road.size(); i++)
    {
        //id重排序后计算中心线
        this->boundary = this->RoadReshape(split_road[i]);

        new_edge = this->GetEdge(is_snake_route); //重排序后的road重新计算起始终止边

        if (new_edge[0][0] == -1 && new_edge[0][1] == -1)
            return result_multi_road;

        if (i == 0) //特殊情况，第一条路径上id为0的点不在起始边，重新给定起始边
        {
            new_edge.push_back({split_road[i][0].id, split_road[i][split_road.size() - 1].id});
        }

        vector<Point> result = this->calculte(new_edge);
        std::cout << "calculate complete!" << std::endl;

        result_multi_road.push_back(result);

        new_edge.clear();
    }
    return result_multi_road;
}

void Calculate::GetBorderLine(vector<vector<int>> edge, vector<Point> &border1, vector<Point> &border2)
{
    int end1 = edge[1][0];
    int end2 = edge[1][1];

    int begin1 = std::max(edge[0][0], edge[0][1]);
    int begin2 = std::min(edge[0][0], edge[0][1]);

    int n = this->boundary.size();

    if (begin1 == n - 1 && begin2 == 0)
        std::swap(begin1, begin2);

    for (; begin1 != end1 && begin1 != end2; begin1 = (begin1 + 1) % n)
        border1.push_back(this->boundary[begin1]);
    border1.push_back(this->boundary[begin1]);

    for (; begin2 != end1 && begin2 != end2; begin2 = (begin2 + n - 1) % n)
        border2.push_back(this->boundary[begin2]);
    border2.push_back(this->boundary[begin2]);
}

double Calculate::GetMostElement(vector<double> list, int &sum)
{
    //遍历，获得每个元素出现的次数
    vector<int> A_per;
    for (int r = 0; r < list.size(); r++)
    {
        double tmp = list[r];
        int count = 0;
        for (int c = 0; c < list.size(); c++)
        {
            if (tmp == list[c])
            {
                count++;
            }
        }
        A_per.push_back(count);
    }

    auto max = std::max_element(A_per.begin(), A_per.end());
    sum = *max;

    for (int i = 0; i < list.size(); i++)
    {
        if (A_per[i] == *max)
            return list[i];
    }
    return list[0];
}

vector<vector<int>> Calculate::GetEdge()
{
    vector<vector<int>> res;
    int n = this->boundary.size();
    double w, l;
    this->GetWidthLength(w, l);

    for (int i = 0; i < n; i++)
    {
        if ((std::isnan(this->boundary[i].angle) == true) || (std::isnan(this->boundary[(i + 1) % n].angle) == true))
        {
            continue;
        }
        if (abs(this->boundary[i].angle + this->boundary[(i + 1) % n].angle - 180) < 0.01)
        {
            //std::cout << P[i].angle << "," << P[(i + 1) % n].angle << std::endl;
            double dis = this->GetDistanseFromId(this->boundary[i].id, this->boundary[(i + 1) % n].id);
            if ((2.5 * dis < w) && (2.5 * dis < l)) //起始边的2.5倍大于边框大小的边是冗余边
                res.push_back({this->boundary[i].id, this->boundary[(i + 1) % n].id});
        }
    }

    if (res.size() == 1)
    {
        vector<int> temp;
        for (auto p : this->boundary)
        {
            if (abs(p.angle - 90) < 0.01)
            {
                if (p.id != res[0][0] && p.id != res[0][1])
                    temp.push_back(p.id);
            }
        }
        if (temp.size() == 2)
            res.push_back({temp[0], temp[1]});
    }

    for (auto e : res)
        std::cout << "edge : " << e[0] << "," << e[1] << " width: " << GetDistanseFromId(e[0], e[1]) << std::endl;
    return res;
}

vector<vector<int>> Calculate::GetEdge(bool &Is_Snake_Route)
{
    vector<vector<int>> res;
    int n = this->boundary.size();
    double w, l;
    this->GetWidthLength(w, l);
    //std::cout << "w : " << w << "l: " << l << std::endl;

    vector<double> width_list;

    for (int i = 0; i < n; i++)
    {
        if ((std::isnan(this->boundary[i].angle) == true) || (std::isnan(this->boundary[(i + 1) % n].angle) == true))
        {
            continue;
        }
        if (abs(this->boundary[i].angle + this->boundary[(i + 1) % n].angle - 180) < 0.01)
        {
            //std::cout << P[i].angle << "," << P[(i + 1) % n].angle << std::endl;
            double dis = this->GetDistanseFromId(this->boundary[i].id, this->boundary[(i + 1) % n].id);
            if ((2.5 * dis < w) && (2.5 * dis < l)) //起始边的2.5倍大于边框大小的边是冗余边
            {
                res.push_back({this->boundary[i].id, this->boundary[(i + 1) % n].id});
                width_list.push_back(dis);
            }
        }
    }

    int smilar_edge_sum; //相同边的数量
    if (res.size() > 0)
    {
        double most_edge_width = this->GetMostElement(width_list, smilar_edge_sum); //相同边大小
                                                                                    //处理蛇型走线
        if (smilar_edge_sum > 10 && most_edge_width == Snake_Width)                 //相同边超过3且等于Snake_Width
        {
            Is_Snake_Route = true;
        }

        vector<vector<int>> temp_res(res);
        res.clear();
        for (int i = 0; i < temp_res.size(); i++)
        {
            //std::cout << "edge : " << e[0] << "," << e[1] << " width: " << GetDistanseFromId(e[0], e[1]) << std::endl;
            if (GetDistanseFromId(temp_res[i][0], temp_res[i][1]) != Snake_Width)
                res.push_back({temp_res[i][0], temp_res[i][1]});
        }

        //check edge 删除冗余边
        vector<vector<int>> temp_res1(res);
        for (int i = 0; i < temp_res1.size() - 1; i++)
        {
            vector<int>::iterator a = std::find(temp_res1[i].begin(), temp_res1[i].end(), temp_res1[(i + 1) % n][0]); //在两个边中找相同id
            vector<int>::iterator b = std::find(temp_res1[i].begin(), temp_res1[i].end(), temp_res1[(i + 1) % n][1]); //有相同id说明两条边相邻

            if (a != temp_res1[i].end() || b != temp_res1[i].end())
            {
                std::cout << "found same id" << std::endl;
                double width_a = GetDistanseFromId(temp_res1[i][0], temp_res1[i][1]);
                double width_b = GetDistanseFromId(temp_res1[(i + 1) % n][0], temp_res1[(i + 1) % n][1]); //相邻边删除长的

                if (std::max(width_a, width_b) == width_a)
                    res.erase(res.begin() + i); //删除数据后需要重新循环，否则size不对不能遍历到全部
                else
                    res.erase(res.begin() + (i + 1) % n);
            }
        }

        //删除过小的边
        if (res.size() > 2)
        {
            vector<double> edge_width;
            for (auto e : res)
                edge_width.push_back(GetDistanseFromId(e[0], e[1]));

            auto maxPosition = std::max_element(edge_width.begin(), edge_width.end()); //最大值
            for (int i = 0; i < res.size(); i++)
            {
                if (GetDistanseFromId(res[i][0], res[i][1]) < (*maxPosition / 10.0))
                {
                    res.erase(res.begin() + i);
                    i = 0;
                    continue;
                }
            }
        }
        for (auto e : res)
            std::cout << "edge : " << e[0] << "," << e[1] << " width: " << GetDistanseFromId(e[0], e[1]) << std::endl;
        return res;
    }
    res.push_back({-1, -1});
    return res;
}

vector<vector<int>> Calculate::GetEdge(bool &Is_Snake_Route, bool &Is_Alpha_Route, double &edge_width)
{
    vector<vector<int>> res;
    int n = this->boundary.size();
    double w, l;
    this->GetWidthLength(w, l);
    //std::cout << "w : " << w << "l: " << l << std::endl;

    vector<double> width_list;

    int angle_90_sum = 0;
    int angle_not90_sum = 0;
    for (auto p : this->boundary)
    {
        if (fabs(p.angle - 90) < 0.5)
            angle_90_sum++;
        else
            angle_90_sum++;
    }
    if (angle_90_sum - angle_not90_sum > 8)
    {
        Is_Alpha_Route = true;
    }

    for (int i = 0; i < n; i++)
    {
        if ((std::isnan(this->boundary[i].angle) == true) || (std::isnan(this->boundary[(i + 1) % n].angle) == true))
        {
            continue;
        }
        if (abs(this->boundary[i].angle + this->boundary[(i + 1) % n].angle - 180) < 0.01)
        {
            //std::cout << P[i].angle << "," << P[(i + 1) % n].angle << std::endl;
            double dis = this->GetDistanseFromId(this->boundary[i].id, this->boundary[(i + 1) % n].id);
            if ((2.5 * dis < w) && (2.5 * dis < l)) //起始边的2.5倍大于边框大小的边是冗余边
            {
                res.push_back({this->boundary[i].id, this->boundary[(i + 1) % n].id});
                width_list.push_back(dis);
            }
        }
    }

    int smilar_edge_sum;                                                        //相同边的数量
    double most_edge_width = this->GetMostElement(width_list, smilar_edge_sum); //相同边大小
    //处理蛇型走线
    if (smilar_edge_sum > 10 && most_edge_width == Snake_Width) //相同边超过3且等于Snake_Width
    {
        Is_Snake_Route = true;
    }

    vector<vector<int>> temp_res(res);
    res.clear();
    for (int i = 0; i < temp_res.size(); i++)
    {
        //std::cout << "edge : " << e[0] << "," << e[1] << " width: " << GetDistanseFromId(e[0], e[1]) << std::endl;
        if (GetDistanseFromId(temp_res[i][0], temp_res[i][1]) != Snake_Width)
            res.push_back({temp_res[i][0], temp_res[i][1]});
    }

    //check edge 删除冗余边
    vector<vector<int>> temp_res1(res);
    for (int i = 0; i < temp_res1.size() - 1; i++)
    {
        vector<int>::iterator a = std::find(temp_res1[i].begin(), temp_res1[i].end(), temp_res1[(i + 1) % n][0]); //在两个边中找相同id
        vector<int>::iterator b = std::find(temp_res1[i].begin(), temp_res1[i].end(), temp_res1[(i + 1) % n][1]); //有相同id说明两条边相邻

        if (a != temp_res1[i].end() || b != temp_res1[i].end())
        {
            std::cout << "found same id" << std::endl;
            double width_a = GetDistanseFromId(temp_res1[i][0], temp_res1[i][1]);
            double width_b = GetDistanseFromId(temp_res1[(i + 1) % n][0], temp_res1[(i + 1) % n][1]); //相邻边删除长的

            if (std::max(width_a, width_b) == width_a)
                res.erase(res.begin() + i); //删除数据后需要重新循环，否则size不对不能遍历到全部
            else
                res.erase(res.begin() + (i + 1) % n);
        }
    }

    //删除过小的边
    if (res.size() > 2)
    {
        vector<double> edge_width;
        for (auto e : res)
            edge_width.push_back(GetDistanseFromId(e[0], e[1]));

        auto maxPosition = std::max_element(edge_width.begin(), edge_width.end()); //最大值
        for (int i = 0; i < res.size(); i++)
        {
            if (GetDistanseFromId(res[i][0], res[i][1]) < (*maxPosition / 10.0))
            {
                res.erase(res.begin() + i);
                i = 0;
                continue;
            }
        }
    }

    if (Is_Alpha_Route)
    {
        vector<double> dis_list;
        for (auto e : res)
            dis_list.push_back(this->GetDistanseFromId(e[0], e[1]));
        auto max = std::max_element(dis_list.begin(), dis_list.end());
        edge_width = *max;
    }

    for (auto e : res)
        std::cout << "edge : " << e[0] << "," << e[1] << " width: " << this->GetDistanseFromId(e[0], e[1]) << std::endl;
    return res;
}

void Calculate::GetWidthLength(double &w, double &l)
{
    vector<double> x_list;
    vector<double> y_list;
    for (auto p : this->boundary)
    {
        x_list.push_back(p.x);
        y_list.push_back(p.y);
    }
    auto x_max = std::max_element(x_list.begin(), x_list.end());
    auto x_min = std::min_element(x_list.begin(), x_list.end());
    auto y_max = std::max_element(y_list.begin(), y_list.end());
    auto y_min = std::min_element(y_list.begin(), y_list.end());

    w = *x_max - *x_min;
    l = *y_max - *y_min;
}

double Calculate::GetDistance2(Point a, Point b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

double Calculate::GetDistanseFromId(int id_from, int id_to)
{
    double x1 = this->boundary[id_from].x;
    double y1 = this->boundary[id_from].y;
    double x2 = this->boundary[id_to].x;
    double y2 = this->boundary[id_to].y;
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<Point> Calculate::GetRouteFromRouteId(vector<int> id_list)
{
    vector<Point> route;
    for (auto Point : this->boundary)
    { //查找idlist中是否存在这个点
        vector<int>::iterator result = std::find(id_list.begin(), id_list.end(), Point.id);
        if (result != id_list.end())
            route.push_back(Point);
    }
    return route;
}

vector<Point> Calculate::RoadReshape(vector<Point> points)
{
    vector<Point> P(points);
    //重排序
    for (int i = 0; i < P.size(); i++)
    {
        P[i].id = i;
    }
    return P;
}

vector<vector<Point>> Calculate::RoadSplit(vector<vector<int>> edge, bool is_alpha_route)
{
    vector<int> route_num;
    vector<Point> route_split;
    vector<vector<Point>> route;

    double search_width_local = Search_Width;
    if (is_alpha_route)
    {
        search_width_local = Search_Width_Alpha;
    }

    int start_id = this->boundary[0].id; //起始ID是第一个点

    for (int k = 0; k < edge.size(); k++)
    {
        double width_start = 0;
        double width_end = 0;
        double externArea = 0;

        width_start = this->GetDistanseFromId(0, 0); //没有起始边宽度

        width_end = this->GetDistanseFromId(edge[0][0], edge[0][1]);

        width_start >= width_end ? externArea = width_start *search_width_local : externArea = width_end * search_width_local;
        //TODO 宽度只取了终点边，没有起始边

        for (auto Point : this->boundary)
        {
            for (int j = start_id; j <= edge[k][0] || j <= edge[k][1]; j++)
            {
                double dis = this->GetDistanseFromId(Point.id, j);
                if (dis <= externArea) //搜索宽度
                {
                    route_num.push_back(Point.id);
                    continue;
                }
            }
        }

        route_split = this->GetRouteFromRouteId(route_num);
        route_num.clear();
        route.push_back(route_split);
        start_id = edge[k][1];
    }
    //std::cout << "route_num:" << route.size() << std::endl;
    return route;
}