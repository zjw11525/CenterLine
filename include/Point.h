#ifndef __POINT_H_
#define __POINT_H_

#include <cmath>

class Point
{
public:
    Point() {}
    Point(int r_id, double r_x, double r_y) : id(r_id), x(r_x), y(r_y), angle(0) {}
    Point(double r_x, double r_y, double r_angle) : x(r_x), y(r_y), angle(r_angle) {}
    ~Point() {}
    int id;
    double x;
    double y;
    double angle;
    inline bool operator==(const Point &b)
    {
        if (fabs(this->x - b.x) < 0.0000001 && (fabs(this->y - b.y)) < 0.0000001)
            return true;
        else
            return false;
    }
};

#endif