///@file geometry.h
///@brief Some 2d geometry functions and data structures.
///@date 8-5-2021
///@author: s.aparajith@live.com
///@copyright : none reserved. No liabilities. MIT License

#ifndef GEOMETRY_H_
#define GEOMETRY_H_
namespace geo_tools{
template<typename T>
struct point2d
{
    T x;
    T y;
    point2d()=default;
    point2d(double x,double y)
    {
        this.x=x;
        this.y=y;
    }
    point2d(const T other)
    {
        this->x = other.x;
        this->y = other.y;
    }
    
    point2d(const point2d<float>& other)
    {
        this->x = static_cast<T>(other.x);
        this->y = static_cast<T>(other.y);
    }

    // find the 2D shortest path between self and another point.
    T cart2d(const point2d<double>& other) const
    {
        return sqrt((this->y - other.y) * (this->y - other.y) +
            (this->x - other.x) * (this->x - other.x));
    }
    T cart2d(const point2d<float>& other) const
    {
        return sqrtf((this->y - other.y) * (this->y - other.y) +
            (this->x - other.x) * (this->x - other.x));
    }
};
}
#endif