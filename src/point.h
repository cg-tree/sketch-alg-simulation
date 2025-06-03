
#ifndef POINT_H
#define POINT_H

class Point {
public:

    Point (){}
    Point(double x, double y);

    Point operator+ (Point& first);

    Point operator- (Point& first);

    Point operator+= (Point& first);

    Point operator-= (Point& first);

    Point operator-();

    double length();

    double getX() const;

    double getY() const;

    bool operator< (const Point& other) const {
        return other.x < this->x || (other.x == this->x && other.y < this->y);
    }

    bool isNear(Point& other, double eps = 1e-6) {
        return std::abs(x - other.x) < eps && std::abs(y - other.y) < eps;
    }

    bool operator==(const Point& other) const {
        return std::abs(x - other.x) < 1e-9 && std::abs(y - other.y) < 1e-9;
    }

    double x,y;

};

double get_dist (Point A, Point B)
{
    return sqrt ((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}
#endif /*POINT_H*/
