
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

Point::Point(double x, double y) : x(x), y(y) {}
Point Point::operator+(Point &first) {
    return Point(x + first.x, y + first.y);
}

Point Point::operator+=(Point &first){
    this->x += first.x;
    this->y += first.y;
    return *this;
}

Point Point::operator-(Point &first) {
    return Point(x - first.x, y - first.y);
}

Point Point::operator-=(Point &first){
    this->x -= first.x;
    this->y -= first.y;
    return *this;
}

Point Point::operator-() {
    return Point(-this->x, -this->y);
}

double Point::length() {
    return sqrt(x * x + y * y);
}

double Point::getX() const {
    return x;
}

double Point::getY() const {
    return y;
}


class PointUtil {

public:
    static double orientation(Point& one, Point& two, Point& three);

    static Point vector(double angle, double length);

    static Point perpendicular(Point& one, Point& two, double length, int orientation);

    static const int CLOCKWISE = 1;
    static const int COUNTERCLOCKWISE = -1;
};

double PointUtil::orientation(Point& one, Point& two, Point& three) {
    double k=(two.getY() - one.getY())*(three.getX() - two.getX())-(two.getX() - one.getX()) * (three.getY() - two.getY());

    if(k>0) {
        return CLOCKWISE;
    } else {

        return COUNTERCLOCKWISE;
    }
}

Point PointUtil::vector(double angle, double length) {
    return Point(length * cos(angle), length * sin(angle));
}

Point PointUtil::perpendicular(Point &one, Point &two, double length, int orientation) {
    double delta_x = two.getX() - one.getX();
    double delta_y = two.getY() - one.getY();
    double angle = atan2(delta_y, delta_x);
    return vector(angle + (orientation * M_PI_2), length);
}


double get_dist (Point A, Point B)
{
    return sqrt ((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}
#endif /*POINT_H*/
