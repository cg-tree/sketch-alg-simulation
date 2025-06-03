
#include "point.h"
#include "line.h"

using namespace std;

class Ellipse {

public:
    Ellipse (){}
    Ellipse(Point centergiven, double x_radius, double y_radius)
    {
        center = centergiven;
        radius_x = x_radius;
        radius_y = y_radius;
        _size = M_PI * radius_x * radius_y;
    }

    bool inside(const Point &vector);

    double size();

    LineSegment segmentIntersections(LineSegment &segment);

    LineSegment intersections(Line &line);

    bool crosses(LineSegment &segment);

    bool crossesEdge(LineSegment &segment);

    Point getCross(LineSegment &segment) ;

    double edgeGradient(Point& point);

    Point getCenter() {
        return center;
    }

    double getXRadius() {
        return radius_x;
    }

    double getYRadius() {
        return radius_y;
    }
    
    Point center;
    double radius_x, radius_y;
    double _size;
};


bool Ellipse::crosses(LineSegment &segment) {
    Line line = segment.getLine();

    double rx = radius_x * radius_x;
    double ry = radius_y * radius_y;

    double a = (1 / rx) + (line.getM() * line.getM() / ry);
    double b = (2 * line.getB() * line.getM() / ry) - (2 * center.getX() / rx) - (2 * center.getY() * line.getM() / ry);
    double c = (line.getB() * line.getB() / ry) - (2 * line.getB() * center.getY() / ry) + (center.getX() * center.getX() / rx) + (center.getY() * center.getY() / ry) - 1;

    // Solution using Quadratic equation -b +- sqrt(b^2 - 4ac)/2a
    // where ax^2 + bx + c = 0
    double discriminant = pow(b,2) - (4 * a * c);

    if (discriminant > 0){
        double x1 = ((-b) + sqrt(discriminant)) / (2 * a);
        double x2 = ((-b) - sqrt(discriminant)) / (2 * a);

        return (segment.getStart().getX() < x1 && segment.getStart().getX() > x2)
            || (segment.getEnd().getX() < x1 && segment.getEnd().getX() > x2);
    } else {
        return false;
    }
}


bool Ellipse::crossesEdge(LineSegment &segment) {
    Line line = segment.getLine();
    LineSegment intersectionSegment = intersections(line);

    Point start = intersectionSegment.getStart();
    Point end = intersectionSegment.getEnd();

    Point segmentStart = segment.getStart();
    Point segmentEnd = segment.getEnd();
    
    //check if discriminant is negative
    if (start.x == 0 && start.y == 0 && end.x == 0 && end.y == 0)
        return false;

    if(segmentStart.getX() > segmentEnd.getX()) {
        Point tmp = segmentStart;
        segmentStart = segmentEnd;
        segmentEnd = tmp;
    }

    return (start.getX() > segmentStart.getX() && start.getX() < segmentEnd.getX()) ||
            (end.getX() > segmentStart.getX() && end.getX() < segmentEnd.getX());
}



Point Ellipse::getCross(LineSegment &segment) {
    Line line = segment.getLine();
    LineSegment intersectionSegment = intersections(line);

    Point start = intersectionSegment.getStart();
    Point end = intersectionSegment.getEnd();

    Point segmentStart = segment.getStart();
    Point segmentEnd = segment.getEnd();
    
    
    //check if discriminant is negative
    if (abs(start.x) <= 1e-9 && abs(start.y) <= 1e-9 && abs(end.x) <= 1e-9 && abs(end.y) <= 1e-5){
        cout<<"Exception with discriminant!!"<<endl;
        return Point (0,0);
    }
    
    if (abs(get_dist (start, segmentStart) + get_dist (start, segmentEnd) - get_dist (segmentStart,segmentEnd)) <= 1e-5)
        return start;

    
    if (abs(get_dist (end, segmentStart) + get_dist (end, segmentEnd) - get_dist (segmentStart,segmentEnd)) <= 1e-5)
        return end;

    cout<<"Exception! at Ovals "<<segmentStart.x << " "<<segmentStart.y<<" " <<segmentEnd.x << " " <<segmentEnd.y<< endl;
    cout<<"Ovals cont. "<<start.x<<" "<<start.y<<" "<<end.x<<" "<<end.y << endl;
    return Point(0, 0);
}

double Ellipse::edgeGradient(Point& point) {
    return atan2(-pow(radius_y, 2) * (point.getX()-center.getX()), (pow(radius_x, 2) * (point.getY()-center.getY())));
}

