
#ifndef LINE_H
#define LINE_H
class Line {

public:
    Line(double m, double b);

    static Line buildByPoints(Point& start, Point& end);

    static Line buildByPointAndAngle(Point& start, double angle);

    double getM() const;

    double getB() const;

private:
    double m, b;
};


class PointUtil {

public:
    static double orientation(Point& one, Point& two, Point& three);

    static Point vector(double angle, double length);

    static Point perpendicular(Point& one, Point& two, double length, int orientation);

    static const int CLOCKWISE = 1;
    static const int COUNTERCLOCKWISE = -1;
};

class LineSegment {

public:
    LineSegment(Point start, Point end);

    LineSegment(const Line &line, const Point &start, const Point &end);

    LineSegment(const LineSegment& copySegment);

    // ~LineSegment();

    double length();

    Line getLine();

    Point getStart();

    Point* getStartPtr();

    Point getEnd();

    Point* getEndPtr();

    Line line;
    Point *start, *end;
};


#endif /* LINE_H*/
