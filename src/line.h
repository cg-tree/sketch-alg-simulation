
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

Line::Line(double m, double b) : m(m), b(b) {}

Line Line::buildByPoints(Point &start, Point &end) {
    double m = (end.getY() - start.getY()) / (end.getX() - start.getX() + 1e-9); //divide by zero case solved by 1e-9
    double b = start.getY() - (m * start.getX());

    return Line(m, b);
}

Line Line::buildByPointAndAngle(Point &start, double angle) {
    double m = tan(angle);
    double b = start.getY() - (m * start.getX());

    return Line(m, b);
}

double Line::getM() const {
    return m;
}


double Line::getB() const {
    return b;
}

LineSegment::LineSegment(Point start, Point end) : line(Line::buildByPoints(start, end)), start(new Point(start.getX(), start.getY())), end(new Point(end.getX(), end.getY())) {}

LineSegment::LineSegment(const Line &line, const Point &start, const Point &end) : line(line), start(new Point(start.getX(), start.getY())), end(new Point(end.getX(), end.getY())) {}

LineSegment::LineSegment(const LineSegment &copySegment): line(copySegment.line), start(new Point(copySegment.start->getX(), copySegment.start->getY())), end(new Point(copySegment.end->getX(), copySegment.end->getY())) {}

double LineSegment::length() {
    Point vector = (*end - *start);
    return vector.length();
}

Line LineSegment::getLine() {
    return line;
}

Point LineSegment::getStart() {
    return *start;
}

Point* LineSegment::getStartPtr() {
    return start;
}

Point LineSegment::getEnd() {
    return *end;

}

// LineSegment::~LineSegment() {
//     delete start;
//     delete end;
// }
#endif /* LINE_H*/
