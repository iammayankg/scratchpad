// #ifndef GEOMETRY_H
// #define GEOMETRY_H
#include <iostream>
#include <utility>
#include <set>
#include <vector>
#include <iterator>
#include <fstream>
#include <algorithm>
#include <sstream>
#include "draw_utils.h"
int width = 4000, height = 1000;
class Point
{

public:
    double m_x, m_y;
    Point(double x, double y) : m_x(x), m_y(y){};
    double x() { return m_x; };
    double y() { return m_y; };
    void x(double new_x) { m_x = new_x; };
    void y(double new_y) { m_y = new_y; };
    void printMe(std::stringstream &s)
    {
        s << "(" << x() << "," << y() << ")";
    }
    void printMe()
    {
        std::stringstream sout;
        printMe(sout);
        std::cout << sout.str();
    }
    void drawMe(cv::Mat &img, std::string prefix)
    {
        std::stringstream msg;
        msg << prefix;
        printMe(msg);
        cv::putText(img, msg.str(), cv::Point2d(x() + 2, y() + 2), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_8);
    }
};

class LineSegment
{
public:
    typedef std::pair<Point, Point> PointPair;
    PointPair points;
    double *m_currentY, slope, intercept;
    std::string m_name;
    LineSegment(Point begin, Point end, double *yStatus = 0) : points(begin, end), m_currentY(yStatus)
    {
        slope = (end.y() - begin.y()) / (end.x() - begin.x());
        intercept = end.y() - slope * end.x();
    };
    std::string &name()
    {
        return m_name;
    }
    double &currentY()
    {
        return *m_currentY;
    }
    double currentX()
    {
        return (currentY() - intercept) / slope;
    }
    double x1() { return points.first.x(); }
    double x2() { return points.second.x(); }
    double y1() { return points.first.y(); }
    double y2() { return points.second.y(); }
    bool intersection(LineSegment &other, Point &result)
    {
        double this_x1 = points.first.x();
        double this_x2 = points.second.x();
        double this_y1 = points.first.y();
        double this_y2 = points.second.y();

        double that_x1 = other.points.first.x();
        double that_x2 = other.points.second.x();
        double that_y1 = other.points.first.y();
        double that_y2 = other.points.second.y();

        double this_dx = this_x2 - this_x1;
        double this_dy = this_y2 - this_y1;
        double that_dx = that_x2 - that_x1;
        double that_dy = that_y2 - that_y1;

        double det = (-this_dx * that_dy) + (this_dy * that_dx);
        if (det == 0)
        {
            throw std::runtime_error("Bad implementation");
        }
        double r = (-that_dy * (that_x1 - this_x1) + that_dx * (that_y1 - this_y1)) / det;
        double s = (-this_dy * (that_x1 - this_x1) + this_dx * (that_y1 - this_y1)) / det;

        if (r < 0 or r > 1 or s < 0 or s > 1)
            return false;
        result.x(this_x1 + r * this_dx);
        result.y(this_y1 + r * this_dy);
        return true;
    }
    void printName(std::stringstream &sout)
    {
        sout << "[" << m_name << "]";
    }
    void printMe(std::stringstream &sout)
    {
        sout << "[" << m_name;
        points.first.printMe(sout);
        sout << "->";
        points.second.printMe(sout);
        sout << "]";
    }
    void printMe()
    {
        std::stringstream sout;
        printMe(sout);
        std::cout << sout.str();
    }
    void drawMe(cv::Mat &img)
    {
        cv::Point2d first(points.first.x(), points.first.y()),
            second(points.second.x(), points.second.y());
        drawLine(img, first, second);
        points.first.drawMe(img, "S");
        points.second.drawMe(img, "E");
        int lineType = 8;
        cv::Point2d textCoordinate((points.first.x() + points.second.x()) / 2,
                                   (points.first.y() + points.second.y()) / 2);
        cv::putText(img, m_name, textCoordinate, cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(0, 0, 255), 2, cv::LINE_8);
    };
};

struct CompareSegments
{
    bool operator()(LineSegment *a, LineSegment *b)
    {
        return a->currentX() < b->currentX();
    }
};

enum EventType
{
    INTERSECTION,
    START,
    END
};
class Event
{
public:
    EventType m_eventType;
    LineSegment *m_psegment1, *m_psegment2;
    Point m_IntersectionPoint;
    std::pair<double, double> key;
    Event(EventType type, LineSegment *segment1,
          LineSegment *segment2, Point intersectionPoint) : m_psegment1(segment1), m_psegment2(segment2),
                                                            m_eventType(type), m_IntersectionPoint(intersectionPoint)
    {
        if (type == START)
        {
            if ((segment1->y1() < segment1->y2()) ||
                ((segment1->y1() == segment1->y2()) &&
                 (segment1->x2() < segment1->x1())))
            {
                key.first = segment1->y2();
                key.second = -segment1->x2();
            }
            else
            {
                key.first = segment1->y1();
                key.second = -segment1->x1();
            }
        }
        else if (type == END)
        {
            if ((segment1->y1() > segment1->y2()) ||
                ((segment1->y1() == segment1->y2()) &&
                 (segment1->x2() > segment1->x1())))
            {
                key.first = segment1->y2();
                key.second = -segment1->x2();
            }
            else
            {
                key.first = segment1->y1();
                key.second = -segment1->x1();
            }
        }
        else if (type == INTERSECTION)
        {
            key.first = intersectionPoint.y();
            key.second = -intersectionPoint.x();
        }
        else
        {
            std::cout << "Something wrong";
        }
    }

    void drawMe(cv::Mat &img)
    {
        cv::Point2d p(-key.second, key.first);
        drawCircle(img, p, "green");
    }

    void printMe(std::stringstream &sout)
    {
        if (m_eventType == START)
        {
            sout << "START[" << key.first << "](";
            m_psegment1->printName(sout);
            sout << ")";
        }
        else if (m_eventType == END)
        {
            sout << "END[" << key.first << "](";
            m_psegment1->printName(sout);
            sout << ")";
        }
        else if (m_eventType == INTERSECTION)
        {
            sout << "INTERSECT[" << key.first << "](";
            m_psegment1->printName(sout);
            sout << " X ";
            m_psegment2->printName(sout);
            sout << ")";
        }
    }
};

struct CompareEvents
{
    bool operator()(Event *a, Event *b)
    {
        return (a->key.first == b->key.first) ? (a->key.second < b->key.second) : (a->key.first < b->key.first);
    }
};

class EventQueue
{
public:
    static EventQueue &getInstance()
    {
        static EventQueue equeue = EventQueue();
        return equeue;
    }
    std::set<Event *, CompareEvents> m_eventQueue;
    std::vector<Event *> m_events;
    EventQueue(){};
    void setEvents(std::vector<Event> *events)
    {
        for (auto i = events->begin(); i != events->end(); ++i)
        {
            m_eventQueue.insert(&(*i));
        }
    };
    Event *createEvent(EventType type, LineSegment *segment1,
                       LineSegment *segment2, Point intersectionPoint)
    {
        Event *e = new Event(type, segment1, segment2, intersectionPoint);
        m_events.push_back(e);
        return e;
    };
    void addEvent(Event *e)
    {
        m_eventQueue.insert(e);
    };
    void drawMe(cv::Mat &mat)
    {
        for (auto it = m_eventQueue.begin(); it != m_eventQueue.end(); ++it)
        {
            (*it)->drawMe(mat);
        }
    };

    void printMe(std::stringstream &sout)
    {
        for (auto it = m_eventQueue.begin(); it != m_eventQueue.end(); ++it)
        {
            Event *e = *it;
            e->printMe(sout);
        }
    }
};

class StatusQueue
{
public:
    std::set<LineSegment *, CompareSegments> m_statusQueue;
    StatusQueue(){};
    static StatusQueue &getInstance()
    {
        static StatusQueue equeue = StatusQueue();
        return equeue;
    }
    void printMe(std::stringstream &sout)
    {
        for (auto it = m_statusQueue.begin(); it != m_statusQueue.end(); ++it)
        {
            sout << "(";
            (*it)->printName(sout);
            sout << "[" << (*it)->currentX() << "," << (*it)->currentY() << "]";
            sout << ")";
        }
    }
    void drawMe(cv::Mat &img) {
        if (m_statusQueue.empty()) {
            return;
        }
        double currentY = (*m_statusQueue.begin())->currentY();
        cv::Point2d start(0, currentY), end(width, currentY);
        drawLine(img, start, end);
        for (auto it = m_statusQueue.begin(); it != m_statusQueue.end(); ++it)
        {
            cv::Point2d intersection = cv::Point2d((*it)->currentX(), (*it)->currentY());
            drawCircle(img, intersection, "blue");
        }
    }
};

void checkIntersection(int pos, int pos2, EventQueue &evQ, StatusQueue &sQ)
{
    LineSegment *segment = (*std::next(sQ.m_statusQueue.begin(), pos));
    LineSegment *segment2 = (*std::next(sQ.m_statusQueue.begin(), pos2));
    Point result(0, 0);
    bool intersectsSeg = segment->intersection(*segment2, result);
    if (intersectsSeg && result.y() < segment->currentY())
    {
        Event *e = evQ.createEvent(INTERSECTION, segment, segment2, result);
        evQ.addEvent(e);
    }
}

void handleStartEvent(LineSegment *segment, EventQueue &evQ, StatusQueue &sQ)
{
    // Event e(START, segment, NULL, Point(0, 0));
    // evQ.m_eventQueue.insert(&e);
    sQ.m_statusQueue.insert(segment);
    auto itSegment = sQ.m_statusQueue.find(segment);
    int pos = std::distance(sQ.m_statusQueue.begin(), itSegment);
    if (pos > 0)
    {
        checkIntersection(pos - 1, pos, evQ, sQ);
    }
    if (pos + 1 < sQ.m_statusQueue.size())
    {
        checkIntersection(pos, pos + 1, evQ, sQ);
    }
}

void handleEndEvent(LineSegment *segment, EventQueue &evQ, StatusQueue &sQ)
{
    auto itSegment = sQ.m_statusQueue.find(segment);
    if (itSegment == sQ.m_statusQueue.end()|| (*itSegment) != segment) {
        throw std::runtime_error("bad pointer");
    }
    int pos = std::distance(sQ.m_statusQueue.begin(), itSegment);
    sQ.m_statusQueue.erase(itSegment);
    if (pos > 0 && pos < sQ.m_statusQueue.size())
    {
        checkIntersection(pos - 1, pos, evQ, sQ);
    }
}

void handleIntersection(LineSegment *segment, LineSegment *segment2,
                        EventQueue &evQ, StatusQueue &sQ)
{
    if (segment->name() == "U" or segment2->name() == "U") {
        std::cout<<"here";
    }
    double currentY = segment->currentY();
    double epsilon = 0.0000001;
    segment->currentY() += epsilon;

    auto itSegment = sQ.m_statusQueue.find(segment);
    auto itSegment2 = sQ.m_statusQueue.find(segment2);

    int pos = std::distance(sQ.m_statusQueue.begin(), itSegment);
    int pos2 = std::distance(sQ.m_statusQueue.begin(), itSegment2);
    int posMin = std::min(pos, pos2);

    assert(*itSegment == segment);
    sQ.m_statusQueue.erase(itSegment);
    auto itInsert = sQ.m_statusQueue.insert(segment);
    assert((*(itInsert.first) == segment));
    
    sQ.m_statusQueue.erase(itSegment2);
    auto itInsert2 = sQ.m_statusQueue.insert(segment2);
    segment->currentY() -= epsilon;
    assert((*itInsert2.first) == segment2);
    segment2->currentY() -= epsilon;

    if (posMin > 0)
    {
        checkIntersection(posMin - 1, posMin, evQ, sQ);
    }
    if (posMin + 2 < sQ.m_statusQueue.size())
    {
        checkIntersection(posMin + 1, posMin + 2, evQ, sQ);
    }
    segment->currentY() = currentY;
}

class PlaneSweepInput
{
public:
    std::ifstream m_file;
    int num_segments;
    std::vector<LineSegment *> m_segmentVector;
    cv::Mat img;
    // StatusQueue sq;
    double *yStatus;
    void clearPatch()
    {
        cv::rectangle(img, cv::Point2d(0, height - 100), cv::Point2d(width, height),
                      cv::Scalar(255, 255, 255));
    }
    void updatePatch()
    {
        // clearPatch();
        EventQueue &eq = EventQueue::getInstance();
        StatusQueue &sq = StatusQueue::getInstance();
        eq.drawMe(img);
        sq.drawMe(img);
        std::stringstream sout, sout2;
        sout<<"Event Queue:=";
        eq.printMe(sout);
        std::string msg = sout.str();
        std::cout << msg;
        sout2<< "Status Queue:=";
        sq.printMe(sout2);
        std::string msg2 = sout2.str();
        double eqOffset = 50;
        double sqOffset = 100;

        cv::putText(img, msg, cv::Point2d(0, height - eqOffset), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_8);
        cv::putText(img, msg2, cv::Point2d(0, height - sqOffset), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_8);
    }
    char drawMe()
    {
        if (!img.empty())
        {
            img.release();
        }
        img = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
        for (auto it = m_segmentVector.begin(); it != m_segmentVector.end(); ++it)
        {
            LineSegment *l = *it;
            l->drawMe(img);
            l->printMe();
        }
        if (img.empty())
        {
            return 'x';
        }
        updatePatch();
        std::string msg = "hello";
        cv::namedWindow(msg.c_str(), cv::WINDOW_AUTOSIZE);
        cv::imshow(msg.c_str(), img);

        return cv::waitKey(0);
    }
    char screenInput()
    {
        return cv::waitKey(0);
    }
    void doSweep(StatusQueue &sq, EventQueue &eq, Event *nextEvent)
    {
        eq.m_eventQueue.erase(nextEvent);
        *yStatus = nextEvent->key.first;

        if (nextEvent->m_eventType == START)
        {
            handleStartEvent(nextEvent->m_psegment1, eq, sq);
        }
        else if (nextEvent->m_eventType == END)
        {
            handleEndEvent(nextEvent->m_psegment1, eq, sq);
        }
        else if (nextEvent->m_eventType == INTERSECTION)
        {
            handleIntersection(nextEvent->m_psegment1, nextEvent->m_psegment2, eq, sq);
        }
    }
    bool startSweep(EventQueue &eq, StatusQueue &sq)
    {
        if (eq.m_eventQueue.empty())
        {
            return false;
        }

        Event *nextEvent = *(eq.m_eventQueue.rbegin());
        doSweep(sq, eq, nextEvent);
        return !eq.m_eventQueue.empty();
    }
    PlaneSweepInput(std::string &filename) : m_file(filename)
    {
        num_segments = 0;
        m_file >> num_segments;
        std::cout << "Num segments:" << num_segments << std::endl;

        // std::vector<LineSegment> ls;
        double dInputX, dInputY;
        std::string dName;
        yStatus = new auto(0.0);
        *yStatus = 0;

        EventQueue &eq = EventQueue::getInstance();

        for (int i = 0; i < num_segments; ++i)
        {
            m_file >> dName;
            m_file >> dInputX;
            m_file >> dInputY;
            Point begin(dInputX, dInputY);
            m_file >> dInputX;
            m_file >> dInputY;
            Point end(dInputX, dInputY);
            *yStatus = std::max(begin.y(), end.y());
            LineSegment *l = new LineSegment(begin, end, yStatus);
            l->name() = dName;
            m_segmentVector.push_back(l);
            Event *lineStart = eq.createEvent(START, l, NULL, Point(0, 0));
            eq.addEvent(lineStart);
            Event *lineEnd = eq.createEvent(END, l, NULL, Point(0, 0));
            eq.addEvent(lineEnd);
        }

        char ch = drawMe();
        bool sweepCont = true;
        StatusQueue &sq = StatusQueue::getInstance();
        while (ch != 'x')
        {
            if (sweepCont)
            {
                sweepCont = startSweep(eq, sq);
            }
            if (ch == 'd')
            {
                drawMe();
            }
            ch = screenInput();
        }
    }
};

// #endif