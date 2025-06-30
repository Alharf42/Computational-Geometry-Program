#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QPointF>
#include <QMouseEvent>
#include <QPainter>
#include <QRandomGenerator>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->SurfaceAreaLabel->setVisible(false);

    connect(ui->Clear, &QPushButton::clicked, this, &MainWindow::on_Clear_clicked);
    connect(ui->MakePolygon, &QPushButton::clicked,this, &MainWindow::on_MakePolygon_clicked);
    connect(ui->ConvexHull, &QPushButton::clicked,this,&MainWindow::on_ConvexHull_clicked);
    connect(ui->Triangulate, &QPushButton::clicked,this,&MainWindow::on_Triangulate_clicked);
    connect(ui->SurfaceArea, &QPushButton::clicked,this,&MainWindow::on_SurfaceArea_clicked);
    connect(ui->SmallestCircle, &QPushButton::clicked,this,&MainWindow::on_SmallestCircle_clicked);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QPointF point = event->pos();

    points.append(point);
    //trigger paintevent after each click to draw new point
    update();

}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setPen(Qt::red);

    //draws all the points every time
    for(const QPointF point : points)
    {
        painter.drawPoint(point);
    }

    //each previous point dissapears and just the current point is drawn

    if(drawPolygon)
    {
        QPainter painter(this);
        painter.setPen(Qt::white);
        painter.setBrush(myRed);
        //store each drawn polygon--currently there will be always just one, since all points are used to draw current polygon
        polygons.append(QPolygonF(points));
        painter.drawPolygon(QPolygonF(points), Qt::OddEvenFill);
    }
    if(drawConvexHull)
    {
        if(points.size()!=0)
        {
        QPolygonF convexHull = findConvexHull(QPolygonF(points));
        if(!drawSmallestCircle)
        {
            QPainter painter(this);
            painter.setPen(Qt::white);
            painter.setBrush(myYellow);
            painter.drawPolygon(convexHull, Qt::OddEvenFill);
        }
        else
        {
            QPainter painter(this);
            std::pair<QPointF, qreal> forCircle = findSmallestCircle(convexHull);
            painter.drawEllipse(forCircle.first, forCircle.second, forCircle.second);
        }
        }
    }
    if(drawTriangulation)
    {
        QVector<QPolygonF> triangles=triangulate(points);
        qreal polygonArea = 0;
        for(QPolygonF triangle : triangles)
        {
            QPainter painter(this);
            painter.setPen(Qt::white);
            int index = QRandomGenerator::global()->bounded(0,colors.size());
            painter.setBrush(colors[index]);
            painter.drawPolygon(triangle, Qt::OddEvenFill);

            if (computeSurfaceArea && points.size() >2)
            {
                QVector3D ViVminus = QVector3D(triangle[1] - triangle[0]);
                QVector3D ViVplus = QVector3D(triangle[2]-triangle[1]);
                QVector3D cross = QVector3D::crossProduct(ViVminus, ViVplus);
                qreal cross2D = cross.z();
                qreal triangleArea = cross2D/2;
                polygonArea +=std::abs(triangleArea);

            }
        }
        if(computeSurfaceArea)
        {
            ui->SurfaceAreaLabel->setVisible(true);
            ui->SurfaceAreaLabel->setText("Surface area of the polygon: "+QString::number(polygonArea)+" square pixels");
        }
    }

}

//restart button
void MainWindow::on_Clear_clicked()
{
    qDebug()<<"clear button clicked";
    points.clear();
    drawPolygon=false;
    drawConvexHull=false;
    drawTriangulation = false;
    drawSmallestCircle = false;
    computeSurfaceArea = false;
    ui->SurfaceAreaLabel->setVisible(false);
    update();
}
//button draw polygon
void MainWindow::on_MakePolygon_clicked()
{
    qDebug()<<"Make Polygon buton clicked";

    drawPolygon =true;
    drawConvexHull=false;
    drawTriangulation = false;
    drawSmallestCircle = false;
    computeSurfaceArea = false;
    ui->SurfaceAreaLabel->setVisible(false);
    update();

}
//////////////////////////////////////////////////////////////////////////////////////
//confex hull
//helper function for computing sorting the points from above the line and bellow

//
//they are ordered such that the convex hull is colored wrong
//sort points in polygon in counter clockwise order
void MainWindow::sortPolygon(QPolygonF& polygon)
{
    //1)find the center point
    QPointF center(0,0);
    for(const QPointF& p : polygon)
        center +=p;
    center/=polygon.size();

    //2)estimate the angle between each point and the center
    std::sort(polygon.begin(),polygon.end(), [center](const QPointF& a, const QPointF& b)
              {
        qreal angleA = std::atan2(a.y() - center.y(), a.x() - center.x());
        qreal angleB = std::atan2(b.y() - center.y(), b.x() - center.x());
        return angleA < angleB;
    });

}

qreal MainWindow::distanceFromLine(QPointF& minPoint, QPointF& maxPoint, QPointF& p)
{
    qreal dx = maxPoint.x() - minPoint.x();
    qreal dy = maxPoint.y() - minPoint.y();

    qreal numerator = std::abs(dy * p.x() - dx * p.y() + maxPoint.x()*minPoint.y() - maxPoint.y()*minPoint.x());
    qreal denominator = std::hypot(dx, dy);
    qreal distanceFromLine = numerator / denominator;
    return distanceFromLine;
}

std::pair<QPolygonF, QPolygonF> MainWindow::dividePolygonByLine(QPolygonF polygon,
                                                               QPointF minPoint,
                                                               QPointF maxPoint)
{
    QPolygonF setUp;
    QPolygonF setDown;

    QVector3D line = QVector3D(maxPoint - minPoint);
    for(QPointF p : polygon)
    {
        //with cross product determine if the point is bellow or upwards the line
        //a sign of the 2D cross product reveals a relative postion of the point
        //to the line

        //QPointF relativeDistance = p - minPoint;
        QVector3D relativeDistance = QVector3D(p - minPoint);

        QVector3D cross = QVector3D::crossProduct(line, relativeDistance);
        //after computing the 3D cross product the scalar result
        //of 2D cross product is in the z coordinate
        //points on the line are colineare thus not necessary
        qreal cross2D = cross.z();
        if(cross2D>0) setUp.append(p);
        else if(cross2D< 0) setDown.append(p);

    }
    return std::pair<QPolygonF, QPolygonF>{setUp, setDown};
}

//helper function to test more accurently than contains point
//whether a point is inside a triangle
//using 2D cross product
bool MainWindow::pointInTriangle(QPointF p, QPointF a, QPointF b, QPointF c)
{
    //lambda function computing the cross product and return its sign
    auto sign = [](QPointF p1, QPointF p2, QPointF p3) {
        return (p1.x() - p3.x()) * (p2.y() - p3.y()) -
               (p2.x() - p3.x()) * (p1.y() - p3.y());
    };

    //sign of cross product for each side of the triangle
    bool b1 = sign(p, a, b) <= 0.0;
    bool b2 = sign(p, b, c) <= 0.0;
    bool b3 = sign(p, c, a) <= 0.0;
//point is inside if all the signs match
    return ((b1 == b2) && (b2 == b3));
}
//helper function to find convex hull
void MainWindow::findHull(QPolygonF& polygon, QPointF& minPoint, QPointF& maxPoint, QPolygonF& convexHull)
{
    if(polygon.isEmpty()) return;
    //3)Find a point above the line with max distance from it

    //first i compute the distance from the line for every point
    //than i find the biggest distance

    QPointF maxUpPoint;
    qreal maxDistance=0;

    for(int i = 0;i<polygon.size();i++)
    {
        QPointF p = polygon[i];
        qreal distance = distanceFromLine(minPoint,maxPoint,p);
        if(distance>=maxDistance)
        //if(distance>maxDistance)
        {
            maxDistance=distance;
            maxUpPoint=p;
        }
    }

    convexHull << maxUpPoint;

    //4)kick out the points inside the triangle

    //QPolygonF triangle;
    //triangle  << maxPoint << maxUpPoint << minPoint;
    QPolygonF modifiedPolygon = polygon;
    for(QPointF p : polygon)
    {
        //if(triangle.containsPoint(p,Qt::OddEvenFill))
        if(pointInTriangle(p, maxPoint, maxUpPoint, minPoint))
        {//is this condition workinng?
            //maybe instead of only modifing the intial polygon
            //i should add the needed points to the final convex hull
            if(p != maxPoint && p != maxUpPoint && p != minPoint)
            modifiedPolygon.removeAll(p);
            //are edge and colinear points tested properly?
            //should i create a function using orientation tests instead?
        }
    }
    polygon = modifiedPolygon;

    //i need to do 2 again
    //helper function
    std::pair<QPolygonF, QPolygonF> dividedP = MainWindow::dividePolygonByLine(polygon, minPoint, maxPoint);

    //5)recursivly repeat 3-4
    //6)continue until nor more points are left
    //do parts 3-6 for point bellow the line
    //asign new polygon without points intriangel to convex Hull
    //if point is present(if we went over all points) call function
    //else return convecHullů
    //needs to change min and max too to call findHull on new min and max recursibly
    //do some of the original ones be part of the triangle always? probably not

    //i have to do this so it works on both left and rifht side, not it works only
    //for the right side probably

    //i need to find the porper condition for when there are no points left
    //if polygon not empty?
    if(!polygon.isEmpty())
    {
        findHull(dividedP.first, minPoint, maxUpPoint, convexHull);
        findHull(dividedP.second, maxUpPoint, maxPoint, convexHull);
    }
    else
    {
        return;
    }

}
//function to find a convex hull of a polygon
QPolygonF MainWindow::findConvexHull(QPolygonF polygon)
{
    QPolygonF convexHull;
    //1) find points with min and max coordinates
    //first find those with min x if more then one choose by min y (or max)
    QPointF minPoint=polygon[0];
    QPointF maxPoint=polygon[0];
    for(QPointF p : polygon)
    {
        if(minPoint.x()>p.x()) minPoint=p;

        if(maxPoint.x()<p.x())maxPoint=p;
    }
    //if there are more then one points with min/max x coordinate
    int moreMinXPoints=0;
    int moreMaxXPoints=0;
    for(QPointF p : polygon)
    {
        if(minPoint.x()==p.x()) moreMinXPoints +=1;
        if(maxPoint.x()==p.x()) moreMaxXPoints +=1;
    }
    //choose them by min/max y coordinate
    if(moreMinXPoints>1)
    {
        for(QPointF p : polygon)
        {
            if(minPoint.y()>p.y()) minPoint=p;
        }

    }
    if(moreMaxXPoints>1)
    {
        for(QPointF p : polygon)
        {

            if(maxPoint.y()<p.y())maxPoint=p;
        }

    }

    convexHull << minPoint << maxPoint;

    //2)divide the polygon by line minPointmaxPoint
    QPolygonF setUp;
    QPolygonF setDown;

    //QPointF line = maxPoint - minPoint;
    QVector3D line = QVector3D(maxPoint - minPoint);
    for(QPointF p : polygon)
    {
        //with cross product determine if the point is bellow or upwards the line
        //a sign of the 2D cross product reveals a relative postion of the point
        //to the line

        //QPointF relativeDistance = p - minPoint;
        QVector3D relativeDistance = QVector3D(p - minPoint);

        QVector3D cross = QVector3D::crossProduct(line, relativeDistance);
        //after computing the 3D cross product the scalar result
        //of 2D cross product is in the z coordinate
        //points on the line are colineare thus not necessary
        qreal cross2D = cross.z();
        if(cross2D>0) setUp.append(p);
        else if(cross2D< 0) setDown.append(p);
    }

//asign new polygon without points intriangel to convex Hull
//if point is present(if we went over all points) call function
    //else return convecHullů
    if(!setUp.isEmpty())
    findHull(setUp, minPoint, maxPoint, convexHull);
    if(!setDown.isEmpty())
    findHull(setDown, maxPoint, minPoint, convexHull);

    sortPolygon(convexHull);
    return convexHull;
}
///////////////////////////////////////////////////////////////////////////////////
//button to compute and show the convex hull of the current polygon
void MainWindow::on_ConvexHull_clicked()
{
    qDebug()<<"Convex Hull clicked";
    //when clicked it will show the found convex hull in egg yolk color on top of our polygon
    //when clicked again it will be disabled and just the current our polygon will be shown
    //currently its done by clicking drawPolygon again-that will disable draw convex hull
    drawConvexHull=true;
    drawPolygon=false;
    drawTriangulation = false;
    drawSmallestCircle = false;
    update();
}
/////////////////////////////////////////////////////////////////////////////
//triangulation by ear clipping

bool MainWindow::isClockwise(const QPolygonF& polygon)
{
    qreal A = 0;
    for(int i = 0;i<polygon.size()-1; i++)
    {
        A+= (polygon[i].x()*polygon[i+1].y() - polygon[i].y()*polygon[i+1].x());
    }
    A=A/2;
    return A<0;
}

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//function that triangulates the polygon

//helper function that checks whether a vertex in a polygon is an ear tip
bool MainWindow::isEarTip(std::list<QPointF>::iterator it,
                          std::list<QPointF>& pointsList,
                          const QPolygonF& originalPolygon)
{
    //a vertex is an ear tip if the triangle made with its adjecent points
    //lays inside the polygon
    //that happens when the angle is convex
    //and when no other points are inside the triangle


    //safety check
    if (pointsList.size() < 3) return false;

    //find adjecent points
    auto it_prev = (it == pointsList.begin()) ? std::prev(pointsList.end()) : std::prev(it);
    auto it_next = std::next(it);
    if (it_next == pointsList.end()) it_next = pointsList.begin();

    QPointF Vminus = *it_prev;
    QPointF Vi     = *it;
    QPointF Vplus  = *it_next;


    //compute the cross product
    QPointF v1 = Vi - Vminus;
    QPointF v2 = Vplus - Vi;
    qreal cross = v1.x() * v2.y() - v1.y() * v2.x();

    if (cross > 0) // for ccw polygon the angle is convex if cross product is positive
    {
        //now check that there are no other points in triangle
        for (const QPointF& p : originalPolygon)
        {
            if (p != Vi && p != Vminus && p != Vplus)
            {
                if (pointInTriangle(p, Vminus, Vi, Vplus))
                    return false;
            }
        }
        return true;
    }

    return false;
}

QVector<QPolygonF> MainWindow::triangulate(QPolygonF polygon)
{
    //algorithm works only for ccw polygons
    //cw polygons need to be reversed
    QPolygonF originalPolygon = polygon;

    if (isClockwise(polygon)) {
        std::reverse(polygon.begin(), polygon.end());  // Makes it counter-clockwise
    }


//doubly linked list to easily find ajecent points
    std::list<QPointF> pointsList(polygon.begin(), polygon.end());
    //list of pints detected as ear tip verteces
    std::list<QPointF> earTips;
    //the finel list of found triangles
    QVector<QPolygonF> triangles;

    // Initial ear tip detection
    for (auto it = pointsList.begin(); it != pointsList.end(); ++it)
    {
        if (isEarTip(it, pointsList, originalPolygon))
            earTips.push_back(*it);
    }

    //loop until last triangle
    while (pointsList.size() > 3)
    {
        bool earClipped = false;
//find new ear tip in modified polygon
        for (auto it = pointsList.begin(); it != pointsList.end(); ++it)
        {
            //when ear tip is found
            if (std::find(earTips.begin(), earTips.end(), *it) != earTips.end())
            {
                //find the adjecent points
                auto it_prev = (it == pointsList.begin()) ? std::prev(pointsList.end()) : std::prev(it);
                auto it_next = std::next(it);
                if (it_next == pointsList.end()) it_next = pointsList.begin();

                QPointF Vminus = *it_prev;
                QPointF Vi     = *it;
                QPointF Vplus  = *it_next;

                // Add new found triangle(ear tip)
                triangles.append(QPolygonF{Vminus, Vi, Vplus});

                // Remove Vi ->clip the ear tip
                auto nextIt = std::next(it);
                if (nextIt == pointsList.end()) nextIt = pointsList.begin();

                //pointsList.erase(it);
                // Erase current point and get new valid iterator
                auto erased = it;
                it = pointsList.erase(erased);
                if (it == pointsList.end()) it = pointsList.begin();
                earTips.remove(Vi);

                // Update ear tip status of adjecent points ->neighbours
                for (QPointF neighbor : {Vminus, Vplus})
                {
                    auto neighborIt = std::find(pointsList.begin(), pointsList.end(), neighbor);
                    if (neighborIt != pointsList.end())
                    {
                        bool isEar = isEarTip(neighborIt, pointsList, originalPolygon);
                        bool alreadyInList = std::find(earTips.begin(), earTips.end(), neighbor) != earTips.end();

                        if (isEar && !alreadyInList)
                            earTips.push_back(neighbor);
                        else if (!isEar && alreadyInList)
                            earTips.remove(neighbor);
                    }
                }

                earClipped = true;
                break;
            }
        }

        if (!earClipped)
        {
            qWarning() << "No ear tip found — input might not be a simple polygon.";
            break;
        }
    }

    // Add the final triangle
    QPolygonF lastTriangle;
    for (const QPointF& p : pointsList)
        lastTriangle.append(p);
    triangles.append(lastTriangle);

    return triangles;
}

///////////////////////////////////////////////////////////////////////////////////
//button to compute and show triangulation
void MainWindow::on_Triangulate_clicked()
{
    qDebug()<<"Triangulation clicked";
    drawTriangulation = true;
    drawConvexHull = false;
    drawPolygon = false;
    drawSmallestCircle = false;
    update();

}
///////////////////////////////////////////////////////////////////////////////////////
//compute surface area
void MainWindow::on_SurfaceArea_clicked()
{
    qDebug()<<"Surface Area clicked";
    computeSurfaceArea = true;
    drawTriangulation = true;
    drawConvexHull = false;
    drawPolygon = true;
    drawSmallestCircle = false;
    update();
}
/////////////////////////////////////////////////////////////////////////////////////////
//function that finds smalles enclosed circle for given polygon
//it ouputs its center and radious
//it uses naive aproach when it uses already found convex hull
//and then finding the circle by trying out two to three points each
//and than outpus the smalles one found

//function that return eucklidean distance between two points
qreal MainWindow::euclideanDistance(const QPointF& x, const QPointF& y)
{
    qreal dx = pow(x.x()-y.x(),2);
    qreal dy = pow(x.y()-y.y(),2);
    return sqrt(dx+dy);
}
//function to test if all points are inside the circle
bool MainWindow::allPointsInside(const std::pair<QPointF, qreal>& circle, const QPolygonF& polygon)
{
    for(const QPointF& p : polygon)
    {
        //compute euclidean distance of p from center
        //if its not smaller than radious is outside
        if(euclideanDistance(p,circle.first)>circle.second)
        {
            return false;
        }
    }
    return true;
}
//function that will return radious and center of a circle defined by 2 points
std::pair<QPointF, qreal> MainWindow::getCircle(const QPointF& a, const QPointF& b)
{
    QPointF center = {(b.x()+a.x())/2, (b.y()+a.y())/2};
    qreal radious = euclideanDistance(a,b)/2;
    return std::pair<QPointF, qreal> {center, radious};
}
//function that will return radious and center of a circle dfiend by three points
std::pair<QPointF, qreal> MainWindow::getCircle(const QPointF& a, const QPointF& b, const QPointF& c)
{
    QPointF center;
    qreal centerX;
    qreal centerY;
    qreal radius;
    qreal x1 = a.x();
    qreal x2 = b.x();
    qreal x3 = c.x();
    qreal y1 = a.y();
    qreal y2 = b.y();
    qreal y3 = c.y();

    // Compute the determinant
    double D = 2 * (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2));
    if (std::abs(D) < 1e-8) {
        // Points are colinear; return a big circle (or ignore)
        return {{0, 0}, std::numeric_limits<qreal>::max()};
    }

    double Ux = ((x1*x1 + y1*y1)*(y2 - y3) + (x2*x2 + y2*y2)*(y3 - y1) + (x3*x3 + y3*y3)*(y1 - y2)) / D;
    double Uy = ((x1*x1 + y1*y1)*(x3 - x2) + (x2*x2 + y2*y2)*(x1 - x3) + (x3*x3 + y3*y3)*(x2 - x1)) / D;

    center= {Ux, Uy};
    radius = euclideanDistance(center, a); // all three are on the circle

    return std::pair<QPointF, qreal> {center, radius};
}

std::pair<QPointF, qreal> MainWindow::findSmallestCircle(QPolygonF convexHull)
{
    QPointF center;
    qreal radious = std::numeric_limits<qreal>::max();
    std::pair<QPointF,qreal> circle = {center,radious};
    std::pair<QPointF, qreal> newCircle;
    //test if all points are inside
    //test if its smaller than previous
    //how to test if its smaller by radious?
    //if N<=3 already solved
    //first try pairs
    //the center is the midpoint of the two points and radious is half the distance between them
    //find all pairs
    for(QPointF p : convexHull)
    {
        for(QPointF q : convexHull)
        {
            newCircle = getCircle(p,q);
            //test if the newest found circle is smaller
            if(allPointsInside(newCircle, convexHull))
            {
                if(newCircle.second <circle.second)
                {
                    circle = newCircle;
                }
            }
        }
    }
    //than try triples
    for(QPointF p : convexHull)
        for(QPointF q : convexHull)
            for(QPointF h : convexHull)
            {
                newCircle = getCircle(p,q,h);
                //test if the newest found circle is smaller
                if(allPointsInside(newCircle, convexHull))
                {
                    if(newCircle.second <circle.second)
                    {
                        circle = newCircle;
                    }
                }
            }

    return circle;
}
void MainWindow::on_SmallestCircle_clicked()
{
    qDebug() << "Smallest Circle clicked";
    drawSmallestCircle = true;
    computeSurfaceArea = false;
    drawConvexHull = true;
    drawTriangulation = false;
    drawPolygon = true;
    ui->SurfaceAreaLabel->setVisible(false);
    update();
}
//TODO::function to detect if polygon is simple and raise an error for functions needing
//simple polygons only



