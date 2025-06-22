#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QPointF>
#include <QMouseEvent>
#include <QPainter>
#include <QRandomGenerator>

//TODO: enable antialising, freeze/unfreeze button so now new points can be added
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
    //if(!points.isEmpty()) painter.drawPoint(points.last());

    //should i make the area where i draw the points smaller and exentible?
    if(drawPolygon)
    {
        QPainter painter(this);
        painter.setPen(Qt::white);
        painter.setBrush(myRed);
        //store each drawn polygon--currently there will be always just one, since all points are used to draw current polygon
        polygons.append(QPolygonF(points));
        //draw convex polygon vs draw polygon ?
        painter.drawPolygon(QPolygonF(points), Qt::OddEvenFill);
    }
    if(drawConvexHull)
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
            //painter.drawEllipse(const QPointF& center, qreal radious,qreal radious);
            painter.drawEllipse(forCircle.first, forCircle.second, forCircle.second);

        }
    }
    if(drawTriangulation)
    {
        QVector<QPolygonF> triangles=triangulate(points);
        qreal polygonArea = 0;
        for(QPolygonF triangle : triangles)
        {
            if(!computeSurfaceArea)
            {
            QPainter painter(this);
            painter.setPen(Qt::white);
            int index = QRandomGenerator::global()->bounded(0,colors.size());
            painter.setBrush(colors[index]);
            painter.drawPolygon(triangle, Qt::OddEvenFill);
            }
            else
            {
                QVector3D ViVminus = QVector3D(triangle[1] - triangle[0]);
                QVector3D ViVplus = QVector3D(triangle[2]-triangle[1]);
                QVector3D cross = QVector3D::crossProduct(ViVminus, ViVplus);
                qreal cross2D = cross.z();
                qreal triangleArea = cross2D/2;
                polygonArea +=triangleArea;

            }
        }
        if(computeSurfaceArea)
        {
            ui->SurfaceAreaLabel->setVisible(true);
            ui->SurfaceAreaLabel->setText("Surface are of the polygon: "+QString::number(polygonArea));
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
    update();

}
//////////////////////////////////////////////////////////////////////////////////////
//confex hull
//helper function for computing sorting the points from above the line and bellow

//two issues
//
//they are ordered such that the convex hull is colored wrong
//sort points in polygon in counter clockwise order
void MainWindow::sortPolygon(QPolygonF& polygon)
{
    //1)find the center point
    QPointF center(0,0);
    for(const QPointF& p : polygon)
        center +=p;
    center/polygon.size();

    //2)estimate the angle between each point and the center
    //first points will be those in 4th quadrant then 3rd, 2nd, 1st
    //if they are in the same quadrant first comes the one with "bigger" angle
    //if they are at the same angle first comes the points further away and than the closer one
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
    //qreal euclideanDistance = sqrt(std::pow(maxPoint.x()-minPoint.x(),2)
    //                             +std::pow(maxPoint.y()-minPoint.y(),2));
    //qreal distanceFromLine = std::abs(dx*p.x() + dy*p.y())/euclideanDistance;
    qreal numerator = std::abs(dy * p.x() - dx * p.y() + maxPoint.x()*minPoint.y() - maxPoint.y()*minPoint.x());
    qreal denominator = std::hypot(dx, dy); // safer than manual sqrt
    qreal distanceFromLine = numerator / denominator;
    return distanceFromLine;
}

std::pair<QPolygonF, QPolygonF> MainWindow::dividePolygonByLine(QPolygonF polygon,
                                                               QPointF minPoint,
                                                               QPointF maxPoint)
{
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
    return std::pair<QPolygonF, QPolygonF>{setUp, setDown};
}

//helper function to finde convex hull
void MainWindow::findHull(QPolygonF& polygon, QPointF& minPoint, QPointF& maxPoint, QPolygonF& convexHull)
{
    if(polygon.isEmpty()) return;
    //3)Find a point above the line with max distance from it
    //QLineF Line(minPoint,maxPoint);

    //first i compute the distance from the line for every point
    //than i find the biggest distance

    QPointF maxUpPoint;
    qreal maxDistance=0;

    for(int i = 0;i<polygon.size();i++)
    {
        QPointF p = polygon[i];
        qreal distance = distanceFromLine(minPoint,maxPoint,p);
        if(distance>=maxDistance)
        {
            maxDistance=distance;
            maxUpPoint=p;
        }
    }

    convexHull << maxUpPoint;

    //4)kick out the points inside the triangle

    QPolygonF triangle;
    triangle  << maxPoint << maxUpPoint << minPoint;
    for(QPointF p : polygon)
    {
        if(triangle.containsPoint(p,Qt::OddEvenFill))
        {//is this condition workinng?
            //maybe instead of only modifing the intial polygon
            //i should add the needed points to the final convex hull
            if(p != maxPoint && p != maxUpPoint && p != minPoint)
            polygon.removeAll(p);
        }
    }

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
//TODO visualize how is the hull being find point by point-is it too quick to see?
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
bool MainWindow::isEarTip(QPointF& Vi, QPolygonF& polygon)
{
    //find adject points
    std::pair<QPointF, QPointF> pair = findAdjecentPoints(Vi, polygon);
    QPointF Vminus = pair.first;
    QPointF Vplus = pair.second;

    //check if Vi has convex angle in polygon
    QPointF v1 = Vi-Vminus;
    QPointF v2 = Vplus - Vi;
    qreal cross = v1.x()*v2.x() - v1.y()*v2.y();
    if(cross<0)
    {
        //check if theres no other point in the triangle
        bool noVertices=true;
        QPolygonF triangle = {Vminus,Vi,Vplus};
        for(QPointF p : polygon)
        {
            if(triangle.containsPoint(p, Qt::OddEvenFill))
            {
                if(p!=Vi && p!=Vminus && p != Vplus)
                {
                    noVertices=false;
                    return noVertices;
                }
            }
        }
    }
    return false;
}
std::pair<QPointF,QPointF> MainWindow::findAdjecentPoints(QPointF Vi, QPolygonF polygon)
{
    int i = polygon.indexOf(Vi);
    QPointF Vminus;
    if(i==0)
    {
        Vminus = polygon[polygon.size()-1];
    }
    else
    {
        Vminus = polygon[i-1];
    }
    QPointF Vplus;
    if(i==polygon.size()-1)
    {
        Vplus = polygon[0];
    }
    else
    {
        Vplus = polygon[i+1];
    }
    return std::pair<QPointF, QPointF> {Vminus, Vplus};
}
//main funtion
QVector<QPolygonF> MainWindow::triangulate(QPolygonF polygon)
{
    QVector<QPolygonF> triangles;
    //1)initialize the ear tip status of each vertex
    //ear tip=for any vertex Vi and corresponding triangle <Vi-1,Vi,Vi+1>
    //if line segment (Vi-1,Vi+1) is a diagonal of the polygon the the triangle is and ear
    //and Vi is an ear tip
    QVector<QPointF> earTips;
    //QVector3D line = QVector3D(Vi+1 - Vi-1);
    //is line inside of polygon or outsied of polygon?
    //no need to check for the whole line?
    //if the angle of Vi is convex and if theres no other point inside the triangle
    //containpoints function
    for(int i = 0; i < polygon.size(); i++)
    {
        QPointF Vi = polygon[i];
        //check if Vi is an ear tip
        if(isEarTip(Vi,polygon))
        {
            earTips << Vi;
        }

    }

    //after initializetion we start to cut the ears--after each cut the ear tip status will
    //be renewed for the vertices of that paricular line(the adjecent vertices to Vi)
    //2)while n>3 do
    while(polygon.size()>3)
    {
        for(QPointF p : polygon)
        {
    //2.1)locate an ear tip Vi
            if(earTips.contains(p))
            {
    //2.2)delete Vi from polygon and make a new triangle to be put to triangles
                std::pair<QPointF,QPointF> pair = findAdjecentPoints(p, polygon);
                QPolygonF newTriangle = {pair.first, p, pair.second};
                triangles.append(newTriangle);
                polygon.removeAll(p);
    //2.3)update the ear status of the adject vertices Vi-1,Vi+1
                if(isEarTip(pair.first, polygon))
                {
                    if(!earTips.contains(pair.first))
                        earTips.append(pair.first);
                }
                else
                {
                    if(earTips.contains(pair.first))
                        earTips.removeAll(pair.first);
                }
                if(isEarTip(pair.second,polygon))
                {
                    if(!earTips.contains(pair.second))
                        earTips.append(pair.second);
                }
                else
                {
                    if(earTips.contains(pair.second))
                        earTips.removeAll(pair.second);
                }
            }
        }
    }
    triangles.append(polygon);

    return triangles;
}
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
std::pair<QPointF, qreal> findSmallestCircle(QPolygonF convexHull)
{
    QPointF center;
    qreal radious;
    //test if all points are inside
    //test if its smaller than previous
    //how to test if its smaller by radious?
    //if N<=3 already solved
    //first try pairs
    //the center is the midpoint of the two points and radious is half the distance between them
    //find all pairs
    //than try triples

    return std::pair<QPointF,qreal> {center,radious};
}
void MainWindow::on_SmallestCircle_clicked()
{
    qDebug() << "Smallest Circle clicked";
    drawSmallestCircle = true;
    computeSurfaceArea = false;
    drawConvexHull = true;
    drawTriangulation = false;
    drawPolygon = true;
    update();
}
//TODO::function to detect if polygon is simple and raise an error for functions needing
//simple polygons only



