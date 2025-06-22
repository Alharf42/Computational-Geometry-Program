#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QPointF>
#include <QMouseEvent>
#include <QPainter>

//TODO: enable antialising, freeze/unfreeze button so now new points can be added
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    // Create a QGraphicsScene
    //QGraphicsScene scene = new QGraphicsScene(this);

    // Set the scene to the QGraphicsView
    // ui->graphicsView->setScene(scene);
    // ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    connect(ui->Clear, &QPushButton::clicked, this, &MainWindow::on_Clear_clicked);
    connect(ui->MakePolygon, &QPushButton::clicked,this, &MainWindow::on_MakePolygon_clicked);
    connect(ui->ConvexHull, &QPushButton::clicked,this,&MainWindow::on_ConvexHull_clicked);
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
        QPainter painter(this);
        painter.setPen(Qt::white);
        painter.setBrush(myYellow);
        painter.drawPolygon(findConvexHull(QPolygonF(points)), Qt::OddEvenFill);
    }

}

//restart button
void MainWindow::on_Clear_clicked()
{
    qDebug()<<"clear button clicked";
    points.clear();
    drawPolygon=false;
    drawConvexHull=false;
    update();
}
//button draw polygon
void MainWindow::on_MakePolygon_clicked()
{
    qDebug()<<"Make Polygon buton clicked";

    drawPolygon =true;
    drawConvexHull=false;
    update();

}
//////////////////////////////////////////////////////////////////////////////////////
//confex hull
//helper function for computing sorting the points from above the line and bellow

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
    QVector<qreal> distanceVector;
    for(QPointF p : polygon)
    {
        qreal dx = maxPoint.x() - minPoint.x();
        qreal dy = maxPoint.y() - minPoint.y();
        qreal euclideanDistance = sqrt(std::pow(maxPoint.x()-minPoint.x(),2)
                                       +std::pow(maxPoint.y()-minPoint.y(),2));
        qreal distanceFromLine = std::abs(dx*p.x() + dy*p.y())/euclideanDistance;


        distanceVector.push_back(distanceFromLine);

    }
    //than i find the biggest distance
    //TODO: do it both in one cycle

    QPointF maxUpPoint = polygon[0];

    for(int i = 0; i< distanceVector.size();i++)
    {
        if(i !=0)
        {
            if(distanceVector[i]>=distanceVector[i-1]
                & distanceVector[i]> distanceVector[0])
            {
                maxUpPoint = polygon[i];
            }
        }
    }

    convexHull << maxUpPoint;

    //4)kick out the points inside the triangle

    QPolygonF triangle;
    triangle  << maxPoint << maxUpPoint << minPoint;
    for(QPointF p : polygon)
    {
        //if(polygon.containsPoint(p,Qt::OddEvenFill))
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
    QPolygonF convexHull = polygon;
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
    update();
}





