#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QGraphicsScene>
#include <QPointF>
#include <QVector>


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    QPolygonF findConvexHull(QPolygonF polygon);//quickHull algorithm
    QVector<QPolygonF> triangulate(QPolygonF polygon);
    std::pair<QPointF, qreal> findSmallestCircle(QPolygonF polygon);





protected:
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void on_Clear_clicked();
    void on_MakePolygon_clicked();
    void on_ConvexHull_clicked();
    void on_Triangulate_clicked();
    void on_SurfaceArea_clicked();
    void on_SmallestCircle_clicked();


private:
    Ui::MainWindow *ui;

    //a container to store clicked points
    QVector<QPointF> points;

    //a variable that keeps information wheather new polygon should be drawn
    bool drawPolygon = false;
    bool drawConvexHull=false;
    bool drawTriangulation=false;
    bool computeSurfaceArea = false;
    bool drawSmallestCircle = false;

    //container for storing polygons
    QVector<QPolygonF> polygons;

    //my colors
    QColor myRed = QColor(168,14,3);
    QColor myYellow = QColor(224, 177, 65);

    //helper function for finding convex hull
    bool pointInTriangle(QPointF p, QPointF a, QPointF b, QPointF c);
    qreal distanceFromLine(QPointF& minPoint, QPointF& maxPoint, QPointF& p);
    void findHull(QPolygonF& polygon, QPointF& min, QPointF& max, QPolygonF& convexHull);
    std::pair<QPolygonF, QPolygonF> dividePolygonByLine(QPolygonF polygon, QPointF minPoint,QPointF maxPoint);
    void sortPolygon(QPolygonF& polygon);

    //helper functions for finding triangulation
    bool isClockwise(const QPolygonF& polygon);
    //std::pair<QPointF,QPointF> findAdjecentPoints(QPointF Vi, QPolygonF polygon, int i);
    //bool isEarTip(QPointF& Vi, QPolygonF& polygon, int i, const QPolygonF& originalPolygon);
    //bool isEarTip(QPointF Vi, std::list<QPointF> listsPoints , const QPolygonF originalPolygon);
    bool isEarTip(std::list<QPointF>::iterator it, std::list<QPointF>& pointsList, const QPolygonF& originalPolygon);
    //helper functions for finding smallest circle
    qreal euclideanDistance(const QPointF& x, const QPointF& y);
    bool allPointsInside(const std::pair<QPointF, qreal>& circle, const QPolygonF& polygon);
    std::pair<QPointF, qreal> getCircle(const QPointF& a, const QPointF& b);
    std::pair<QPointF, qreal> getCircle(const QPointF& a, const QPointF& b, const QPointF& c);

};
//iteratable of qt predefined colors
static const QVector<Qt::GlobalColor> colors = {
    Qt::black, Qt::white, Qt::red, Qt::green, Qt::blue,
    Qt::cyan, Qt::magenta, Qt::yellow,
    Qt::darkRed, Qt::darkGreen, Qt::darkBlue,
    Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow,
    Qt::gray, Qt::darkGray, Qt::lightGray
};

#endif // MAINWINDOW_H
