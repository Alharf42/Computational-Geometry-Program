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



protected:
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void on_Clear_clicked();
    void on_MakePolygon_clicked();
    void on_ConvexHull_clicked();


private:
    Ui::MainWindow *ui;

    //QGraphicsScene *scene;
    //a container to store clicked points
    QVector<QPointF> points;
    //a variable that keeps information wheather new polygon should be drawn
    bool drawPolygon = false;
    bool drawConvexHull=false;
    //container for storing polygons
    QVector<QPolygonF> polygons;

    QColor myRed = QColor(168,14,3);
    QColor myYellow = QColor(224, 177, 65);
    //helper function for finding convex hull
    void findHull(QPolygonF& polygon, QPointF& min, QPointF& max, QPolygonF& convexHull);
    std::pair<QPolygonF, QPolygonF> dividePolygonByLine(QPolygonF polygon, QPointF minPoint,QPointF maxPoint);

};
#endif // MAINWINDOW_H
