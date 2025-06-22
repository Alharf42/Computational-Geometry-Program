/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionRestart;
    QWidget *centralwidget;
    QPushButton *Clear;
    QPushButton *MakePolygon;
    QPushButton *ConvexHull;
    QPushButton *Triangulate;
    QPushButton *SurfaceArea;
    QLabel *SurfaceAreaLabel;
    QPushButton *SmallestCircle;
    QMenuBar *menubar;
    QMenu *menuRestart;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 600);
        actionRestart = new QAction(MainWindow);
        actionRestart->setObjectName("actionRestart");
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        Clear = new QPushButton(centralwidget);
        Clear->setObjectName("Clear");
        Clear->setGeometry(QRect(0, 0, 80, 24));
        MakePolygon = new QPushButton(centralwidget);
        MakePolygon->setObjectName("MakePolygon");
        MakePolygon->setGeometry(QRect(80, 0, 91, 24));
        ConvexHull = new QPushButton(centralwidget);
        ConvexHull->setObjectName("ConvexHull");
        ConvexHull->setGeometry(QRect(170, 0, 80, 24));
        Triangulate = new QPushButton(centralwidget);
        Triangulate->setObjectName("Triangulate");
        Triangulate->setGeometry(QRect(250, 0, 80, 24));
        SurfaceArea = new QPushButton(centralwidget);
        SurfaceArea->setObjectName("SurfaceArea");
        SurfaceArea->setGeometry(QRect(330, 0, 80, 24));
        SurfaceAreaLabel = new QLabel(centralwidget);
        SurfaceAreaLabel->setObjectName("SurfaceAreaLabel");
        SurfaceAreaLabel->setGeometry(QRect(0, 30, 441, 16));
        SurfaceAreaLabel->setStyleSheet(QString::fromUtf8("background-color: transparent;\n"
"color: red;"));
        SmallestCircle = new QPushButton(centralwidget);
        SmallestCircle->setObjectName("SmallestCircle");
        SmallestCircle->setGeometry(QRect(410, 0, 91, 24));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 21));
        menuRestart = new QMenu(menubar);
        menuRestart->setObjectName("menuRestart");
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuRestart->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionRestart->setText(QCoreApplication::translate("MainWindow", "Restart", nullptr));
        Clear->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        MakePolygon->setText(QCoreApplication::translate("MainWindow", "Make Polygon", nullptr));
        ConvexHull->setText(QCoreApplication::translate("MainWindow", "Convex Hull", nullptr));
        Triangulate->setText(QCoreApplication::translate("MainWindow", "Triangulate", nullptr));
        SurfaceArea->setText(QCoreApplication::translate("MainWindow", "Surface Area", nullptr));
        SurfaceAreaLabel->setText(QCoreApplication::translate("MainWindow", "Surface area of selected polygon: ", nullptr));
        SmallestCircle->setText(QCoreApplication::translate("MainWindow", "Smallest Circle", nullptr));
        menuRestart->setTitle(QCoreApplication::translate("MainWindow", "Menu", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
