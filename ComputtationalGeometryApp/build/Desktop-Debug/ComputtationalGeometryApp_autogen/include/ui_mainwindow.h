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
        menuRestart->setTitle(QCoreApplication::translate("MainWindow", "Menu", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
