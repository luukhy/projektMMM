#include "mainwindow.h"
#include <QApplication> 
#include <iostream>      

int main(int argc, char *argv[])
{
    std::cout << "Application starting..." << std::endl;
    QApplication a(argc, argv);
    
    MainWindow w;
    std::cout << "MainWindow instance created." << std::endl;
    
    w.show();
    std::cout << "MainWindow shown." << std::endl;

    return a.exec();
}