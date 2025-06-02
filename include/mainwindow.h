#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector> 
#include <force.h>

namespace Ui {
    class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

// dodanie slotu do uruchamiania symulacji z UI
public slots:
    void runSimulationAndPlot(); 

private:
    Ui::MainWindow *ui;

    double t_max;
    double dt;
    double x0;
    double v0;
    double k;
    double m;
    double mi;

    std::vector<double> t;
    std::vector<double> x_rk4;
    std::vector<double> v_rk4;
    std::vector<double> x_euler;
    std::vector<double> v_euler;    

    void solveRK4(double initial_a, double initial_b, double step,
    double (*func_a)(double, double, double, Force),
    double (*func_b)(double, double, double, Force),
    std::vector<double>& result_a, std::vector<double>& result_b,
    Force input_force);

    void solveEuler(double initial_a, double initial_b, double step,
    double (*func_a)(double, double, double, Force),
    double (*func_b)(double, double, double, Force),
    std::vector<double>& result_a, std::vector<double>& result_b,
    Force input_force);

    void plotResults(); 
};

double v_dt(double time, double x, double v, Force input); // 'time' zamiast 't' dla jasności
double x_dt(double time, double x, double v, Force input);

#endif // MAINWINDOW_H