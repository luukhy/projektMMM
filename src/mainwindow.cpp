#include "mainwindow.h"
#include "ui_mainwindow.h" 
#include <matplot/matplot.h>
#include <iostream>
#include <QPushButton>

#ifndef PI
#define PI 3.14159265
#endif


double k_global = 30.0;
double m_global = 1.0;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    
    t_max = 2.0;
    dt = 0.01;
    x0 = 4.0;
    v0 = 0.0;
    k = 3.0;
    m = 1.0;
    mi = 0.01;

    if (ui->runButton) 
    {
        connect(ui->runButton, &QPushButton::clicked, this, &MainWindow::runSimulationAndPlot);
    }
    if (ui->kSpinBox) { // sprawdzenie czy kSpinBox istnieje
        ui->kSpinBox->setValue(k_global);
        ui->kSpinBox->setValue(k); 
        
    }
    if (ui->mSpinBox) {
        ui->mSpinBox->setValue(m_global); 
        ui->mSpinBox->setValue(m); 
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}




void MainWindow::solveRK4(double initial_a, double initial_b, double step,
    double (*func_a)(double, double, double, Force),
    double (*func_b)(double, double, double, Force),
    std::vector<double>& result_a, std::vector<double>& result_b,
    Force input_force)
    {
        result_a.push_back(initial_a);
        result_b.push_back(initial_b);
        
        double k1a, k1b, k2a, k2b, k3a, k3b, k4a, k4b;
        for (int i = 0; i < t.size(); i++)
        {
            k1a = func_a(t[i], result_a[i], result_b[i], input_force);
            k1b = func_b(t[i], result_a[i], result_b[i], input_force);
            
            k2a = func_a(t[i] + 0.5*step, result_a[i] + 0.5*step*k1a, result_b[i] + 0.5*k1b*step, input_force);
            k2b = func_b(t[i] + 0.5*step, result_a[i] + 0.5*step*k1a, result_b[i] + 0.5*k1b*step, input_force);
            
            k3a = func_a(t[i] + 0.5*step, result_a[i] + 0.5*step*k2a, result_b[i] + 0.5*step*k2b, input_force);
            k3b = func_b(t[i] + 0.5*step, result_a[i] + 0.5*step*k2a, result_b[i] + 0.5*step*k2b, input_force);
            
            k4a = func_a(t[i] + step, result_a[i] + step*k3a, result_b[i] + step*k3b, input_force);
            k4b = func_b(t[i] + step, result_a[i] + step*k3a, result_b[i] + step*k3b, input_force);
            
            result_a.push_back(result_a[i] + 1.0/6.0*step*(k1a + 2*k2a + 2*k3a + k4a));
            result_b.push_back(result_b[i] + 1.0/6.0*step*(k1b + 2*k2b + 2*k3b + k4b));
        }
    }

void MainWindow::solveEuler(double initial_a, double initial_b, double step,
    double (*func_a)(double, double, double, Force),
    double (*func_b)(double, double, double, Force),
    std::vector<double>& result_a, std::vector<double>& result_b,
    Force input_force)
    {
        result_a.clear();
        result_b.clear();
        
        result_a.push_back(initial_a);
        result_b.push_back(initial_b);
        
        for (size_t i = 0; i < t.size() - 1; ++i)
        {
            double da_dt = func_a(t[i], result_a[i], result_b[i], input_force);
            double db_dt = func_b(t[i], result_a[i], result_b[i], input_force);
            
            double next_a = result_a[i] + step * da_dt;
            double next_b = result_b[i] + step * db_dt;
            
            result_a.push_back(next_a);
            result_b.push_back(next_b);
        }
    }

void MainWindow::runSimulationAndPlot()
{
    std::cout << "runSimulationAndPlot called." << std::endl;
    size_t nupoints = static_cast<size_t>(t_max / dt) + 1;
    t = matplot::linspace(0, t_max, nupoints);

    double period = 3.0;
    double amplitude = 4.0;
    double phase = 0.0;
    double offset = 0.0;
    double duty_cycle = 0.5;
    
    double freq =  sqrt(k/m) / (2*PI);
    period = 1.0 / freq;
    Force input_force(ForceType::TRIANGLE, t, period, amplitude, phase, offset, duty_cycle);

    solveRK4(x0, v0, dt, x_dt, v_dt, x_rk4, v_rk4, input_force);
    solveEuler(x0, v0, dt, x_dt, v_dt, x_euler, v_euler, input_force);
    
    std::cout << "RK4 x_rk4 size: " << x_rk4.size() << std::endl;
    std::cout << "Euler x_euler size: " << x_euler.size() << std::endl;

    plotResults();
}

void MainWindow::plotResults()
{
    std::cout << "plotResults called." << std::endl;
    if (t.empty() || x_rk4.empty() || x_euler.empty() || v_rk4.empty() || v_euler.empty()) {
        std::cout << "No data to plot." << std::endl;
        return;
    }

    #ifdef __linux__
        setenv("GNUTERM", "x11", 1);
    #endif

    matplot::figure(true);
    matplot::subplot(1, 2, 1);
    matplot::hold(matplot::on);
    matplot::plot(t, x_rk4)->line_width(1).display_name("RK4");
    matplot::plot(t, x_euler)->line_width(1).display_name("Euler");
    matplot::xlabel("time");
    matplot::title("Displacement");
    matplot::legend();

    matplot::subplot(1, 2, 2);
    matplot::hold(matplot::on);
    matplot::plot(t, v_rk4)->line_width(1).display_name("RK4");
    matplot::plot(t, v_euler)->line_width(1).display_name("Euler");
    matplot::xlabel("time");
    matplot::title("Velocity");
    matplot::legend();
    
    matplot::show(); 
}


double v_dt(double time, double x, double v, Force input)
{
    return -k_global/m_global * x + input.atTime(time);
}

double x_dt(double time, double x, double v, Force input)
{
    return v;
}
