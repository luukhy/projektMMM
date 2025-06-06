#include "mainwindow.h"
#include "ui_mainwindow.h" 
#include <matplot/matplot.h>
#include <iostream>
#include <QPushButton>
#include <force.h>

#ifndef PI
#define PI 3.14159265
#endif



double k_global = 2.0;
double m_global = 1.0;

MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent),
ui(new Ui::MainWindow),
m_input_force(new Force)

{
    ui->setupUi(this);
    ui->customPlot->addGraph();
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->customPlot_vel->addGraph(); //RK4
    ui->customPlot_vel->addGraph(); //Euler
    ui->customPlot_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    
    ui->customPlot_disp->addGraph();
    ui->customPlot_disp->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    

    // GUI values init 
    ui->k1SpinBox->setValue(1.0);
    ui->k2SpinBox->setValue(1.0);  
    ui->massSpinBox->setValue(1.0);  
    // ui->miSpinBox->setValue(1.0);  
    // ui->tmaxSpinBox->setValue(10.0);
    // ui->dtSpinBox->setValue(0.01);
    // ui->x0SpinBox->setValue(0.01);
    // ui->v0SpinBox->setValue(0.01);
    // ui->v0SpinBox->setValue(0.01);
    // ui->typeDropBox->setVal(cos tam cos tam)


    // simulation variables init
    m_t_max = 6.0;
    m_dt = 0.01;
    size_t nupoints = static_cast<size_t>(m_t_max / m_dt) + 1;
    m_t = matplot::linspace(0, m_t_max, nupoints);

    m_x0 = 10.0;
    m_v0 = 0.0;
    m_k1 = ui->k1SpinBox->value();
    m_k2 = 3.0;
    m_mass = 1.0;
    m_mi = 0.01;


    // input force variables init
    double period = 3.0;
    double amplitude = 0.0;
    double phase = 0.0;
    double offset = 0.0;
    double duty_cycle = 0.5;
    m_input_force->updateForce(ForceType::SINE, m_t, period, amplitude, phase, offset, duty_cycle);

    connect(ui->runButton, &QPushButton::clicked, this, &MainWindow::runSimulationAndPlot);
    connect(ui->k1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, &MainWindow::updateInputGraph);

    connect(ui->k2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::updateInputGraph);
    
    
    // m_timer = new QTimer(this);
    // connect(m_timer, &QTimer::timeout, this, &MainWindow::updateInputGraph);
    // m_timer->start(30); // update every 30 ms


    connect(ui->customPlot, &QCustomPlot::mouseMove, this, [this](QMouseEvent *event){
    double x = ui->customPlot->xAxis->pixelToCoord(event->pos().x());
    double y = ui->customPlot->yAxis->pixelToCoord(event->pos().y());
    statusBar()->showMessage(QString("x=%1, y=%2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
    });

    connect(ui->customPlot_disp, &QCustomPlot::mouseMove, this, [this](QMouseEvent *event){
    double x = ui->customPlot->xAxis->pixelToCoord(event->pos().x());
    double y = ui->customPlot->yAxis->pixelToCoord(event->pos().y());
    statusBar()->showMessage(QString("x=%1, y=%2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
    });

    connect(ui->customPlot_vel, &QCustomPlot::mouseMove, this, [this](QMouseEvent *event){
    double x = ui->customPlot->xAxis->pixelToCoord(event->pos().x());
    double y = ui->customPlot->yAxis->pixelToCoord(event->pos().y());
    statusBar()->showMessage(QString("x=%1, y=%2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
    });

}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_timer;
    delete m_input_force;
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
        for (int i = 0; i < m_t.size(); i++)
        {
            k1a = func_a(m_t[i], result_a[i], result_b[i], input_force);
            k1b = func_b(m_t[i], result_a[i], result_b[i], input_force);
            
            k2a = func_a(m_t[i] + 0.5*step, result_a[i] + 0.5*step*k1a, result_b[i] + 0.5*k1b*step, input_force);
            k2b = func_b(m_t[i] + 0.5*step, result_a[i] + 0.5*step*k1a, result_b[i] + 0.5*k1b*step, input_force);
            
            k3a = func_a(m_t[i] + 0.5*step, result_a[i] + 0.5*step*k2a, result_b[i] + 0.5*step*k2b, input_force);
            k3b = func_b(m_t[i] + 0.5*step, result_a[i] + 0.5*step*k2a, result_b[i] + 0.5*step*k2b, input_force);
            
            k4a = func_a(m_t[i] + step, result_a[i] + step*k3a, result_b[i] + step*k3b, input_force);
            k4b = func_b(m_t[i] + step, result_a[i] + step*k3a, result_b[i] + step*k3b, input_force);
            
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
        
        for (size_t i = 0; i < m_t.size() - 1; ++i)
        {
            double da_dt = func_a(m_t[i], result_a[i], result_b[i], input_force);
            double db_dt = func_b(m_t[i], result_a[i], result_b[i], input_force);
            
            double next_a = result_a[i] + step * da_dt;
            double next_b = result_b[i] + step * db_dt;
            
            result_a.push_back(next_a);
            result_b.push_back(next_b);
        }
    }

void MainWindow::runSimulationAndPlot()
{
    std::cout << "runSimulationAndPlot called." << std::endl;
    size_t nupoints = static_cast<size_t>(m_t_max / m_dt) + 1;
    m_t = matplot::linspace(0, m_t_max, nupoints);

    double period = 3.0;
    double amplitude = 0.0;
    double phase = 0.0;
    double offset = 0.0;
    double duty_cycle = 0.5;
    
    // double freq =  sqrt(m_k1/m_mass) / (2*PI);
    // period = 1.0 / freq;
    
    Force input_force(ForceType::TRIANGLE, m_t, period, amplitude, phase, offset, duty_cycle);
    solveRK4(m_x0, m_v0, m_dt, x_dt, v_dt, m_x_rk4, m_v_rk4, *m_input_force);
    solveEuler(m_x0, m_v0, m_dt, x_dt, v_dt, m_x_euler, m_v_euler, input_force);
    
    // std::cout << "RK4 x_rk4 size: " << m_x_rk4.size() << std::endl;
    // std::cout << "Euler x_euler size: " << m_x_euler.size() << std::endl;


    updateQtPlots();
    //plotResultsMatplot();
}

void MainWindow::updateQtPlots()
{
    double y_max_val, y_min_val;
    // velocity plots
    QVector<double> x(m_v_rk4.size()), y(m_v_rk4.size());
    y_max_val = m_v_rk4[0];
    y_min_val = m_v_rk4[0];
    for (int i = 0; i < m_t.size(); i++) 
    {
        x[i] = m_t[i];
        y[i] = m_v_rk4[i];

        if (y[i] > y_max_val)
            y_max_val = y[i];
        if (y[i] < y_min_val)
            y_min_val = y[i];
    }
    ui->customPlot_vel->xAxis->setRange(0, m_t[m_t.size() - 1]);
    ui->customPlot_vel->yAxis->setRange(y_min_val, y_max_val);
    ui->customPlot_vel->graph(0)->setData(x, y);
    ui->customPlot_vel->graph(0)->setPen(QPen(Qt::red));

    
    for (int i = 0; i < m_t.size(); i++) 
    {
        x[i] = m_t[i];
        y[i] = m_v_euler[i];

        if (y[i] > y_max_val)
            y_max_val = y[i];
        if (y[i] < y_min_val)
            y_min_val = y[i];
    }
    ui->customPlot_vel->xAxis->setRange(0, m_t[m_t.size() - 1]);
    ui->customPlot_vel->yAxis->setRange(y_min_val, y_max_val);
    ui->customPlot_vel->graph(1)->setData(x, y);
    ui->customPlot_vel->graph(1)->setPen(QPen(Qt::blue, 1, Qt::DashLine));
    
    ui->customPlot_vel->legend->setVisible(true);
    ui->customPlot_vel->graph(0)->setName("RK4");
    ui->customPlot_vel->graph(1)->setName("Euler");

    
    ui->customPlot_vel->replot();
}

void MainWindow::plotResultsMatplot()
{
    // std::cout << "plotResults called." << std::endl;
    // if (t.empty() || x_rk4.empty() || x_euler.empty() || v_rk4.empty() || v_euler.empty()) {
    //     std::cout << "No data to plot." << std::endl;
    //     return;
    // }

    // #ifdef __linux__
    //     setenv("GNUTERM", "x11", 1);
    // #endif

    // matplot::figure(true);
    // matplot::subplot(1, 2, 1);
    // matplot::hold(matplot::on);
    // matplot::plot(t, x_rk4)->line_width(1).display_name("RK4");
    // matplot::plot(t, x_euler)->line_width(1).display_name("Euler");
    // matplot::xlabel("time");
    // matplot::title("Displacement");
    // matplot::legend();

    // matplot::subplot(1, 2, 2);
    // matplot::hold(matplot::on);
    // matplot::plot(t, v_rk4)->line_width(1).display_name("RK4");
    // matplot::plot(t, v_euler)->line_width(1).display_name("Euler");
    // matplot::xlabel("time");
    // matplot::title("Velocity");
    // matplot::legend();
    
    // matplot::show(); 
}

void MainWindow::updateInputGraph()
{   
    
    // for (int i = 0; i < x_rk4.size(); i++) 
    // {
    //     x[i] = this->t[i];
    //     y[i] = this->x_rk4[i];

    //     if (y[i] > y_max_val)
    //         y_max_val = y[i];
    //     if (y[i] < y_min_val)
    //         y_min_val = y[i];
    // }
    const int N = m_t.size();
    QVector<double> x(N), y(N);
    double amplitude = ui->k1SpinBox->value();
    double period = ui->k2SpinBox->value(); //ui->periodSpinBoc->value();
    double phase = 0.0; //ui->phaseSpinBoc->value();
    double offset = 0.0; //ui->offsetSpinBoc->value();
    double duty_cycle = 0.5; //ui->dutyCycleSpinBoc->value();

    m_input_force->updateForce(ForceType::SINE, m_t, period, amplitude, phase, offset, duty_cycle);
    for (int i = 0; i < N; ++i) {
        x[i] = m_t[i];
        y[i] = m_input_force->getValues()[i];
    }

    ui->customPlot->graph(0)->setData(x, y);
    ui->customPlot->replot();

}

double v_dt(double time, double x, double v, Force input)
{
    return -k_global/m_global * x + input.atTime(time);
}

double x_dt(double time, double x, double v, Force input)
{
    return v;
}
