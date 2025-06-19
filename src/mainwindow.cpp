#include "mainwindow.h"
#include "ui_mainwindow.h" 
#include <matplot/matplot.h>
#include <iostream>
#include <QPushButton>
#include <force.h>
#include <QPixmap>
#include <filesystem>
#include <sstream>

#ifndef PI
#define PI 3.14159265
#endif



double k1_global = 1.0;
double k2_global = 1.0;
double m_global = 1.0;
double mi_global = 1.0;

MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent),
ui(new Ui::MainWindow),
m_input_force(new Force)

{
    ui->setupUi(this);
    ui->customPlot->addGraph(); // input force
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    
    ui->customPlot_vel->addGraph(); //RK4 (0)
    ui->customPlot_vel->addGraph(); //Euler (1)
    ui->customPlot_vel->addGraph(); //input (2)
    ui->customPlot_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    
    ui->customPlot_disp->addGraph(); //RK4 (0)
    ui->customPlot_disp->addGraph(); //Euler (1)
    ui->customPlot_disp->addGraph(); //input (2)
    ui->customPlot_disp->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->Imagelabel->setScaledContents(true);
 
    // std::ostringstream oss;
    // oss << RESOURCE_PATH << "/model.png";
    
    QPixmap pixmap(RESOURCE_PATH); 
    ui->Imagelabel->setPixmap(pixmap);
    if (pixmap.isNull()) {
        qDebug() << "Failed to load image!";
    } else {
        ui->Imagelabel->setPixmap(pixmap);
    }

    // GUI values init 
    ui->k1SpinBox->setValue(1.0);
    ui->k2SpinBox->setValue(1.0);  
    ui->massSpinBox->setValue(1.0);  
    ui->miSpinBox->setValue(1.0);  
    ui->t_maxSpinBox->setValue(10.0);
    ui->d_tSpinBox->setValue(0.01);
    ui->x_0SpinBox->setValue(0.0);
    ui->v_0SpinBox->setValue(0.0);

    ui->periodSpinBox->setValue(1.0);
    ui->ampSpinBox->setValue(1.0);
    ui->phaseSpinBox->setValue(0.0);
    ui->offsetSpinBox->setValue(0.0);
    ui->dutySpinBox->setValue(0.5);

    ui->dropBox_signal->setCurrentIndex(0);


    // simulation variables init
    readAndSetSimVariables();

    // input-force variables init
    readAndSetForceVariables();



    connect(ui->runButton, &QPushButton::clicked, this, &MainWindow::runSimulationAndPlot);
    connect(ui->dropBox_signal, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, &MainWindow::updateInputGraph);
    QList<QDoubleSpinBox*> spinBoxes = {ui->t_maxSpinBox, ui->d_tSpinBox, ui->periodSpinBox,
                            ui->ampSpinBox, ui->phaseSpinBox, ui->offsetSpinBox, ui->dutySpinBox};
    for (QDoubleSpinBox* spinBox : spinBoxes) 
    {
        connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &MainWindow::updateInputGraph);
    }

    updateInputGraph();

}

MainWindow::~MainWindow()
{
    delete m_input_force;
    delete ui;
}




void MainWindow::solveRK4(double initial_a, double initial_b, double step,
    double (*func_a)(double, double, double, Force),
    double (*func_b)(double, double, double, Force),
    std::vector<double>& result_a, std::vector<double>& result_b,
    Force input_force)
    {
        result_a.clear();
        result_b.clear();

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
    readAndSetSimVariables();
    readAndSetForceVariables();
    
    solveRK4(m_x0, m_v0, m_dt, x_dt, v_dt, m_x_rk4, m_v_rk4, *m_input_force);
    solveEuler(m_x0, m_v0, m_dt, x_dt, v_dt, m_x_euler, m_v_euler, *m_input_force);
    
    // std::cout << "RK4 x_rk4 size: " << m_x_rk4.size() << std::endl;
    // std::cout << "Euler x_euler size: " << m_x_euler.size() << std::endl;


    updateQtPlots();
    //plotResultsMatplot();
}

void MainWindow::updateQtPlots()
{
    double y_max, y_min;
    // velocity plots
    QVector<double> x, y;
    y_max = m_v_rk4[0];
    y_min = m_v_rk4[0];
    for (int i = 0; i < m_t.size()-1; i++) 
    {
        x.append(m_t[i]);
        y.append(m_v_rk4[i]);

        if (y[i] > y_max)
            y_max = y[i];
        if (y[i] < y_min)
            y_min = y[i];
    }
    ui->customPlot_vel->xAxis->setRange(0, m_t[m_t.size() - 1]);
    if (y_max - y_min <= 0.1)
        ui->customPlot_vel->yAxis->setRange(floor(y_min - 3), floor(y_max + 3));
    else
       ui->customPlot_vel->yAxis->setRange(y_min, y_max);
    ui->customPlot_vel->graph(0)->setData(x, y);
    ui->customPlot_vel->graph(0)->setPen(QPen(Qt::red));
    x.clear();
    y.clear();

    for (int i = 0; i < m_t.size()-1; i++) 
    {
        x.append(m_t[i]);
        y.append(m_v_euler[i]);

        if (y[i] > y_max)
            y_max = y[i];
        if (y[i] < y_min)
            y_min = y[i];
    }
    ui->customPlot_vel->xAxis->setRange(0, m_t[m_t.size() - 1]);
    if (y_max - y_min <= 0.1)
        ui->customPlot_vel->yAxis->setRange(floor(y_min - 3), floor(y_max + 3));
    else
       ui->customPlot_vel->yAxis->setRange(y_min, y_max);    
    ui->customPlot_vel->graph(1)->setData(x, y);
    ui->customPlot_vel->graph(1)->setPen(QPen(Qt::blue, 1, Qt::DashLine));
    x.clear();
    y.clear();

    // displacement plots 

    y_max = m_x_rk4[0];
    y_min = m_x_rk4[0];
    for (int i = 0; i < m_t.size()-1; i++) 
    {
        x.append(m_t[i]);
        y.append(m_x_rk4[i]);

        if (y[i] > y_max)
            y_max = y[i];
        if (y[i] < y_min)
            y_min = y[i];
    }
    ui->customPlot_disp->xAxis->setRange(0, m_t[m_t.size() - 1]);
    if (y_max - y_min <= 0.1)
        ui->customPlot_disp->yAxis->setRange(floor(y_min - 3), floor(y_max + 3));
    else
       ui->customPlot_disp->yAxis->setRange(y_min, y_max);    
    ui->customPlot_disp->graph(0)->setData(x, y);
    ui->customPlot_disp->graph(0)->setPen(QPen(Qt::red));
    x.clear();
    y.clear();
    
    for (int i = 0; i < m_t.size()-1; i++) 
    {
        x.append(m_t[i]);
        y.append(m_x_euler[i]);

        if (y[i] > y_max)
            y_max = y[i];
        if (y[i] < y_min)
            y_min = y[i];
    }
    ui->customPlot_disp->xAxis->setRange(0, m_t[m_t.size() - 1]);
    if (y_max - y_min <= 0.1)
        ui->customPlot_disp->yAxis->setRange(floor(y_min - 3), floor(y_max + 3));
    else
       ui->customPlot_disp->yAxis->setRange(y_min, y_max);    
    ui->customPlot_disp->graph(1)->setData(x, y);
    ui->customPlot_disp->graph(1)->setPen(QPen(Qt::blue, 1, Qt::DashLine));
    
    // add input force plots for reference
    QVector<double> in_force_vals(m_input_force->getValues().size());
    for (int i = 0; i < m_t.size(); i++)
    {
        in_force_vals[i] = m_input_force->getValues()[i];
    }
    ui->customPlot_vel->graph(2)->setData(x, in_force_vals);
    ui->customPlot_vel->graph(2)->setPen(QPen(Qt::gray, 1, Qt::DashDotLine));
    ui->customPlot_disp->graph(2)->setData(x, in_force_vals);
    ui->customPlot_disp->graph(2)->setPen(QPen(Qt::gray, 1, Qt::DashDotLine));
    
    // legend
    ui->customPlot_vel->legend->setVisible(true);
    ui->customPlot_vel->graph(0)->setName("RK4");
    ui->customPlot_vel->graph(1)->setName("Euler");
    ui->customPlot_vel->graph(2)->setName("Input");

    ui->customPlot_disp->legend->setVisible(true);
    ui->customPlot_disp->graph(0)->setName("RK4");
    ui->customPlot_disp->graph(1)->setName("Euler");
    ui->customPlot_disp->graph(2)->setName("Input");
    
    ui->customPlot_vel->replot();
    ui->customPlot_disp->replot();
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
    readAndSetSimVariables();

    const int N = m_t.size();
    QVector<double> x(N), y(N);

    readAndSetForceVariables();
    
    double y_max = m_input_force->getValues()[0];
    double y_min = m_input_force->getValues()[0];
    for (int i = 0; i < N; ++i) {
        x[i] = m_t[i];
        y[i] = m_input_force->getValues()[i];

        if (y[i] > y_max)
            y_max = y[i];
        if (y[i] < y_min)
            y_min = y[i];
    }
    ui->customPlot->graph(0)->setData(x, y);
    if (y_max - y_min <= 0.1)
        ui->customPlot_vel->yAxis->setRange(y_min - 3.0,y_max + 3.0);
    else
        ui->customPlot->yAxis->setRange(y_min, y_max);  
    ui->customPlot->xAxis->setRange(0, m_t.back());
    ui->customPlot->replot();

}



void MainWindow::readAndSetSimVariables()
{
    m_k1 = ui->k1SpinBox->value();
    m_k2 = ui->k2SpinBox->value();
    m_mass = ui->massSpinBox->value();
    m_mi = ui->miSpinBox->value();
    m_t_max = ui->t_maxSpinBox->value();
    m_dt = ui->d_tSpinBox->value();
    size_t nupoints = static_cast<size_t>(m_t_max / m_dt) + 1;
    m_t = matplot::linspace(0, m_t_max, nupoints);
    m_x0 = ui->x_0SpinBox->value();
    m_v0 = ui->v_0SpinBox->value();

    k1_global = m_k1;
    k2_global = m_k2;
    m_global = m_mass;
    mi_global = m_mi;
    
}

void MainWindow::readAndSetForceVariables()
{
    double period = ui->periodSpinBox->value();
    double amplitude = ui->ampSpinBox->value();
    double phase = ui->phaseSpinBox->value();
    double offset = ui->offsetSpinBox->value();
    double duty_cycle = ui->dutySpinBox->value();
    ForceType force_type = static_cast<ForceType>(ui->dropBox_signal->currentIndex());

    m_input_force->updateForce(force_type, m_t, period, amplitude, phase, offset, duty_cycle);
}

// std::filesystem::path getResPath()
// {
//     return std::filesystem::path(RESOURCE_PATH);
// }

double v_dt(double time, double x, double v, Force input)
{   
    return 1.0/m_global*( (input.atTime(time)) - ((k1_global + k2_global) * x) - /*(sgn(v)**/  v * mi_global );
}

double x_dt(double time, double x, double v, Force input)
{
    return v;
}

int sgn(double value)
{
    if (value < 0)
        return -1;
    else 
        return 1;
}

