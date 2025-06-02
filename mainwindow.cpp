#include "mainwindow.h"
#include "ui_mainwindow.h" 
#include <matplot/matplot.h>
#include <iostream>
#include <QPushButton>
#define PI 3.14159265
//na koniec usunac niepotrzebne cout 

double k_global = 30.0;
double m_global = 1.0;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    
    t_max = 30.0;
    dt = 0.1;
    x0 = 4.0;
    v0 = 0.0;
    k = 3.0;
    m = 1.0;
    mi = 0.01;

    if (ui->runButton) 
    {
        connect(ui->runButton, &QPushButton::clicked, this, &MainWindow::runSimulationAndPlot);
    }
    if (ui->kSpinBox) { // Sprawdź, czy kSpinBox istnieje
        ui->kSpinBox->setValue(k_global);
        ui->kSpinBox->setValue(k); 
        
    }
    if (ui->mSpinBox) { // Sprawdź, czy kSpinBox istnieje
        ui->mSpinBox->setValue(m_global); 
        ui->mSpinBox->setValue(m); 
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}


Force::Force(ForceType type, std::vector<double>time, double period, double amplitude, double phase)
{
    this->time = time;
    this->force_type = type;
    this->period = period;
    this->amplitude = amplitude;
    this->phase = phase;
    this->freq = 1.0/period;
    this->values = populateForce(force_type, time, this->freq, amplitude, phase);
}
void Force::updateForce(ForceType type, std::vector<double>time, double period, double amplitude, double phase)
{
    this->values.clear();
    this->time.clear();

    this->time = time;
    this->force_type = type;
    this->period = period;
    this->amplitude = amplitude;
    this->phase = phase;
    this->values = populateForce(force_type, time, freq, amplitude, phase);        
}

double Force::atTime(double t)
{
    // if (this->force_type == ForceType::SINE)
    // {
    //     return amplitude * sin(2 * PI / this->period * t + 2 * PI * this->phase);
    //     ampl * sin(2 * PI / period * args[i] + 2 * PI * phase)
    // }
    return getInterpolatedValue(this->time, this->values, t);
}

std::vector<double> Force::getValues()
{
    return this->values;
}

std::vector<double> Force::getTime()
{
    return this->time;
}


double getInterpolatedValue(std::vector<double>& time, const std::vector<double>& values, double t_searched)
{
    size_t n = time.size();
    
    if (n < 2 || n != values.size())
    {
        //std::cout << "INVALID VECTORS!!!";
        return 0.0;
    }

    if (t_searched < time.front() || t_searched > time.back())
    {
        //std::cout << "TIME VALUE OUT OF BOUNDS!!!";
        return 0.0;
    }

    auto iterator = std::lower_bound(time.begin(), time.end(), t_searched);

    if (iterator == time.begin())
    {
        return values.front();
    }
    if (iterator == time.end())
    {
        return values.back();
    }

    size_t  i = iterator - time.begin();
    size_t i0 = i - 1;
    size_t i1 = i;
    
    double t0 = time[i0];
    double t1 = time[i1];
    double f0 = values[i0];
    double f1 = values[i1];

    double interpolated = f0 + (t_searched - t0) / (t1 - t0) * (f1 -f0);
    return interpolated;

}

void populateSine(std::vector<double>& vals, std::vector<double> args, double freq , double ampl, double phase)
{
    for (int i = 0; i < args.size(); i++)
    {   
        double value_t = ampl * sin(2 * PI * freq * args[i] + 2 * PI * phase);
        vals.push_back(value_t);
    }
}


std::vector<double> populateForce(ForceType force_type, std::vector<double> time, double freq, double amplitude, double phase)
{
    std::vector<double> input_force;
    if (force_type == ForceType::SINE)
    {
        populateSine(input_force, time, freq, amplitude, phase);
    }
    return input_force;
}


double v_dt(double time, double x, double v, Force input)
{
    return -k_global/m_global * x + input.atTime(time);
}

double x_dt(double time, double x, double v, Force input)
{
    return v;
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
            // Obliczenie pochodnych w aktualnym punkcie (t[i], result_a[i], result_b[i])
            double da_dt = func_a(t[i], result_a[i], result_b[i], input_force);
            double db_dt = func_b(t[i], result_a[i], result_b[i], input_force);
            
            // Obliczenie następnej wartości przy użyciu metody Eulera
            double next_a = result_a[i] + step * da_dt;
            double next_b = result_b[i] + step * db_dt;
            
            // Dodanie obliczonych wartości do wektorów wynikowych
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
    double amplitude = 1.0;
    double phase = 0.0;
    
    double freq =  sqrt(k/m) / (2*PI);
    period = 1.0 / freq;
    Force input_force(ForceType::SINE, t, period, amplitude, phase);

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