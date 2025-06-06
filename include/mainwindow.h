#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector> 
#include <force.h>

// struct defaultInForceValues
// {
//     double t_max = 10;
//     double dt = 0.01;
//     size_t nupoints = static_cast<size_t>(t_max / dt) + 1;
//     double time = matplot::linspace(0, t_max, nupoints);
//     ForceType force_type = ForceType::SINE;
//     double period = 1.0;
//     double amplitude = 1.0;
//     double phase = 0.0;
//     double freq = 1.0/period;
//     double offset = 0.0;
//     double duty_cycle = 0.5;
// } def_force_vals;

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
    // UI variables
    Ui::MainWindow *ui;
    Force *m_input_force;
    QTimer *m_timer;
    double m_phase = 0.0;


    // simulation variables
    double m_t_max;
    double m_dt;
    double m_x0;
    double m_v0;
    double m_k1;
    double m_k2;
    double m_mass;
    double m_mi;


    std::vector<double> m_t;
    std::vector<double> m_x_rk4;
    std::vector<double> m_v_rk4;
    std::vector<double> m_x_euler;
    std::vector<double> m_v_euler;    

    // Force m_input_force;

    // simulation functionalities
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

    // UI functionalities
    void updateQtPlots();
    void plotResultsMatplot(); 
    void updateInputGraph();
};

double v_dt(double time, double x, double v, Force input); // 'time' zamiast 't' dla jasności
double x_dt(double time, double x, double v, Force input);



#endif // MAINWINDOW_H