#ifndef MAINWINDOW_HEADER
#define MAINWINDOW_HEADER

#include <QMainWindow>
#include <vector> 
#include <force.h>
#include <functional>

namespace Ui {
    class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    public slots:
    void runSimulationAndPlot(); 
    void updateQtPlots();
    void updateInputGraph();

    private:
    // UI variables
    Ui::MainWindow *ui;
    double m_phase = 0.0;
    Force *m_input_force;

    
    
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

    void readAndSetSimVariables();
    void readAndSetForceVariables();

    // UI functionalities
    void plotResultsMatplot(); 
    
};    

// std::filesystem::path getResPath();
double v_dt(double time, double x, double v, Force input); // 'time' zamiast 't' dla jasności
double x_dt(double time, double x, double v, Force input);
int sgn(double value);

#endif // MAINWINDOW_HEADER