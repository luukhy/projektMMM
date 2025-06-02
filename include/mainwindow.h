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

// Dodajemy publiczny slot do uruchamiania symulacji z UI (np. przycisku)
public slots:
    void runSimulationAndPlot(); // Nazwa slotu

private:
    Ui::MainWindow *ui;

    // Parametry systemu (przeniesione z main.cpp)
    double t_max;
    double dt;
    double x0;
    double v0;
    double k;
    double m;
    double mi; // Zauważ, że 'mi' nie było używane w Twoim v_dt, może miało być 0.3?

    // Wektor czasu i wektory wyników
    std::vector<double> t;
    std::vector<double> x_rk4;
    std::vector<double> v_rk4;
    std::vector<double> x_euler;
    std::vector<double> v_euler;


 
    // Prywatne metody dla logiki symulacji (przeniesione z main.cpp)
    // Zmieniamy nazwy, aby odróżnić od globalnych, jeśli takie jeszcze istnieją
    // i dostosowujemy je, aby były metodami klasy
    
    

    // Metody solverów - teraz będą używać v_dt i x_dt bezpośrednio
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

    // Metoda do rysowania wykresów
    void plotResults(); 
};

double v_dt(double time, double x, double v, Force input); // 'time' zamiast 't' dla jasności
double x_dt(double time, double x, double v, Force input);

#endif // MAINWINDOW_H