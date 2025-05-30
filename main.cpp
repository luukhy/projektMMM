#include <iostream>
#include <matplot/matplot.h>
#include <vector>

const double t_max = 30.0;
const double dt = 0.1;
const double x0 = 4;
const double v0 = 0;
auto t = matplot::linspace(0, t_max, static_cast<int>(t_max / dt));
double k = 2.0;
double m = 1.0;
double mi = 0.01;

std::vector<double> x;
std::vector<double> v;

double v_dt(double t, double x, double v)
{
    return -k/m * x - 0.3/m * v;
}

double x_dt(double t, double x, double v)
{
    return v;    
}

void solveRK4(double initial_a, double initial_b, double step,
            double (*func_a)(double, double, double),
            double (*func_b)(double, double, double),
            std::vector<double>& result_a, std::vector<double>& result_b)
{
    result_a.push_back(initial_a);
    result_b.push_back(initial_b);

    double k1a, k1b, k2a, k2b, k3a, k3b, k4a, k4b;
    for (int i = 0; i < t.size(); i++)
    {
        k1a = func_a(t[i], result_a[i], result_b[i]);
        k1b = func_b(t[i], result_a[i], result_b[i]);

        k2a = func_a(t[i] + 0.5*step, result_a[i] + 0.5*step*k1a, result_b[i] + 0.5*k1b*step);
        k2b = func_b(t[i] + 0.5*step, result_a[i] + 0.5*step*k1a, result_b[i] + 0.5*k1b*step);
        
        k3a = func_a(t[i] + 0.5*step, result_a[i] + 0.5*step*k2a, result_b[i] + 0.5*step*k2b);
        k3b = func_b(t[i] + 0.5*step, result_a[i] + 0.5*step*k2a, result_b[i] + 0.5*step*k2b);

        k4a = func_a(t[i] + step, result_a[i] + step*k3a, result_b[i] + step*k3b);
        k4b = func_b(t[i] + step, result_a[i] + step*k3a, result_b[i] + step*k3b);

        result_a.push_back(result_a[i] + 1.0/6.0*step*(k1a + 2*k2a + 2*k3a + k4a));
        result_b.push_back(result_b[i] + 1.0/6.0*step*(k1b + 2*k2b + 2*k3b + k4b));
    }
}

int main()
{
    solveRK4(x0, v0, dt, x_dt, v_dt, x, v);

    matplot::subplot(1, 2, 1);
    matplot::plot(t, x);
    matplot::xlabel("time");
    matplot::title("Displacement");

    matplot::subplot(1, 2, 2);
    matplot::plot(t, v);
    matplot::xlabel("time");
    matplot::title("Velocity");
    matplot::show();
    return 0;
}

