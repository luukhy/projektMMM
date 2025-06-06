#include <iostream>
#include <matplot/matplot.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <force.h>

#define PI 3.14159265

const double t_max = 6.0;
const double dt = 0.01;
const double x0 = 10;
const double v0 = 0;
auto t = matplot::linspace(0, t_max, static_cast<int>(t_max / dt));
double k = 30.0;
double m = 1.0;
double mi = 1;


void solveRK4(double initial_a, double initial_b, double step,
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
    
void solveEuler(double initial_a, double initial_b, double step,
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
    
    std::vector<double> x_rk4;
    std::vector<double> v_rk4;
    std::vector<double> x_euler;
    std::vector<double> v_euler;
    
double v_dt(double t, double x, double v, Force input)
{
    return -k/m * x + input.atTime(t) - mi * v;
}

double x_dt(double t, double x, double v, Force input)
{
    return v;    
}

int main()
{
    double period = 2.0;
    double amplitude = 0;
    double phase = 4.3;
    double offset = 0.0;
    double duty_cycle = 0.25;
    
    double freq =  sqrt(k/m) / (2*PI);
    period = 1.0 / freq;
    Force input_force(ForceType::SINE, t, period, amplitude, phase, offset, duty_cycle);
    
    solveRK4(x0, v0, dt, x_dt, v_dt, x_rk4, v_rk4, input_force);
    solveEuler(x0, v0, dt, x_dt, v_dt, x_euler, v_euler, input_force);
    

    #ifdef __linux__
        setenv("GNUTERM", "x11", 1);
    #endif
    
    matplot::figure(true);
    matplot::subplot(1, 2, 1);
    matplot::hold(matplot::on);
    matplot::plot(t, input_force.getValues());
    // matplot::plot(t, x_euler);
    // matplot::xlabel("time");
    // matplot::title("Displacement");

    matplot::subplot(1, 2, 2);
    matplot::hold(matplot::on);
    matplot::plot(t, v_rk4);
    matplot::plot(t, x_rk4);
    // matplot::plot(t, x_euler);
    // matplot::plot(t, v_euler);
    matplot::xlabel("time");
    matplot::title("vel + displ");
    matplot::show();


    std::vector<double> vec1 = {0.0, 1.0, 2.0, 3.0, 4.0};
                        //       0    1    2    3    4
    std::vector<double> vec2 = {3.0, 15.0, 2.0, 3.3, 4.2};



    return 0;
}
