#include <iostream>
#include <matplot/matplot.h>
#include <vector>
#include <math.h>
#include <algorithm>

#define PI 3.14159265

const double t_max = 20.0;
const double dt = 0.01;
const double x0 = 0;
const double v0 = 0;
auto t = matplot::linspace(0, t_max, static_cast<int>(t_max / dt));
double k = 10.0;
double m = 1.0;
double mi = 0.01;

enum class ForceType {SQUARE, SAWTOOTH, SINE};

std::vector<double> populateForce(ForceType force_type, std::vector<double> time, double period, double amplitude, double phase);
double getInterpolatedValue(std::vector<double>& time, const std::vector<double>& values, double t_searched);
void populateSine(std::vector<double>& vals, std::vector<double> args, double period, double ampl, double phase);


class Force
{   
    private:
    ForceType force_type;
    std::vector<double> values;
    std::vector<double> time;
    double period, amplitude, phase, freq;
    public:
    
    Force(ForceType type, std::vector<double>time, double period, double amplitude, double phase)
    {
        this->time = time;
        this->force_type = type;
        this->period = period;
        this->amplitude = amplitude;
        this->phase = phase;
        this->freq = 1.0/period;
        this->values = populateForce(force_type, time, this->freq, amplitude, phase);
    }
    void updateForce(ForceType type, std::vector<double>time, double period, double amplitude, double phase)
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
    
    double atTime(double t)
    {
        // if (this->force_type == ForceType::SINE)
        // {
        //     return amplitude * sin(2 * PI / this->period * t + 2 * PI * this->phase);
        //     ampl * sin(2 * PI / period * args[i] + 2 * PI * phase)
        // }
        return getInterpolatedValue(this->time, this->values, t);
    }

    std::vector<double> getValues()
    {
        return this->values;
    }

    std::vector<double> getTime()
    {
        return this->time;
    }

};


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
    
    std::vector<double> x_rk4;
    std::vector<double> v_rk4;
    std::vector<double> x_euler;
    std::vector<double> v_euler;
    
double v_dt(double t, double x, double v, Force input)
{
    return -k/m * x + input.atTime(t);
}

double x_dt(double t, double x, double v, Force input)
{
    return v;    
}

int main()
{
    double period = 3.0;
    double amplitude = 1.0;
    double phase = 0.0;
    
    double freq =  sqrt(k/m) / (2*PI);
    period = 1.0 / freq;
    Force input_force(ForceType::SINE, t, period, amplitude, phase);
    
    solveRK4(x0, v0, dt, x_dt, v_dt, x_rk4, v_rk4, input_force);
    solveEuler(x0, v0, dt, x_dt, v_dt, x_euler, v_euler, input_force);
    
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
    matplot::title("Velocity");
    matplot::show();


    std::vector<double> vec1 = {0.0, 1.0, 2.0, 3.0, 4.0};
                        //       0    1    2    3    4
    std::vector<double> vec2 = {3.0, 15.0, 2.0, 3.3, 4.2};



    return 0;
}
