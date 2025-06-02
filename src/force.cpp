#include <force.h>
#include <math.h>
#include <cmath>

#define PI 3.14159265


Force::Force(ForceType type, std::vector<double>time, double period, double amplitude, double phase, double offset)
{
    this->time = time;
    this->force_type = type;
    this->period = period;
    this->amplitude = amplitude;
    this->phase = phase;
    this->freq = 1.0/period;
    this->offset = offset;
    this->values = populateForce(force_type, time, this->freq, amplitude, phase, offset);
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
    this->values = populateForce(force_type, time, freq, amplitude, phase, this->offset);        
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

void populateSine(std::vector<double>& vals, std::vector<double> args, double freq , double ampl, double phase, double offset)
{
    for (int i = 0; i < args.size(); i++)
    {   
        double value_t = ampl * sin(2 * PI * freq * args[i] + 2 * PI * phase) + offset;
        vals.push_back(value_t);
    }
}

void populateSawtooth(std::vector<double>& vals, std::vector<double> args, double freq , double ampl, double phase, double offset)
{
    double period = 1.0 / freq;
    for (int i = 0; i < args.size(); i++)
    {   
        double value_t = ampl * 2 * abs((args[i] - phase)/ period - floor((args[i] - phase)/ period + 0.5)) + offset;
        vals.push_back(value_t);
    }
}


std::vector<double> populateForce(ForceType force_type, std::vector<double> time, double freq, double amplitude, double phase, double offset)
{
    std::vector<double> input_force;
    if (force_type == ForceType::SINE)
    {
        populateSine(input_force, time, freq, amplitude, phase, offset);
    }
    if (force_type == ForceType::TRIANGLE)
    {
        populateSawtooth(input_force, time, freq, amplitude, phase, offset);
    }
    return input_force;
}
