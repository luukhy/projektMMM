#include <force.h>


#define PI 3.14159265


Force::Force()
{
    double t_max = 10;
    double dt = 0.01;
    size_t nupoints = static_cast<size_t>(t_max / dt) + 1;
    this->time = matplot::linspace(0, t_max, nupoints);
    this->force_type = ForceType::SINE;
    this->period = 1.0;
    this->amplitude = 1.0;
    this->phase = 0.0;
    this->freq = 1.0/period;
    this->offset = 0.0;
    this->duty_cycle = 0.5;
    this->values = populateForce(force_type, time, this->freq, amplitude, phase, offset, duty_cycle);
}

Force::Force(ForceType type, std::vector<double>time, double period, double amplitude, double phase, double offset = 0.0, double duty_cycle = 0.0)
{
    this->time = time;
    this->force_type = type;
    this->period = period;
    this->amplitude = amplitude;
    this->phase = phase;
    this->freq = 1.0/period;
    this->offset = offset;
    this->duty_cycle = duty_cycle;
    this->values = populateForce(force_type, time, this->freq, amplitude, phase, offset, duty_cycle);
}

void Force::updateForce(ForceType type, std::vector<double>time, double period, double amplitude, double phase, double duty_cycle)
{
    this->values.clear();
    this->time.clear();

    this->time = time;
    this->force_type = type;
    this->period = period;
    this->amplitude = amplitude;
    this->phase = phase;
    this->duty_cycle = duty_cycle;
    this->values = populateForce(force_type, time, freq, amplitude, phase, this->offset, duty_cycle);        
}

double Force::atTime(double t)
{
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

void populateSquare(std::vector<double>& vals, std::vector<double> args, double freq , double ampl, double phase, double offset, double duty_cycle)
{
    if (ampl == 0.0)
        offset = 0.0;
    double period = 1.0 / freq;
    double high_time = duty_cycle * period;
    for (int i = 0; i < args.size(); i++)
    {   
        double t_mod = fmod(args[i], period);
        if (t_mod < high_time)
            vals.push_back(ampl + offset);
        else 
            vals.push_back(-ampl + offset);
    }
}


std::vector<double> populateForce(ForceType force_type, std::vector<double> time, double freq, double amplitude, double phase, double offset, double duty_cycle)
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
    if (force_type == ForceType::SQUARE)
    {
        populateSquare(input_force, time, freq, amplitude, phase, offset, duty_cycle);
    }
    return input_force;
}
