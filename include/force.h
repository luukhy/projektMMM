#ifndef FORCE_CLASS_HEADER
#define FORCE_CLASS_HEADER

#include <vector>

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
    
    Force(ForceType type, std::vector<double>time, double period, double amplitude, double phase);
    void updateForce(ForceType type, std::vector<double>time, double period, double amplitude, double phase);
    double atTime(double t);
    std::vector<double> getValues();
    std::vector<double> getTime();
};

#endif //FORCE_CLASS_HEADER