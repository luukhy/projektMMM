#ifndef FORCE_CLASS_HEADER
#define FORCE_CLASS_HEADER

#include <vector>
#include <math.h>
#include <cmath>
#include <matplot/matplot.h>

enum class ForceType {SQUARE, TRIANGLE, SINE};

std::vector<double> populateForce(ForceType force_type, std::vector<double> time, double period, double amplitude, double phase, double offset, double duty_cycle);
double getInterpolatedValue(std::vector<double>& time, const std::vector<double>& values, double t_searched);
void populateSine(std::vector<double>& vals, std::vector<double> args, double period, double ampl, double phase, double offset);
void populateSawtooth(std::vector<double>& vals, std::vector<double> args, double period, double ampl, double phase, double offset);
void populateSquare(std::vector<double>& vals, std::vector<double> args, double period, double ampl, double phase, double offset, double duty_cycle);

class Force
{   
    private:
    ForceType force_type;
    std::vector<double> values;
    std::vector<double> time;
    double period, amplitude, phase, freq, offset = 0.0, duty_cycle = 0;
    
    public:
    Force();
    Force(ForceType force_type, std::vector<double>time, double period, double amplitude, double phase, double offfset, double duty_cycle);
    void updateForce(ForceType force_type, std::vector<double> time, double period, double amplitude, double phase, double offset, double duty_cycle);
    double atTime(double t);
    std::vector<double> getValues();
    std::vector<double> getTime();

    void setValues(std::vector<double> vals) {this->values = vals;};
    void setPeriod(double period) {this->period = period;}
    void setAmplitude(double amplitude) {this->amplitude = amplitude;}
    void setPhase(double phase) {this->phase = phase;}
    void setFreq(double freq) {this->freq = freq;}
    void setOffset(double offset) {this->offset = offset;}
    void setDutyCycle(double duty_cycle) {this->offset = offset;}

    const double getPeriod() {return this->period;}
    const double getAmplitude() {return this->amplitude;}
    const double getPhase() {return this->phase;}
    const double getFreq() {return this->freq;}
    const double getOffset() {return this->offset;}
};

#endif //FORCE_CLASS_HEADER