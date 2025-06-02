#ifndef FORCE_CLASS_HEADER
#define FORCE_CLASS_HEADER

#include <vector>

enum class ForceType {SQUARE, TRIANGLE, SINE};

std::vector<double> populateForce(ForceType force_type, std::vector<double> time, double period, double amplitude, double phase, double offset);
double getInterpolatedValue(std::vector<double>& time, const std::vector<double>& values, double t_searched);
void populateSine(std::vector<double>& vals, std::vector<double> args, double period, double ampl, double phase, double offset);
void populateSawtooth(std::vector<double>& vals, std::vector<double> args, double period, double ampl, double phase, double offset);

class Force
{   
    private:
    ForceType force_type;
    std::vector<double> values;
    std::vector<double> time;
    double period, amplitude, phase, freq, offset = 0.0;
    
    public:
    Force(ForceType type, std::vector<double>time, double period, double amplitude, double phase, double offfset);
    void updateForce(ForceType type, std::vector<double>time, double period, double amplitude, double phase);
    double atTime(double t);
    std::vector<double> getValues();
    std::vector<double> getTime();

    void setPeriod(double period) {this->period = period;}
    void setAmplitude(double amplitude) {this->amplitude = amplitude;}
    void setPhase(double phase) {this->phase = phase;}
    void setFreq(double freq) {this->freq = freq;}
    void setOffset(double offset) {this->offset = offset;}

    const double getPeriod() {return this->period;}
    const double getAmplitude() {return this->amplitude;}
    const double getPhase() {return this->phase;}
    const double getFreq() {return this->freq;}
    const double getOffset() {return this->offset;}
};

#endif //FORCE_CLASS_HEADER