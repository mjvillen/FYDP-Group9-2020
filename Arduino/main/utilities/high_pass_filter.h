#include <math.h>

class HighPassFilter
{
public:
    HighPassFilter(float reduced_frequency)
    : alpha(1-exp(-2*M_PI*reduced_frequency)), y(0) {}
    float operator()(float x) {
        y += alpha*(x-y);
        return x-y;
    }
private:
    float alpha, y;
};