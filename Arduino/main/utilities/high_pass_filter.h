#include <math.h>

class HighPassFilter
{
public:
    HighPassFilter(float reduced_frequency)
    : alpha(1-exp(-2*M_PI*reduced_frequency)) {}
    float operator()(float x, float y) {
        y += alpha*(x-y);
        return x-y;
    }
private:
    float alpha;
};