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


//High pass butterworth filter order=2 alpha1=0.001 
class  FilterBuHp2
{
	public:
		FilterBuHp2()
		{
			v[0]=0.0;
			v[1]=0.0;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (9.955669720176472115e-1 * x)
				 + (-0.99115359586893536648 * v[0])
				 + (1.99111429220165336851 * v[1]);
			return 
				 (v[0] + v[2])
				- 2 * v[1];
		}
};