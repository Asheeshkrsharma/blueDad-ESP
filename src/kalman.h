#include <math.h>
class Kalman_Filter_Distance
{
private:
    float R; // noise power desirable
    float Q; // noise power estimated
    float A;
    float C;
    float B;
    float cov;
    float x; // estimated signal without noise
public:
    /**
	 * Create 1-dimensional kalman filter
	 * @param  {float} options.R Process noise
	 * @param  {float} options.Q Measurement noise
	 * @param  {float} options.A State vector
	 * @param  {float} options.B Control vector
	 * @param  {float} options.C Measurement vector
	 */
    Kalman_Filter_Distance(float R = 1, float Q = 1, float A = 1, float B = 0, float C = 1)
    {
        this->R = R; // noise power desirable
        this->Q = Q; // noise power estimated
        this->A = A;
        this->C = C;
        this->B = B;
        this->cov = NAN;
        this->x = NAN; // estimated signal without noise
    }

    /**
	 * Filter a new value
	 * @param  {Number} z Measurement
	 * @param  {Number} u Control
	 * @return {Number}
	 */
    float filter(float z, float u = 0)
    {

        if (isnan(this->x))
        {
            this->x = (1 / this->C) * z;
            this->cov = (1 / this->C) * this->Q * (1 / this->C);
        }
        else
        {
            //Compute prediction
            float predX = (this->A * this->x) + (this->B * u);
            float predCov = ((this->A * this->cov) * this->A) + this->R;

            //Kalman gain
            float K = predCov * this->C * (1 / ((this->C * predCov * this->C) + this->Q));

            //Correction
            this->x = predX + K * (z - (this->C * predX));
            this->cov = predCov - (K * this->C * predCov);
        }
        return this->x;
    }

    /**
	 * Return the last filtered measurement
	 * @return {Number}
	 */
    float lastMeasurement()
    {
        return this->x;
    }

    /**
	 * Set measurement noise Q
	 * @param {Number} noise
	 */
    void setMeasurementNoise(float noise)
    {
        this->Q = noise;
    }

    /**
	 * Set the process noise R
	 * @param {Number} noise
	 */
    void setProcessNoise(float noise)
    {
        this->R = noise;
    }

    void reset(){
        this->cov = NAN;
        this->x = NAN; // estimated signal without noise
    }

    ~Kalman_Filter_Distance() {}
};