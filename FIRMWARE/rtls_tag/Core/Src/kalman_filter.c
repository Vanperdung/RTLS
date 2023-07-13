#include "kalman_filter.h"
#include "math.h"

void kalman_init(kalmanFilter *obj, float mea_e, float est_e, float q, float _last_estimate)
{
	obj->_err_measure = mea_e;
	obj->_err_estimate = est_e;
	obj->_last_estimate = _last_estimate;
	obj->_q = q;
}
float updateEstimate(kalmanFilter *obj, float mea)
{
	obj->_err_estimate += obj->_q;
	obj->_kalman_gain = obj->_err_estimate / (obj->_err_estimate + obj->_err_measure);
	obj->_current_estimate = obj->_last_estimate + obj->_kalman_gain * (mea - obj->_last_estimate);
	obj->_err_estimate = (1.0 - obj->_kalman_gain) * obj->_err_estimate;
	obj->_last_estimate = obj->_current_estimate;
	return obj->_current_estimate;
}

void setMeasurementError(kalmanFilter *obj, float mea_e)
{
	obj->_err_measure = mea_e;
}

void setEstimateError(kalmanFilter *obj, float est_e)
{
	obj->_err_estimate = est_e;
}

void setProcessNoise(kalmanFilter *obj, float q)
{
	obj->_q = q;
}

float getKalmanGain(kalmanFilter obj)
{
	return obj._kalman_gain;
}
