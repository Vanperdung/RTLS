#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct 
{
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
}kalmanFilter;

void kalman_init(kalmanFilter *obj, float mea_e, float est_e, float q, float _last_estimate);
float updateEstimate(kalmanFilter *obj, float mea);
void setMeasurementError(kalmanFilter *obj, float mea_e);
void setEstimateError(kalmanFilter *obj, float est_e);
void setProcessNoise(kalmanFilter *obj, float q);
float getKalmanGain(kalmanFilter obj);


#endif
