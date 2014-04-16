typedef struct kalman_filter KF;

#ifndef __KALMAN_H
#define __KALMAN_H

#include "math.h"

struct kalman_filter{
	float Q, R, K, X, x, P, p;
};

void kalman_filter_init(KF *kf, float Q, float R, float X, float P);
void kalman_state_predict(KF *kf);
void kalman_covariance_predict(KF *kf);
void kalman_state_update(KF *kf, float Z);
void kalman_covariance_update(KF *kf);
void kalman_gain(KF *kf);
void kalman_filtering(KF *kf, float *data, int length);

#endif

