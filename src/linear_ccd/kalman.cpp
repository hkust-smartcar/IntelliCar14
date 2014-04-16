#include "linear_ccd/kalman.h"

void kalman_filter_init(KF *kf, float Q, float R, float X, float P){
	kf->Q = Q;
	kf->R = R;
	kf->K = 0;
	kf->X = X;
	kf->x = 0;
	kf->P = P;
	kf->p = 0;
}

void kalman_state_predict(KF *kf){
	kf->x = kf->X;
}

void kalman_covariance_predict(KF *kf){
	kf->p = kf->P + kf->Q;
}

void kalman_state_update(KF *kf, float Z){
	kf->X = kf->x + (kf->K * (Z - kf->x));
}

void kalman_covariance_update(KF *kf){
	kf->P = (1 - kf->K) * kf->p;
}

void kalman_gain(KF *kf){
	kf->K = kf->p / (kf->p + kf->R);
}

void kalman_filtering(KF *kf, float *data, int length){
	for(int i = 0; i < length; i++){
		kalman_state_predict(&kf[i]);
		kalman_covariance_predict(&kf[i]);
		kalman_gain(&kf[i]);
		kalman_state_update(&kf[i], data[i]);
		kalman_covariance_update(&kf[i]);
		data[i] = kf[i].X;
	}
}
