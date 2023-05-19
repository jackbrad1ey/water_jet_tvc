/*
 * Extended Kalman Filter library
 * Written by: Angus McLennan
 * and Pablo Pleydell.
 * Date: 12.02.23
 */

#ifndef EKF_H
#define EKF_H


#include "usbd_cdc_if.h"

#include <math.h>
#include "string.h"
#include "arm_math.h"

/* Definition of size of state vector */
#define STATE_VECTOR_ROWS 4
#define SQUARE_MATRIX_ROWS (STATE_VECTOR_ROWS)
#define SQUARE_MATRIX_COLS (STATE_VECTOR_ROWS)
#define SQUARE_MATRIX_SIZE (SQUARE_MATRIX_ROWS*SQUARE_MATRIX_COLS)
#define GRAVITY_MS2 9.81

typedef struct {

	/*	Out state vector, defined as four quaternion floats qu1, qu2, qu3 and qu4 */
	arm_matrix_instance_f32 qu;
	// Float array which stores the state data. This is referenced by qu
	float qu_data[STATE_VECTOR_ROWS];

	/*	Current pan and tilt angles of gimbal */
	float pan;
	float tilt;

	/* Kalman gain matrix */
	arm_matrix_instance_f32 K;
	// Float array which stores state covariance data. This is referenced by P
	float K_data[4*3];	// 4*3

	/* State covariance matrix */
	arm_matrix_instance_f32 P;
	// Float array which stores state covariance data. This is referenced by P
	float P_data[STATE_VECTOR_ROWS * STATE_VECTOR_ROWS];	// 4x4

	/* Process noise covariance matrix */
	arm_matrix_instance_f32 Q;
	// Float array which stores process noise covariance matrix data
	float Q_data[STATE_VECTOR_ROWS * STATE_VECTOR_ROWS];	// 4x4

	/* Measurement covariance matrix */
	arm_matrix_instance_f32 R;
	float R_data[3 * 3];	// 3x3

	/* Axial thrust of rocket in N */
	float Px;

	/* Mass of rocket in kg */
	float m;
} EKF;

void EKF_Init(EKF *ekf, float *initial_qu, float *inital_K, float *inital_P, float *inital_Q, float *inital_R, float Px);
void EKF_Predict(EKF *ekf, float p_rps, float q_rps, float r_rps, float dt);

void EKF_Update(EKF *ekf, float ax_mps2, float ay_mps2, float az_mps2, float m, float pan, float tilt);
arm_status dEP(arm_matrix_instance_f32 *qu, arm_matrix_instance_f32 *w, arm_matrix_instance_f32 *qd);
void EP2Euler321(float *qu, float *euler);

#endif
