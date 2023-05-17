#include "EKF.h"

void EKF_Init(EKF *ekf, float *initial_qu, float *inital_K, float *inital_P,
		float *inital_Q, float *inital_R, float Px) {
	ekf->Px = Px;
	arm_status result;
	// Initialise state vector
	memcpy(ekf->qu_data, initial_qu, sizeof(ekf->qu_data) * sizeof(float));
	arm_mat_init_f32(&ekf->qu, STATE_VECTOR_ROWS, 1, (float32_t*) ekf->qu_data);

	// Initialise Kalman gain
	memcpy(ekf->K_data, inital_K, sizeof(ekf->K_data) * sizeof(float));
	arm_mat_init_f32(&ekf->K, 4, 3, (float32_t*) ekf->K_data);

	/* Initialise state covariance matrix */
	memcpy(&ekf->P_data, inital_P, sizeof(ekf->P_data) * sizeof(float));
	arm_mat_init_f32(&ekf->P, SQUARE_MATRIX_ROWS, SQUARE_MATRIX_COLS,
			(float32_t*) ekf->P_data);

	/* Initialise process noise */
	memcpy(ekf->Q_data, inital_Q, sizeof(ekf->Q_data) * sizeof(float));
	arm_mat_init_f32(&ekf->Q, SQUARE_MATRIX_ROWS, SQUARE_MATRIX_COLS,
			(float32_t*) ekf->Q_data);

	/* Initialise measurement noise */
	memcpy(ekf->R_data, inital_R, sizeof(ekf->R_data) * sizeof(float));
	arm_mat_init_f32(&ekf->R, 3, 3, (float32_t*) ekf->R_data);
}

void EKF_Predict(EKF *ekf, float p_rps, float q_rps, float r_rps, float dt) {
	arm_status result;

	/******** Compute State Vector *********/
	// Init input vector
	arm_matrix_instance_f32 u_vector;
	float u_vector_data[3];
	u_vector_data[0] = p_rps;
	u_vector_data[1] = q_rps;
	u_vector_data[2] = r_rps;
	arm_mat_init_f32(&u_vector, 3, 1, u_vector_data);

	// Calculate dq vector
	arm_matrix_instance_f32 dq;
	arm_matrix_instance_f32 temp_mat;
	float dq_data[4];
	float temp_mat_data[4];
	arm_mat_init_f32(&dq, STATE_VECTOR_ROWS, 1, dq_data);
	arm_mat_init_f32(&temp_mat, STATE_VECTOR_ROWS, 1, temp_mat_data);
	result = dEP(&ekf->qu, &u_vector, &dq);

	// Multiply dt by result in dq vector and store in dq vector
	for (int i = 0; i < STATE_VECTOR_ROWS; i++) {
		dq.pData[i] *= dt;
	}

	// Add qu to result in dq vector and store in qu vector (updating current state)
	result = arm_mat_add_f32(&ekf->qu, &dq, &temp_mat);
	arm_copy_f32(temp_mat_data, &ekf->qu_data, sizeof(temp_mat_data)/sizeof(float));

//	char printData[128];
//	size_t sz =
//			snprintf(printData, sizeof(printData),
//					"%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\r\n",
//					p_rps, q_rps, r_rps, dt, ekf->qu_data[0], ekf->qu_data[1],
//					ekf->qu_data[2], ekf->qu_data[3], dq_data[0], dq_data[1],
//					dq_data[2], dq_data[3]);
//			snprintf(printData, sizeof(printData),
//					"%0.4f, %0.4f, %0.4f, %0.4f\r\n",ekf->qu_data[0], ekf->qu_data[1],
//					ekf->qu_data[2], ekf->qu_data[3]);

//			debug_print(printData, sz);

	/******** Calculate A Matrix *********/
	float A_data[SQUARE_MATRIX_SIZE];
	float A_temp_data[SQUARE_MATRIX_SIZE];
	float A_transpose_data[SQUARE_MATRIX_SIZE];
	A_data[0] = 1;
	A_data[1] = -1 * (dt * p_rps) / 2;
	A_data[2] = -1 * (dt * q_rps) / 2;
	A_data[3] = -1 * (dt * r_rps) / 2;
	A_data[4] = (dt * p_rps) / 2;
	A_data[5] = 1;
	A_data[6] = (dt * r_rps) / 2;
	A_data[7] = -1 * (dt * q_rps) / 2;
	A_data[8] = (dt * q_rps) / 2;
	A_data[9] = -1 * (dt * r_rps) / 2;
	A_data[10] = 1;
	A_data[11] = (dt * p_rps) / 2;
	A_data[12] = (dt * r_rps) / 2;
	A_data[13] = (dt * q_rps) / 2;
	A_data[14] = -1 * (dt * p_rps) / 2;
	A_data[15] = 1;

	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 A_temp;
	arm_matrix_instance_f32 A_transpose;
	arm_mat_init_f32(&A, SQUARE_MATRIX_ROWS, SQUARE_MATRIX_COLS, A_data);
	arm_mat_init_f32(&A_temp, SQUARE_MATRIX_ROWS, SQUARE_MATRIX_COLS, A_temp_data);
	arm_mat_init_f32(&A_transpose, SQUARE_MATRIX_ROWS, SQUARE_MATRIX_COLS,
			A_transpose_data);
	arm_mat_trans_f32(&A, &A_transpose);

	/******** Calculate P Matrix *********/
	// Multiply A by P and store in A
	result = arm_mat_mult_f32(&A, &ekf->P, &A_temp);
	arm_copy_f32(A_temp_data, A_data, sizeof(A_temp_data)/sizeof(float));

	// Multiply new A by A transpose and store in A
	result = arm_mat_mult_f32(&A, &A_transpose, &A_temp);
	arm_copy_f32(A_temp_data, A_data, sizeof(A_temp_data)/sizeof(float));
	// Add new A to Q and store in P
	result = arm_mat_add_f32(&A, &ekf->Q, &ekf->P);
}

void EKF_Update(EKF *ekf, float ax_mps2, float ay_mps2, float az_mps2, float m,
		float pan, float tilt) {
	ekf->m = m;
	ekf->pan = pan;
	ekf->tilt = tilt;

	arm_status result;
	arm_matrix_instance_f32 temp_matrix3x4;
	arm_matrix_instance_f32 temp_matrix3x3;
	arm_matrix_instance_f32 temp_matrix3x3_2;
	arm_matrix_instance_f32 temp_matrixinv3x3;
	float tmp_data[12];
	float tmp_data2[9];
	float tmp_data3[9];
	float tmp_inv_data[9];
	arm_mat_init_f32(&temp_matrix3x4, 3, 4, tmp_data);
	arm_mat_init_f32(&temp_matrix3x3, 3, 3, tmp_data2);
	arm_mat_init_f32(&temp_matrix3x3_2, 3, 3, tmp_data3);
	arm_mat_init_f32(&temp_matrixinv3x3, 3, 3, tmp_inv_data);

	/******** Calculate C Matrix *********/
	float C_data[3 * 4];
	C_data[0] = -2 * GRAVITY_MS2 * ekf->qu.pData[2];
	C_data[1] = 2 * GRAVITY_MS2 * ekf->qu.pData[3];
	C_data[2] = -2 * GRAVITY_MS2 * ekf->qu.pData[0];
	C_data[3] = 2 * GRAVITY_MS2 * ekf->qu.pData[1];
	C_data[4] = 2 * GRAVITY_MS2 * ekf->qu.pData[1];
	C_data[5] = 2 * GRAVITY_MS2 * ekf->qu.pData[0];
	C_data[6] = 2 * GRAVITY_MS2 * ekf->qu.pData[3];
	C_data[7] = 2 * GRAVITY_MS2 * ekf->qu.pData[2];
	C_data[8] = 2 * GRAVITY_MS2 * ekf->qu.pData[0];
	C_data[9] = -2 * GRAVITY_MS2 * ekf->qu.pData[1];
	C_data[10] = -2 * GRAVITY_MS2 * ekf->qu.pData[2];
	C_data[11] = 2 * GRAVITY_MS2 * ekf->qu.pData[3];

	arm_matrix_instance_f32 C;
	arm_matrix_instance_f32 C_transpose;
	arm_matrix_instance_f32 temp_matrix;
	arm_matrix_instance_f32 temp_matrix4x3;
	arm_matrix_instance_f32 temp_matrix4x1;
	arm_matrix_instance_f32 temp_matrix4x4;
	float C_transpose_data[12];
	float tmp2_data[12];
	float mat_4x3_data[12];
	float mat_4x1_data[4];
	float mat_4x4_data[16];
	arm_mat_init_f32(&C, 3, 4, C_data);
	arm_mat_init_f32(&C_transpose, 4, 3, C_transpose_data);
	arm_mat_init_f32(&temp_matrix, 3, 4, tmp2_data);
	arm_mat_init_f32(&temp_matrix4x3, 4, 3, mat_4x3_data);
	arm_mat_init_f32(&temp_matrix4x1, 4, 1, mat_4x1_data);
	arm_mat_init_f32(&temp_matrix4x4, 4, 4, mat_4x4_data);
	arm_mat_trans_f32(&C, &C_transpose);

	/******** Calculate Kalman Gain *********/
	// Calculate C*P*C^T
	// Multiply C by P and store in temp_matrix
	result = arm_mat_mult_f32(&C, &ekf->P, &temp_matrix3x4);
	// Multiply result in temp_matrix by C transpose and store in temp_matrix
	result = arm_mat_mult_f32(&temp_matrix3x4, &C_transpose, &temp_matrix3x3);
	// Add result in temp_matrix to R and store in temp_matrix
	result = arm_mat_add_f32(&temp_matrix3x3, &ekf->R, &temp_matrix3x3_2);
	arm_copy_f32(tmp_data3, tmp_data2, sizeof(tmp_data3)/sizeof(float));
	// Invert result and store in temp_matrix;
	result = arm_mat_inverse_f32(&temp_matrix3x3, &temp_matrixinv3x3);

	// Multiply P by C transpose and store in temp_matrix2
	result = arm_mat_mult_f32(&ekf->P, &C_transpose, &temp_matrix4x3);
	// Multiply result in temp_matrix2 by temp_matrix and store in K
	result = arm_mat_mult_f32(&temp_matrix4x3, &temp_matrixinv3x3, &ekf->K);

	/******** Update State Vector *********/
	// Calcualte K*(y-h(x,u))
	// Calcualte H vector
	float H_data[3];
	H_data[0] = -1 * GRAVITY_MS2
			* (2 * ekf->qu.pData[0] * ekf->qu.pData[2]
					- 2 * ekf->qu.pData[1] * ekf->qu.pData[3])
			- (ekf->Px
					* (arm_cos_f32(ekf->pan / 2) * arm_cos_f32(ekf->pan / 2)
							* arm_cos_f32(ekf->tilt / 2 + PI / 2)
							* arm_cos_f32(ekf->tilt / 2 + PI / 2)
							- arm_cos_f32(ekf->pan / 2)
									* arm_cos_f32(ekf->pan / 2)
									* arm_sin_f32(ekf->tilt / 2 + PI / 2)
									* arm_sin_f32(ekf->tilt / 2 + PI / 2)
							+ arm_sin_f32(ekf->pan / 2)
									* arm_sin_f32(ekf->pan / 2)
									* arm_cos_f32(ekf->tilt / 2 + PI / 2)
									* arm_cos_f32(ekf->tilt / 2 + PI / 2)
							- arm_sin_f32(ekf->pan / 2)
									* arm_sin_f32(ekf->pan / 2)
									* arm_sin_f32(ekf->tilt / 2 + PI / 2)
									* arm_sin_f32(ekf->tilt / 2 + PI / 2)))
					/ ekf->m;
	H_data[1] = GRAVITY_MS2
			* (2 * ekf->qu.pData[0] * ekf->qu.pData[1]
					+ 2 * ekf->qu.pData[2] * ekf->qu.pData[3])
			- (4 * ekf->Px * arm_cos_f32(ekf->pan / 2)
					* arm_sin_f32(ekf->pan / 2)
					* arm_cos_f32(ekf->tilt / 2 + PI / 2)
					* arm_sin_f32(ekf->tilt / 2 + PI / 2)) / ekf->m;
	H_data[2] = GRAVITY_MS2
			* (ekf->qu.pData[0] * ekf->qu.pData[0]
					- ekf->qu.pData[1] * ekf->qu.pData[1]
					- ekf->qu.pData[2] * ekf->qu.pData[2]
					+ ekf->qu.pData[3] * ekf->qu.pData[3])
			- (ekf->Px
					* (2 * arm_cos_f32(ekf->pan / 2) * arm_cos_f32(ekf->pan / 2)
							* arm_cos_f32(ekf->tilt / 2 + PI / 2)
							* arm_sin_f32(ekf->tilt / 2 + PI / 2)
							- 2 * arm_sin_f32(ekf->pan / 2)
									* arm_sin_f32(ekf->pan / 2)
									* arm_cos_f32(ekf->tilt / 2 + PI / 2)
									* arm_sin_f32(ekf->tilt / 2 + PI / 2)))
					/ ekf->m;
	arm_matrix_instance_f32 H;
	arm_mat_init_f32(&H, 3, 1, H_data);

	// Create vector of sensor readings from accelerometer
	float accel_data[3];
	accel_data[0] = ax_mps2;
	accel_data[1] = ay_mps2;
	accel_data[2] = az_mps2;
	arm_matrix_instance_f32 accel_vector;
	arm_mat_init_f32(&accel_vector, 3, 1, accel_data);

	// Subtract accelerometer vector from H vector and store in accelerometer vector
	arm_mat_sub_f32(&accel_vector, &H, &accel_vector);
	// Multiply result in accel_vector by K and store in temp_matrix
	result = arm_mat_mult_f32(&ekf->K, &accel_vector, &temp_matrix4x1);

	// Add current state vector to result in temp_matrix and store in current state vector
	result = arm_mat_add_f32(&ekf->qu, &temp_matrix4x1, &ekf->qu);

	/******** Update P Matrix *********/
	// Create identity matrix
	float identity_matrix_data[SQUARE_MATRIX_SIZE] = { 1, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 1 };
	arm_matrix_instance_f32 I;
	arm_mat_init_f32(&I, SQUARE_MATRIX_ROWS, SQUARE_MATRIX_COLS,
			identity_matrix_data);

	// Multiply K by C and store in temp_matrix
	result = arm_mat_mult_f32(&ekf->K, &C, &temp_matrix4x4);
	// Subtract the result in temp_matrix from I and store in temp_matrix
	result = arm_mat_sub_f32(&I, &temp_matrix4x4, &temp_matrix4x4);
	// Multiply the result in temp_result by P and store in P
	result = arm_mat_mult_f32(&temp_matrix4x4, &ekf->P, &ekf->P);
}

/* dq = dEP(Q,W) returns the Euler parameter derivative
 * for a given Euler parameter vector Q and body
 * angular velocity vector w.
 */
arm_status dEP(arm_matrix_instance_f32 *qu, arm_matrix_instance_f32 *w,
		arm_matrix_instance_f32 *dq) {
	/* Calculate B Matrix */
	arm_status result;
	arm_matrix_instance_f32 B;
	float B_data[] = { -1 * qu->pData[1], -1 * qu->pData[2], -1 * qu->pData[3],
			qu->pData[0], -1 * qu->pData[3], qu->pData[2], qu->pData[3],
			qu->pData[0], -1 * qu->pData[1], -1 * qu->pData[2], qu->pData[1],
			qu->pData[0] };
	arm_mat_init_f32(&B, 4, 3, B_data);

	/* Multiply B by w and store in dq */
	result = arm_mat_mult_f32(&B, w, dq);
	if (result)
		return result;

	/* Multiply result in dq by 0.5 and store in dq */
	arm_mat_scale_f32(dq, 0.5, dq);
	if (result)
		return result;
}

/*
 * E = EP2Euler321(Q) translates the Euler parameter vector
 * Q into the corresponding (3-2-1) Euler angle set.
 */
void EP2Euler321(float *qu, float *euler) {
	float q0 = qu[0];
	float q1 = qu[1];
	float q2 = qu[2];
	float q3 = qu[3];
	// Normalise input quaternion
	float d = sqrt(pow(q0,2)+pow(q1,2)+pow(q2,2)+pow(q3,2));
	q0 /= d;
	q1 /= d;
	q2 /= d;
	q3 /= d;
	euler[0] = atan2(2 * (q1 * q2 + q0 * q3),
			q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
	euler[1] = asin(-2 * (q1 * q3 - q0 * q2));
	euler[2] = atan2(2 * (q2 * q3 + q0 * q1),
			q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}
