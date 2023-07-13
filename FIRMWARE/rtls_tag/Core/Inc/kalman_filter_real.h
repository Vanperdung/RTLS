#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

typedef struct
{
    double **data;
    int rows;
    int cols;
} Matrix;

typedef struct
{
    double dt;
    double u_x;
    double u_y;
    double std_acc;
    double x_std_meas;
    double y_std_meas;

    Matrix u;
    Matrix x;
    Matrix A;
    Matrix B;
    Matrix H;
    Matrix Q;
    Matrix R;
    Matrix P;
} KalmanFilter;

Matrix create_matrix(int rows, int cols);
void free_matrix(Matrix mat);
Matrix matrix_multiply(Matrix mat1, Matrix mat2);
Matrix matrix_transpose(Matrix mat);
Matrix matrix_inverse(Matrix mat);
Matrix matrix_add(Matrix mat1, Matrix mat2);
Matrix matrix_subtract(Matrix mat1, Matrix mat2);
Matrix scalar_multiply(double scalar, Matrix mat);
Matrix eye(int n);
Matrix dot(Matrix mat1, Matrix mat2);
Matrix ones(int rows, int cols);
Matrix zeros(int rows, int cols);
Matrix create_matrix_from_array(double *arr, int rows, int cols);
void print_matrix(Matrix mat);
KalmanFilter create_kalman_filter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas);
Matrix kalman_filter_predict(KalmanFilter *kf);
Matrix kalman_filter_update(KalmanFilter *kf, Matrix z);
void updateAcc(KalmanFilter *kf, double u_x, double u_y);
void updateDeltaT(KalmanFilter *kf, double dt);