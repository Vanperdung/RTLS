#include "kalman_filter_real.h"

Matrix create_matrix(int rows, int cols)
{
    Matrix mat;
    mat.rows = rows;
    mat.cols = cols;
    mat.data = (double **)malloc(rows * sizeof(double *));
    for (int i = 0; i < rows; i++)
    {
        mat.data[i] = (double *)malloc(cols * sizeof(double));
    }
    return mat;
}

void free_matrix(Matrix mat)
{
    for (int i = 0; i < mat.rows; i++)
    {
        free(mat.data[i]);
    }
    free(mat.data);
}

Matrix matrix_multiply(Matrix mat1, Matrix mat2)
{
    if (mat1.cols != mat2.rows)
    {
        // printf("Invalid matrix dimensions for multiplication.\n");
        // exit(1);
    }

    Matrix result = create_matrix(mat1.rows, mat2.cols);
    for (int i = 0; i < mat1.rows; i++)
    {
        for (int j = 0; j < mat2.cols; j++)
        {
            result.data[i][j] = 0.0;
            for (int k = 0; k < mat1.cols; k++)
            {
                result.data[i][j] += mat1.data[i][k] * mat2.data[k][j];
            }
        }
    }
    return result;
}

Matrix matrix_transpose(Matrix mat)
{
    Matrix result = create_matrix(mat.cols, mat.rows);
    for (int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            result.data[j][i] = mat.data[i][j];
        }
    }
    return result;
}

Matrix matrix_inverse(Matrix mat)
{
    if (mat.rows != mat.cols)
    {
        // printf("Matrix must be square for inversion.\n");
        // exit(1);
    }

    int n = mat.rows;
    Matrix aug = create_matrix(n, 2 * n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            aug.data[i][j] = mat.data[i][j];
        }
        for (int j = n; j < 2 * n; j++)
        {
            aug.data[i][j] = (i == j - n) ? 1.0 : 0.0;
        }
    }

    for (int i = 0; i < n; i++)
    {
        double pivot = aug.data[i][i];
        if (pivot == 0.0)
        {
            // printf("Matrix is singular.\n");
            // exit(1);
        }
        for (int j = 0; j < 2 * n; j++)
        {
            aug.data[i][j] /= pivot;
        }
        for (int j = 0; j < n; j++)
        {
            if (j != i)
            {
                double factor = aug.data[j][i];
                for (int k = 0; k < 2 * n; k++)
                {
                    aug.data[j][k] -= factor * aug.data[i][k];
                }
            }
        }
    }

    Matrix inv = create_matrix(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            inv.data[i][j] = aug.data[i][j + n];
        }
    }

    free_matrix(aug);
    return inv;
}

Matrix matrix_add(Matrix mat1, Matrix mat2)
{
    if (mat1.rows != mat2.rows || mat1.cols != mat2.cols)
    {
        // printf("Invalid matrix dimensions for addition.\n");
        // exit(1);
    }

    Matrix result = create_matrix(mat1.rows, mat1.cols);
    for (int i = 0; i < mat1.rows; i++)
    {
        for (int j = 0; j < mat1.cols; j++)
        {
            result.data[i][j] = mat1.data[i][j] + mat2.data[i][j];
        }
    }
    return result;
}

Matrix matrix_subtract(Matrix mat1, Matrix mat2)
{
    if (mat1.rows != mat2.rows || mat1.cols != mat2.cols)
    {
        // printf("Invalid matrix dimensions for subtraction.\n");
        // exit(1);
    }

    Matrix result = create_matrix(mat1.rows, mat1.cols);
    for (int i = 0; i < mat1.rows; i++)
    {
        for (int j = 0; j < mat1.cols; j++)
        {
            result.data[i][j] = mat1.data[i][j] - mat2.data[i][j];
        }
    }
    return result;
}

Matrix scalar_multiply(double scalar, Matrix mat)
{
    Matrix result = create_matrix(mat.rows, mat.cols);
    for (int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            result.data[i][j] = scalar * mat.data[i][j];
        }
    }
    return result;
}

Matrix eye(int n)
{
    Matrix mat = create_matrix(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            mat.data[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    return mat;
}

Matrix dot(Matrix mat1, Matrix mat2)
{
    if (mat1.cols != mat2.rows)
    {
        // printf("Invalid matrix dimensions for dot product.\n");
        // exit(1);
    }

    Matrix result = create_matrix(mat1.rows, mat2.cols);
    for (int i = 0; i < mat1.rows; i++)
    {
        for (int j = 0; j < mat2.cols; j++)
        {
            result.data[i][j] = mat1.data[i][j] * mat2.data[i][j];
        }
    }
    return result;
}

Matrix ones(int rows, int cols)
{
    Matrix mat = create_matrix(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mat.data[i][j] = 1.0;
        }
    }
    return mat;
}

Matrix zeros(int rows, int cols)
{
    Matrix mat = create_matrix(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mat.data[i][j] = 0.0;
        }
    }
    return mat;
}

Matrix create_matrix_from_array(double *arr, int rows, int cols)
{
    Matrix mat = create_matrix(rows, cols);
    int index = 0;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mat.data[i][j] = arr[index];
            index++;
        }
    }
    return mat;
}

void print_matrix(Matrix mat)
{
    for (int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            printf("%f ", mat.data[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}


KalmanFilter create_kalman_filter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas)
{
    KalmanFilter kf;
    kf.dt = dt;
    kf.u_x = u_x;
    kf.u_y = u_y;
    kf.std_acc = std_acc;
    kf.x_std_meas = x_std_meas;
    kf.y_std_meas = y_std_meas;

    kf.u = create_matrix(2, 1);
    kf.u.data[0][0] = u_x;
    kf.u.data[1][0] = u_y;

    kf.x = create_matrix(4, 1);
    kf.A = create_matrix(4, 4);
    kf.B = create_matrix(4, 2);
    kf.H = create_matrix(2, 4);
    kf.Q = create_matrix(4, 4);
    kf.R = create_matrix(2, 2);
    kf.P = create_matrix(4, 4);

    kf.x.data[0][0] = 0.0;
    kf.x.data[1][0] = 0.0;
    kf.x.data[2][0] = 0.0;
    kf.x.data[3][0] = 0.0;

    // kf.A.data[0][0] = 1.0;
    // kf.A.data[0][2] = dt;
    // kf.A.data[1][1] = 1.0;
    // kf.A.data[1][3] = dt;
    // kf.A.data[2][2] = 1.0;
    // kf.A.data[3][3] = 1.0;

    kf.A.data[0][0] = 1.0;
    kf.A.data[0][2] = 0;
    kf.A.data[1][1] = 1.0;
    kf.A.data[1][3] = 0;
    kf.A.data[2][2] = 1.0;
    kf.A.data[3][3] = 1.0;

    kf.B.data[0][0] = (dt * dt) / 2;
    kf.B.data[1][1] = (dt * dt) / 2;
    kf.B.data[2][0] = dt;
    kf.B.data[3][1] = dt;

    kf.H.data[0][0] = 1.0;
    kf.H.data[1][1] = 1.0;

    kf.Q.data[0][0] = (dt * dt * dt * dt) / 4;
    kf.Q.data[0][2] = (dt * dt * dt) / 2;
    kf.Q.data[1][1] = (dt * dt * dt * dt) / 4;
    kf.Q.data[1][3] = (dt * dt * dt) / 2;
    kf.Q.data[2][0] = (dt * dt * dt) / 2;
    kf.Q.data[2][2] = (dt * dt);
    kf.Q.data[3][1] = (dt * dt * dt) / 2;
    kf.Q.data[3][3] = (dt * dt);

    kf.R.data[0][0] = x_std_meas * x_std_meas;
    kf.R.data[1][1] = y_std_meas * y_std_meas;

    kf.P = eye(4);

    return kf;
}

void updateDeltaT(KalmanFilter *kf, double dt)
{
    kf->dt = dt;
    kf->B.data[0][0] = (dt * dt) / 2;
    kf->B.data[1][1] = (dt * dt) / 2;
    kf->B.data[2][0] = dt;
    kf->B.data[3][1] = dt;
    kf->Q.data[0][0] = (dt * dt * dt * dt) / 4;
    kf->Q.data[0][2] = (dt * dt * dt) / 2;
    kf->Q.data[1][1] = (dt * dt * dt * dt) / 4;
    kf->Q.data[1][3] = (dt * dt * dt) / 2;
    kf->Q.data[2][0] = (dt * dt * dt) / 2;
    kf->Q.data[2][2] = (dt * dt);
    kf->Q.data[3][1] = (dt * dt * dt) / 2;
    kf->Q.data[3][3] = (dt * dt);
}

Matrix kalman_filter_predict(KalmanFilter *kf)
{
    Matrix x_k = matrix_multiply(kf->A, kf->x);
    Matrix u_k = matrix_multiply(kf->B, kf->u);
    kf->x = matrix_add(x_k, u_k);

    Matrix P_k = matrix_multiply(kf->A, matrix_multiply(kf->P, matrix_transpose(kf->A)));
    kf->P = matrix_add(P_k, kf->Q);

    Matrix x_k_2 = create_matrix(2, 1);
    x_k_2.data[0][0] = kf->x.data[0][0];
    x_k_2.data[1][0] = kf->x.data[1][0];
    return x_k_2;
}

Matrix kalman_filter_update(KalmanFilter *kf, Matrix z)
{
    Matrix H_t = matrix_transpose(kf->H);
    Matrix PHT = matrix_multiply(kf->P, H_t);
    Matrix S = matrix_add(matrix_multiply(kf->H, PHT), kf->R);

    Matrix K = matrix_multiply(PHT, matrix_inverse(S));

    Matrix Hx = matrix_multiply(kf->H, kf->x);
    Matrix z_minus_Hx = matrix_subtract(z, Hx);
    Matrix K_z_minus_Hx = matrix_multiply(K, z_minus_Hx);

    kf->x = matrix_add(kf->x, K_z_minus_Hx);

    Matrix I = eye(kf->H.cols);
    Matrix KH = matrix_multiply(K, kf->H);
    Matrix I_minus_KH = matrix_subtract(I, KH);
    kf->P = matrix_multiply(I_minus_KH, kf->P);

    Matrix x_k_2 = create_matrix(2, 1);
    x_k_2.data[0][0] = kf->x.data[0][0];
    x_k_2.data[1][0] = kf->x.data[1][0];
    return x_k_2;
}

void updateAcc(KalmanFilter *kf, double u_x, double u_y)
{
    kf->u_x = u_x;
    kf->u_y = u_y;
    kf->u.data[0][0] = u_x;
    kf->u.data[1][0] = u_y;
}