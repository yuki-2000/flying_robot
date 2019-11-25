#ifndef UTILITY_HEADER_FILE
#define UTILITY_HEADER_FILE

#include <cinttypes>
#include <cmath>
#include <cstdio>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr float TO_RAD = M_PI / 180;
constexpr float TO_DEG = 1.0 / TO_RAD;

constexpr float GRAVITY_ACC = 9.8;

float map_float(float x, float xmin, float xmax, float ymin, float ymax);

float constrain_float(float x, float xmin, float xmax);

float safe_asin(float x);

void rotate_vector3(float M[3][3], float x[3], float y[3]);

template <size_t N>
void matrix_add_row(double matrix[N][N], const size_t from_row, double mult_val,
                    const size_t dest_row) {
    for (size_t i = 0; i < N; i++) {
        matrix[dest_row][i] += mult_val * matrix[from_row][i];
    }
}

template <size_t N>
bool inv_matrix(const double M[N][N], double output[N][N]) {
    double A[N][N] = {0.0};
    const double eps = 1e-32;

    // copy M and initialize output to
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < N; j++) {
            A[i][j] = M[i][j];
            output[i][j] = 0.0;
        }
        output[i][i] = 1.0;
    }

    for (size_t row = 0; row < N; row++) {
        size_t i_nonzero = row;

        // search nonzero column
        while (abs(A[i_nonzero][row]) < eps && row < N) i_nonzero++;
        if (i_nonzero == N) return false;

        matrix_add_row(A, i_nonzero, 1.0, row);
        matrix_add_row(output, i_nonzero, 1.0, row);

        const double val = A[row][row];

        // normalize row to A[row][row]=1
        for (size_t i = 0; i < N; i++) {
            A[row][i] /= val;
            output[row][i] /= val;
        }

        // elimination
        for (size_t ahead = row + 1; ahead < N; ahead++) {
            const double mult_val = -A[ahead][row] / A[row][row];
            matrix_add_row(A, row, mult_val, ahead);
            matrix_add_row(output, row, mult_val, ahead);
        }
    }

    // elimination
    for (int row = N - 1; row >= 0; row--) {
        for (size_t prev = 0; prev < (size_t)row; prev++) {
            const double mult_val = -A[prev][row];

            matrix_add_row(A, row, mult_val, prev);
            matrix_add_row(output, row, mult_val, prev);
        }
    }

    return true;
}

template <size_t N>
bool solve(const double M[N][N], double x[N], const double b[N]) {
    double inv_mat[N][N];
    const auto succ = inv_matrix(M, inv_mat);

    for (size_t i = 0; i < N; i++) {
        x[i] = 0;
        for (size_t j = 0; j < N; j++) {
            x[i] += inv_mat[i][j] * b[j];
        }
    }

    return succ;
}

// 値の符号を取る関数
template <typename T>
T sign(T a) {
    return a > 0 ? 1 : a < 0 ? -1 : 0;
}

#endif
