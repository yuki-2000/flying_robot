#include "utility.hpp"

float map_float(float x, float xmin, float xmax, float ymin, float ymax) {
    return (x - xmin) * (ymax - ymin) / (xmax - xmin) + ymin;
}

float safe_asin(float x) {
    if (x < -1) {
        return M_PI / 2;
    } else if (x > 1) {
        return M_PI / 2;
    } else {
        return asin(x);
    }
}

void rotate_vector3(float M[3][3], float x[3], float y[3]) {
    for (int i = 0; i < 3; i++) {
        y[i] = 0;
        for (int j = 0; j < 3; j++) {
            y[i] += M[i][j] * x[j];
        }
    }
}

float constrain_float(float x, float xmin, float xmax) {
    if (x < xmin)
        return xmin;
    else if (x > xmax)
        return xmax;
    else
        return x;
}
