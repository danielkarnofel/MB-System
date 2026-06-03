#pragma once

#include "vec3.h"

#include <algorithm>
#include <cmath>

struct EigenDecomposition {
    Vec3 values;
    Vec3 vectors[3];

    int index_of_smallest_value() const {
        int index = 0;
        if (values.y < values.x) {
            index = 1;
        }
        if ((index == 0 && values.z < values.x) || (index == 1 && values.z < values.y)) {
            index = 2;
        }
        return index;
    }
};

struct Mat3 {
    double m[3][3];

    Mat3() {
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                m[row][col] = 0.0;
            }
        }
    }

    void add_outer_product(const Vec3 &v) {
        m[0][0] += v.x * v.x;
        m[0][1] += v.x * v.y;
        m[0][2] += v.x * v.z;
        m[1][0] += v.y * v.x;
        m[1][1] += v.y * v.y;
        m[1][2] += v.y * v.z;
        m[2][0] += v.z * v.x;
        m[2][1] += v.z * v.y;
        m[2][2] += v.z * v.z;
    }

    Mat3 multiply(double scale) const {
        Mat3 result;
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                result.m[row][col] = m[row][col] * scale;
            }
        }
        return result;
    }

    Mat3 operator*(double scale) const {
        return multiply(scale);
    }

    EigenDecomposition eigen_decomposition_symmetric() const {
        Mat3 a = *this;
        Mat3 eigenvectors = identity();

        for (int iteration = 0; iteration < 32; iteration++) {
            int p = 0;
            int q = 1;
            double largest = std::fabs(a.m[0][1]);

            if (std::fabs(a.m[0][2]) > largest) {
                p = 0;
                q = 2;
                largest = std::fabs(a.m[0][2]);
            }
            if (std::fabs(a.m[1][2]) > largest) {
                p = 1;
                q = 2;
                largest = std::fabs(a.m[1][2]);
            }
            if (largest < 1.0e-12) {
                break;
            }

            const double theta = 0.5 * std::atan2(2.0 * a.m[p][q], a.m[q][q] - a.m[p][p]);
            const double c = std::cos(theta);
            const double s = std::sin(theta);

            const double app = c * c * a.m[p][p] - 2.0 * s * c * a.m[p][q] + s * s * a.m[q][q];
            const double aqq = s * s * a.m[p][p] + 2.0 * s * c * a.m[p][q] + c * c * a.m[q][q];
            a.m[p][p] = app;
            a.m[q][q] = aqq;
            a.m[p][q] = 0.0;
            a.m[q][p] = 0.0;

            for (int r = 0; r < 3; r++) {
                if (r != p && r != q) {
                    const double arp = c * a.m[r][p] - s * a.m[r][q];
                    const double arq = s * a.m[r][p] + c * a.m[r][q];
                    a.m[r][p] = arp;
                    a.m[p][r] = arp;
                    a.m[r][q] = arq;
                    a.m[q][r] = arq;
                }

                const double vrp = c * eigenvectors.m[r][p] - s * eigenvectors.m[r][q];
                const double vrq = s * eigenvectors.m[r][p] + c * eigenvectors.m[r][q];
                eigenvectors.m[r][p] = vrp;
                eigenvectors.m[r][q] = vrq;
            }
        }

        EigenPair pairs[3] = {
            {a.m[0][0], {eigenvectors.m[0][0], eigenvectors.m[1][0], eigenvectors.m[2][0]}},
            {a.m[1][1], {eigenvectors.m[0][1], eigenvectors.m[1][1], eigenvectors.m[2][1]}},
            {a.m[2][2], {eigenvectors.m[0][2], eigenvectors.m[1][2], eigenvectors.m[2][2]}}
        };

        std::sort(pairs, pairs + 3, [](const EigenPair &a_pair, const EigenPair &b_pair) {
            return a_pair.value < b_pair.value;
        });

        EigenDecomposition result;
        result.values = {pairs[0].value, pairs[1].value, pairs[2].value};
        result.vectors[0] = normalize(pairs[0].vector);
        result.vectors[1] = normalize(pairs[1].vector);
        result.vectors[2] = normalize(pairs[2].vector);
        return result;
    }

private:
    struct EigenPair {
        double value;
        Vec3 vector;
    };

    static Mat3 identity() {
        Mat3 result;
        result.m[0][0] = 1.0;
        result.m[1][1] = 1.0;
        result.m[2][2] = 1.0;
        return result;
    }
};
