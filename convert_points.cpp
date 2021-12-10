#include <stdint.h>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <tuple>

constexpr size_t ledsN = 12;

double array[ledsN*2] = {
    19.3750,53.5940, // 0  U12
    20.9250,61.2250, // 1  U11
    29.9010,61.2000, // 2  U10
    31.4960,53.8480, // 3   U9
    38.9890,39.2430, // 4   U8
    33.4720,43.3070, // 5   U7
    25.5270,37.8460, // 6   U6
    13.8430,38.3540, // 7   U5
    17.7240,32.0040, // 8   U4
    17.7240,43.1800, // 9   U3
    11.3740,30.3530, // 10  U2
     8.5800,22.7330, // 11  U1
};

int main() {
    double xmin = +100000.0;
    double xmax = -100000.0;
    double ymin = +100000.0;
    double ymax = -100000.0;
    double cxa = -100000.0;
    double cya = -100000.0;

    double cx = (array[ 0*2+0] +
                 array[ 3*2+0] +
                 array[ 9*2+0] +
                 array[ 5*2+0]) / 4;

    double cy = (array[ 0*2+1] +
                 array[ 3*2+1] +
                 array[ 9*2+1] +
                 array[ 5*2+1]) / 4;

    printf("%f %f\n", cx, cy);

    for (size_t c = 0; c < ledsN; c++) {

        xmin = std::min(xmin, array[c*2+0]);
        xmax = std::max(xmax, array[c*2+0]);
        ymin = std::min(ymin, array[c*2+1]);
        ymax = std::max(ymax, array[c*2+1]);

        cxa = std::max(cxa, fabs(array[c*2+0]-cx));
        cya = std::max(cya, fabs(array[c*2+1]-cy));
    }

    std::vector<std::tuple<double, double, double, double>> coords;

    for (size_t c = 0; c < ledsN; c++) {
        double xr = (array[c*2+0]-xmin)/(xmax-xmin);
        double yr = (array[c*2+1]-ymin)/(ymax-ymin);
        double xc = (array[c*2+0]-cx)/cxa;
        double yc = (array[c*2+1]-cy)/cya;
        coords.push_back({xr, yr, sqrt(xc * xc + yc * yc) / sqrt(2), atan2(yc, xc) + 3.14159265359});
    }

    std::reverse(coords.begin(),coords.end());

    for (size_t c = 0; c < ledsN; c++) {
        printf("    fixed32<24>(%14.12ff), fixed32<24>(%14.12ff), fixed32<24>(%14.12ff), fixed32<24>(%14.12ff),\n", 
            std::get<0>(coords[c]),
            std::get<1>(coords[c]),
            std::get<2>(coords[c]),
            std::get<3>(coords[c]));
    }

    return 0;
}
