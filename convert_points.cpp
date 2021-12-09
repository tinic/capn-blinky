#include <stdint.h>
#include <algorithm>
#include <iostream>
#include <stdio.h>

constexpr size_t ledsN = 12;

double array[ledsN*2] = {
    19.3750,53.5940,
    20.9250,61.2250,
    29.9010,61.2000,
    31.4960,53.8480,
    38.9890,39.2430,
    33.4720,43.3070,
    25.5270,37.8460,
    13.8430,38.3540,
    17.7240,32.0040,
    17.7240,43.1800,
    11.3740,30.3530,
     8.5800,22.7330,
};

int main() {
    double xmin = +100000.0;
    double xmax = -100000.0;
    double ymin = +100000.0;
    double ymax = -100000.0;

    for (size_t c = 0; c < ledsN; c++) {
        xmin = std::min(xmin, array[c*2+0]);
        xmax = std::max(xmax, array[c*2+0]);
        ymin = std::min(ymin, array[c*2+1]);
        ymax = std::max(ymax, array[c*2+1]);
    }

    for (size_t c = 0; c < ledsN; c++) {
        printf("    fixed32<24>(%14.12ff), fixed32<24>(%14.12ff),\n",
            (array[c*2+0]-xmin)/(xmax-xmin),
            (array[c*2+1]-ymin)/(ymax-ymin)
        );
    }

    return 0;
}
