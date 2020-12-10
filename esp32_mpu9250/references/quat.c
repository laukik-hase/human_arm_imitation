#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

float qToFloat(long number, uint8_t q)
{
    unsigned long mask = 0;
    for (int i = 0; i < q; i++)
    {
        mask |= (1 << i);
    }
    return (number >> q) + ((number & mask) / (float)(2 << (q - 1)));
}

int main()
{
    /* Biases:
        dGx: -2.352844 dGy: -128.554962 dGz: -0.406693
        dAx: 0.010651 dAy: 0.007126 dAz: 0.990311
    */

    printf("Gx: %f\n", qToFloat(-62431, 16));
    printf("Gy: %f\n", qToFloat(-7022026, 16));
    printf("Gz: %f\n", qToFloat(130006, 16));

    printf("Ax: %f\n", qToFloat(-742, 16));
    printf("Ay: %f\n", qToFloat(297, 16));
    printf("Az: %f\n", qToFloat(65275, 16));

    return 0;
}