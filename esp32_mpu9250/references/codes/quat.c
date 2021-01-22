#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

float qToFloat(int32_t number, uint8_t q)
{
    return (float)((double)number / (double)((int32_t)1 << q));
}

int32_t floatToQ(float number, uint8_t q)
{
    return (int32_t)(number * ((int32_t)1 << q));
}

int main()
{
    float n = 30;
    float Q = qToFloat(245342453, n);
    printf("Q -> f: %f\n", Q);
    printf("f -> Q: %d\n", floatToQ(Q, n));
    return 0;
}