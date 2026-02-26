#include <Arduino.h>

float randomFloatRange(float x)
{
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX / x);
}