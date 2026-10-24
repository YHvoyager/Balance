#ifndef QMATH_H
#define QMATH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Trigonometric lookup functions. Input angle is in degrees. */
float qmath_sin(float angle_deg);
float qmath_cos(float angle_deg);
float qmath_tan(float angle_deg);
float qmath_atan2(float y, float x);

/* Short aliases for existing code that uses the qmath module. */
float qsin(float angle_deg);
float qcos(float angle_deg);
float qtan(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* QMATH_H */