#include "main.h"
#include "math.h"

double position1(double x1, double y1, double l1, double l2) {
  double d = (x1*x1 + y1*y1 - l1*l1 - l2*l2)/(2*l1*l2);
  return atan2((sqrt(1 - d*d)), d); //74.37deg
}

double position2(double a2, double x1, double y1, double l1, double l2) {
  return atan2(y1,x1) + atan2((l2*sin(a2)), (l1+l2*cos(a2))); // -60
}
