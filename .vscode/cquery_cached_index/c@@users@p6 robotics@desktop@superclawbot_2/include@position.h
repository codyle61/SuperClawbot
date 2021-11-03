#include "main.h"
#include "math.h"

double position1(int x1, int y1, int l1, int l2) {
  double d = (x1*x1 + y1*y1 - l1*l1 - l2*l2)/(2*l1*l2);
  return atan2((sqrt(1 - d*d)), d);
}

double position2(int a2, int x1, int y1, int l1, int l2) {
  return atan2(y1,x1) + atan2((l2*sin(a2)), (l1+l2*cos(a2)));
}
