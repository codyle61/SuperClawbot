#include "main.h"

void chassisSet(int left, int right) {
  motorSet(2, left);
  motorSet(9, right);
}
