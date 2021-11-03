#include "main.h"

void clawSet(int claw, int wrist) {
  motorSet(3, -claw);
  motorSet(4, -wrist);
}
