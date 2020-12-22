#include <stdio.h>
#include "pmalloc.h"

using namespace PMALLOC;

int main() {
  PM_init();
  PM_stop();
  PM_recover();
  return 0;
}