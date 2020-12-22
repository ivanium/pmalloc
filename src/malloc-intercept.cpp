#include "pmalloc.h"

using namespace PMALLOC;

#define RECURSIVE_GUARD
#ifdef RECURSIVE_GUARD
namespace {
__thread bool in_malloc = false;

void ENTER() {
  if (PREDICT_FALSE(in_malloc)) {
    __PM_printf("recursive call.\n");
    abort();
  }
  in_malloc = true;
}

void EXIT() {
  if (PREDICT_FALSE(!in_malloc)) {
    __PM_printf("exit before enter!\n");
    abort();
  }
  in_malloc = false;
}
} // namespace

extern "C" {
void *malloc(size_t size) {
  ENTER();
  void *p = PM_malloc(size);
  EXIT();
  return p;
}

void *calloc(size_t n, size_t size) {
  ENTER();
  void *p = PM_calloc(n, size);
  EXIT();
  return p;
}

void free(void *ptr) {
  ENTER();
  PM_free(ptr);
  EXIT();
}

void *realloc(void *addr, size_t size) {
  ENTER();
  void *p = PM_realloc(addr, size);
  EXIT();
  return p;
}

int posix_memalign(void **addrPtr, size_t align, size_t size) {
  ENTER();
  int result = PM_memalign(addrPtr, align, size);
  EXIT();
  return result;
}

void *valloc(size_t size) {
  __PM_printf("valloc size: %d\nvalloc is deprecated!\n", size);
  abort();
}

void *memalign(size_t boundary, size_t size) {
  __PM_printf("memalign boundary: %d, size: %d\nmemalign is deprecated!\n", boundary, size);
  abort();
}
} // extern "C"
#else // buggy `alias to undefined symbol`
#define WEAK(x) __attribute__((weak, alias(#x)))
#ifndef __THROW
#define __THROW
#endif

#define PM_PREFIX(x) PM_##x

#define WEAK_REDEF1(type, fname, arg1)                                         \
  type fname(arg1) __THROW WEAK(PM_##fname)
#define WEAK_REDEF2(type, fname, arg1, arg2)                                   \
  type fname(arg1, arg2) __THROW WEAK(PM_##fname)
#define WEAK_REDEF3(type, fname, arg1, arg2, arg3)                             \
  type fname(arg1, arg2, arg3) __THROW WEAK(PM_##fname)

extern "C" {
WEAK_REDEF1(void *, malloc, size_t);
WEAK_REDEF1(void, free, void *);
WEAK_REDEF2(void *, calloc, size_t, size_t);
WEAK_REDEF2(void *, realloc, void *, size_t);
int posix_memalign(void **, size_t, size_t) __THROW WEAK(PM_memalign);
}
#endif