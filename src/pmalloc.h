#ifndef __PMALLOC_H
#define __PMALLOC_H

#include <cstdlib>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>

namespace PMALLOC {
// constants
#define CACHE_LINE_SIZE 64
#define SB_BITS 16
// persistent memory root
#define FILESIZE 5 * 1024 * 1024 * 1024ULL + 24
#define HEAPFILE "/dev/shm/gc_heap.dat"

// #define MUTEX_LOCK
#ifdef MUTEX_LOCK
typedef pthread_mutex_t lock_t;
#define lock_init(m) pthread_mutex_init(&(m), NULL)
#define lock(m) assert(pthread_mutex_lock(&(m)) == 0)
#define unlock(m) pthread_mutex_unlock(&(m))
#define trylock(m) pthread_mutex_trylock(&(m))
#define min(x, y) ((x) < (y) ? (x) : (y))
#else
// manual spin lock for better performance
typedef int pthread_spinlock_t;
static int pthread_spin_init(pthread_spinlock_t *lock, int shared) {
  __asm__ __volatile__("" ::: "memory");
  *lock = 0;
  return 0;
}

static int pthread_spin_lock(pthread_spinlock_t *lock) {
  while (1) {
    for (int i = 0; i < 8192; i++) {
      if (__sync_bool_compare_and_swap(lock, 0, 1)) {
        return 0;
      }
    }
    sched_yield();
  }
}

static int pthread_spin_trylock(pthread_spinlock_t *lock) {
  if (__sync_bool_compare_and_swap(lock, 0, 1)) {
    return 0;
  }
  return EBUSY;
}

static int pthread_spin_unlock(pthread_spinlock_t *lock) {
  __asm__ __volatile__("" ::: "memory");
  *lock = 0;
  return 0;
}
#define lock_init(m) pthread_spin_init(&(m), PTHREAD_PROCESS_SHARED)
typedef pthread_spinlock_t lock_t;
#define lock(m) assert(pthread_spin_lock(&(m)) == 0)
#define unlock(m) pthread_spin_unlock(&(m))
#define trylock(m) pthread_spin_trylock(&(m))
#define min(x, y) ((x) < (y) ? (x) : (y))
#endif

// export POSIX APIs
void *PM_internal_malloc(size_t size, size_t alignment);
void *PM_malloc(size_t size);
void *PM_calloc(size_t n, size_t size);
void PM_free(void *pointer);
void *PM_realloc(void *pointer, size_t size);
int PM_memalign(void **memoryPointer, size_t alignment, size_t size);

// Persistent Memory Related APIs
void PM_init();
void PM_stop();
void PM_recover();
void *PM_internal_mmap(size_t size);
int PM_internal_munmap(void *addr, size_t len);

// mmap anynomous
void __map_transient_region();
// mmap file
void __map_persistent_region();
void __remap_persistent_region();

// persist the curr and base address
void __close_persistent_region();
// print the status
void __close_transient_region();
// store heap root
void __store_heap_start(void *);
// retrieve heap root
void *__fetch_heap_start();

int __nvm_region_allocator(void ** /*ret */, size_t /* alignment */,
                           size_t /*size */);

// internal defines
struct SingleBlock;
struct SuperBlock;
struct Heap;

struct SingleBlock {
  size_t alignedSize; // >= size
  size_t size;        // size of data

  size_t magicBytes;
  SingleBlock(size_t alignedSize, size_t size);
};

union Block {
  size_t size; // size of data
  void *next;  // next free block in freeList
};

struct SuperBlock {
  const size_t blockSize;
  const int sizeClassNum;
  const int maxBlockCnt;
  void *freeList;
  Block *blockHdrs;
  char *blocks;

  lock_t mtx;
  Heap *parent;
  size_t inuse_mem;
  SuperBlock *prev;
  SuperBlock *next;

  int nextBIdx;
  int freeBCnt;

  char *bitmap;

  size_t magicBytes;
  SuperBlock(size_t sizePerBlock, size_t sizeClazz);
  void *allocBlock(size_t size);
  void freeBlock(void *addr);
  void detachHeap();
  void attachHeap(Heap *heap);
};
inline SuperBlock *getSuperBlock(void *addr);

struct Heap {
#define CACHE_SIZE 6
  lock_t mtx;
  void *threadCache[CACHE_SIZE]; // local cache for obj_size in [8, 16, 32, 64,
                                 // 128, 256]
  size_t freeTCSize;

  size_t inuse_mem;
  size_t alloc_mem;

  SuperBlock **SBHeads;
  SuperBlock **SBTails;
  Heap();
  void *allocBlock(size_t alignedSize, size_t size);
#undef CACHE_SIZE
};

struct Region {
#define HDR_CNT 1024
#define BITMAP_SIZE (2 * 1024 * 1024)
  char *base_addr;
  char *close_addr;
  void *heap_start;
  char *curr_addr;
  int sinBNextIdx;
  int sinBFreeCnt;
  int supBNextIdx;
  int supBFreeCnt;
  SingleBlock *sinBFreelist;
  SuperBlock *supBFreelist;
  SingleBlock sinBHdrs[HDR_CNT];
  SuperBlock supBHdrs[HDR_CNT];
  char sinBBitmap[HDR_CNT / 8];
  char supBBitmap[HDR_CNT / 8];
  int bitmapLen;
  char bitmapChunk[BITMAP_SIZE];
#undef HDR_CNT
#undef BITMAP_SIZE
  // Region();
  SingleBlock *allocSingleBlockHdr();
  SuperBlock *allocSuperBlockHdr();
  void freeSingleBlockHdr(SingleBlock *sinb);
  void freeSuperBlockHdr(SuperBlock *supb);
  char *allocBitmap(size_t cnt);
};

size_t roundUpSize(size_t size);
int getSizeClass(size_t size);

// printf without `malloc` to avoid recursive malloc call
void __PM_printf(const char *str...);

// bitmap related funcs
#define BYTE_MASK 0b111
void setBit(char *bitmap, int idx);
void clrBit(char *bitmap, int idx);
bool tstBit(char *bitmap, int idx);

// persistent operations
// #define NOFENCE
static inline void mfence() {
#ifndef NOFENCE
  asm volatile("mfence" ::: "memory");
#endif
}

static inline void clflush(void *addr) {
  asm volatile("clflush %0" : "+m"(addr));
}

static inline void clflush_range(void *addr, size_t size) {
  uintptr_t start = (uintptr_t)addr & ~(CACHE_LINE_SIZE - 1);
  for (; start < (uintptr_t)addr + size; start += CACHE_LINE_SIZE) {
    clflush((void *)start);
  }
}

#define NOFLUSH // for DRAM senario simulation

#ifdef NOFLUSH
#define PERSIST(addr)                                                          \
  do {                                                                         \
  } while (0)
#define PERSIST_RANGE(addr, size)                                              \
  do {                                                                         \
  } while (0)
#else
#define PERSIST(addr)                                                          \
  do {                                                                         \
    mfence();                                                                  \
    clflush(addr);                                                             \
    mfence();                                                                  \
  } while (0)
#define PERSIST_RANGE(addr, size)                                              \
  do {                                                                         \
    mfence();                                                                  \
    clflush_range(addr, size);                                                 \
    mfence();                                                                  \
  } while (0)
#endif

#ifdef HAVE_BUILTIN_EXPECT
#define PREDICT_TRUE(x) __builtin_expect(!!(x), 1)
#define PREDICT_FALSE(x) __builtin_expect(!!(x), 0)
#else
#define PREDICT_TRUE(x) (x)
#define PREDICT_FALSE(x) (x)
#endif
} // namespace PMALLOC

#endif //__PMALLOC_H