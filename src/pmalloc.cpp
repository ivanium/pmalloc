#include <assert.h>
#include <pthread.h>
#if defined HAVE_STDINT_H
#include <stdint.h>
#endif
#include <errno.h>
#include <fcntl.h>
#include <new>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "pmalloc.h"
namespace PMALLOC {
const int LOG_BASE = 1;         // log2(BASE)
const int LOG_INITIAL_BASE = 3; // log2(INITIAL_BASE)
const int BASE = 1 << LOG_BASE;
const int INITIAL_BASE = 1 << LOG_INITIAL_BASE;
const double K = 1.2;
const double FRACTION = 1.0 / 4;
const int MAX_MEMORY_PER_TLAB = 8 * 1024 * 1024; // 8MB for thread local buffer
const size_t MAX_SMALL_OBJ = 256;
const size_t STATIC_BUF_SIZE = 1 << 20; // 1MB

const size_t MAGIC_BYTES_SINGLEBLOCK = 0x000FC0DE;
const size_t MAGIC_BYTES_SUPERBLOCK = 0x001FC0DE;

const size_t SUPERBLOCK_SIZE = 256 * 4096; // 256 PAGEs in a SuperBlock (1MB)
const size_t SUPERBLOCK_MASK = ~(SUPERBLOCK_SIZE - 1);
int CLASS_COUNT;

static double __gHeap[sizeof(Heap) / sizeof(double) + 1];
static __thread double __tHeap[sizeof(Heap) / sizeof(double) + 1];
Heap *globalHeap = (Heap *)&__gHeap;
static __thread Heap *threadHeap;

static Region *region;

static __thread void *SingleAddrs[1000];
static __thread int saCnt;

bool initialized = false;
static __thread bool thread_initialized = false;

#define TABLE_SIZE 32
void *ASYNC_clflush_table[TABLE_SIZE];
bool stop;

pthread_mutex_t init_mtx = PTHREAD_MUTEX_INITIALIZER;

#define isValidAlign(align)                                                    \
  ((align) > sizeof(void *) && !((align) & ((align)-1)))

#define roundUpToAlign(addr, align)                                            \
  (((size_t)(addr) + (align - 1)) & ~(align - 1))

#define setSelfPtr(memAddr, metaAddr)                                          \
  *(void **)((char *)(memAddr) - sizeof(void *)) = (metaAddr)

inline size_t roundUpSize(size_t size) {
  if (size < INITIAL_BASE) {
    return INITIAL_BASE;
  } else if ((size & (size - 1)) == 0) {
    return size;
  }
  size_t bit;
  asm("bsrq %1, %0" : "=r"(bit) : "r"(size >> LOG_INITIAL_BASE));
  return INITIAL_BASE << (bit + 1);
}

inline int getSizeClass(size_t size) {
  if (size < INITIAL_BASE) {
    return 0;
  }
  size_t bit;
  asm("bsrq %1, %0" : "=r"(bit) : "r"(size >> LOG_INITIAL_BASE));
  bit += (size & (size - 1)) != 0;
  return bit / LOG_BASE;
}

SingleBlock::SingleBlock(size_t _alignedSize, size_t _size)
    : alignedSize(_alignedSize), size(_size),
      magicBytes(MAGIC_BYTES_SINGLEBLOCK ^ (size_t) this) {}

inline SuperBlock *getSuperBlock(void *addr) {
  return (SuperBlock *)((size_t)addr & SUPERBLOCK_MASK);
}

SuperBlock::SuperBlock(size_t _blockSize, size_t _sizeClassNum)
    : parent(NULL), inuse_mem(0), prev(NULL), next(NULL),
      sizeClassNum(_sizeClassNum), blockSize(_blockSize),
      maxBlockCnt((SUPERBLOCK_SIZE - sizeof(SuperBlock)) /
                  (sizeof(Block) + _blockSize)),
      freeList(NULL), blockHdrs((Block *)((char *)this + sizeof(SuperBlock))),
      blocks((char *)blockHdrs + (sizeof(Block)) * maxBlockCnt), nextBIdx(0),
      freeBCnt(0), magicBytes(MAGIC_BYTES_SUPERBLOCK ^ (size_t) this) {
  lock_init(mtx);
  bitmap = (char *)PM_internal_mmap(roundUpToAlign(maxBlockCnt, 8));
  PERSIST_RANGE(this, 3 * sizeof(size_t));
}

void *SuperBlock::allocBlock(size_t size) {
  void *res;
  if (freeBCnt > 0) { // alloc from freelist first to reduce fragments
    inuse_mem += size;
    parent->inuse_mem += size;
    freeBCnt--;

    Block *b = (Block *)freeList;
    int blockIdx = b - blockHdrs;
    freeList = b->next;
    b->size = size;
    res = (Block **)(blocks + blockIdx * blockSize);
    setBit(bitmap, blockIdx);
    return res;
  } else if (nextBIdx <
             maxBlockCnt) { // alloc new blocks first for wear leveling
    inuse_mem += size;
    parent->inuse_mem += size;
    res = (Block **)(blocks + nextBIdx * blockSize);
    blockHdrs[nextBIdx].size = size;
    setBit(bitmap, nextBIdx);
    nextBIdx++;
    return res;
  } else {
    return NULL;
  }
}

void SuperBlock::freeBlock(void *addr) {
  int blockIdx = ((size_t)addr - (size_t)blocks) / blockSize;
  Block *b = blockHdrs + blockIdx;
  inuse_mem -= b->size;
  parent->inuse_mem -= b->size;
  b->next = freeList;
  freeList = (void *)b;
  freeBCnt++;
  clrBit(bitmap, blockIdx);
  if (inuse_mem == 0) {
    freeBCnt = 0;
    nextBIdx = 0;
  }
}

void SuperBlock::detachHeap() {
  parent->inuse_mem -= inuse_mem;
  parent->alloc_mem -= SUPERBLOCK_SIZE;

  if (prev != NULL) {
    prev->next = next;
  } else {
    parent->SBHeads[sizeClassNum] = next;
  }
  if (next != NULL) {
    next->prev = prev;
  } else {
    parent->SBTails[sizeClassNum] = prev;
  }
  prev = next = NULL;
  parent = NULL;
}

void SuperBlock::attachHeap(Heap *h) {
  parent = h;
  h->inuse_mem += inuse_mem;
  h->alloc_mem += SUPERBLOCK_SIZE;

  SuperBlock *insertAfter = h->SBTails[sizeClassNum];
  while (insertAfter != NULL && inuse_mem < insertAfter->inuse_mem) {
    insertAfter = insertAfter->prev;
  }
  if (insertAfter != NULL) {
    next = insertAfter->next;
    prev = insertAfter;
    insertAfter->next = this;
    if (next != NULL) {
      next->prev = this;
    } else {
      h->SBTails[sizeClassNum] = this;
    }
  } else { // insert at list head
    prev = NULL;
    next = h->SBHeads[sizeClassNum];
    if (h->SBHeads[sizeClassNum] != NULL) {
      h->SBHeads[sizeClassNum]->prev = this;
    } else { // list is empty
      h->SBTails[sizeClassNum] = this;
    }
    h->SBHeads[sizeClassNum] = this;
  }
}

Heap::Heap() {
  lock_init(mtx);
  SBHeads = (SuperBlock **)PM_internal_mmap(sizeof(SuperBlock *) * CLASS_COUNT);
  SBTails = (SuperBlock **)PM_internal_mmap(sizeof(SuperBlock *) * CLASS_COUNT);
  inuse_mem = 0;
  alloc_mem = sizeof(Heap) + 2 * sizeof(SuperBlock *) * CLASS_COUNT;

  for (int i = 0; i < CLASS_COUNT; i++) {
    SBHeads[i] = NULL;
    SBTails[i] = NULL;
  }
  for (int i = 0; i < 6; i++) {
    threadCache[i] = NULL;
  }
}

void *Heap::allocBlock(size_t alignedSize, size_t size) {
  void *res;
  int sizeClass = getSizeClass(alignedSize);
  SuperBlock *tailSB = SBTails[sizeClass];
  while (tailSB != NULL) { // try from fullest SB to reduce fragment
    res = tailSB->allocBlock(size);
    if (res != NULL) {
      return res;
    }
    tailSB = tailSB->prev;
  }
  return NULL;
}

// Region::Region()
//     : base_addr(0), curr_addr(NULL), sinBNextIdx(0), sinBFreeCnt(0),
//       supBNextIdx(0), supBFreeCnt(0), sinBFreelist(NULL), supBFreelist(NULL),
//       bitmapLen(0) {
//   memset(sinBHdrs, 0, sizeof(sinBHdrs));
//   memset(supBHdrs, 0, sizeof(supBHdrs));
//   memset(sinBBitmap, 0, sizeof(sinBBitmap));
//   memset(supBBitmap, 0, sizeof(supBBitmap));
//   memset(bitmapChunk, 0, sizeof(bitmapChunk));
// }
void clearRegion(Region *region) { memset(region, 0, sizeof(Region)); }
SingleBlock *Region::allocSingleBlockHdr() {
  SingleBlock *res;
  if (sinBFreeCnt > 0) {
    res = (SingleBlock *)sinBFreelist;
    sinBFreelist = *(SingleBlock **)sinBFreelist;
    int idx = res - (SingleBlock *)sinBHdrs;
    setBit(sinBBitmap, idx);
    return res;
  }
  res = sinBHdrs + sinBNextIdx;
  setBit(sinBBitmap, sinBNextIdx);
  sinBNextIdx++;
  return res;
}
void Region::freeSingleBlockHdr(SingleBlock *sinb) {
  *(SingleBlock **)sinb = sinBFreelist;
  sinBFreelist = sinb;
  int idx = sinb - (SingleBlock *)sinBHdrs;
  clrBit(sinBBitmap, idx);
}
SuperBlock *Region::allocSuperBlockHdr() {
  SuperBlock *res;
  if (supBFreeCnt > 0) {
    res = (SuperBlock *)supBFreelist;
    supBFreelist = *(SuperBlock **)supBFreelist;
    int idx = res - (SuperBlock *)supBHdrs;
    setBit(supBBitmap, idx);
    return res;
  }
  res = supBHdrs + supBNextIdx;
  setBit(supBBitmap, supBNextIdx);
  supBNextIdx++;
  return res;
}
void Region::freeSuperBlockHdr(SuperBlock *supb) {
  *(SuperBlock **)supb = supBFreelist;
  supBFreelist = supb;
  int idx = supb - (SuperBlock *)supBHdrs;
  clrBit(supBBitmap, idx);
}
char *Region::allocBitmap(size_t cnt) {
  char *res = bitmapChunk + bitmapLen;
  bitmapLen += cnt;
  return res;
}

inline void __init() {
  if (PREDICT_FALSE(!thread_initialized)) {
    if (!initialized) { // init global heap
      pthread_mutex_lock(&init_mtx);
      if (!initialized) {
        CLASS_COUNT = getSizeClass(SUPERBLOCK_SIZE) + 1;
        new (globalHeap) Heap();
        initialized = true;
      }
      pthread_mutex_unlock(&init_mtx);
    }

    threadHeap = (Heap *)&__tHeap;
    new (threadHeap) Heap();
    thread_initialized = true;
  }
}

inline void *PM_align_malloc(size_t size, size_t alignment) {
  __init();
  if (size == 0) {
    return NULL;
  }
  size_t alignedSize = roundUpSize(size + alignment - 1);
  void *res = NULL;
  if (alignedSize > SUPERBLOCK_SIZE / 2) {
    alignedSize = size + alignment - 1;
    SingleBlock *sinb = (SingleBlock *)PM_internal_mmap(
        sizeof(SingleBlock) + sizeof(void *) + alignedSize);
    if (sinb == NULL || sinb == MAP_FAILED) { // check size or alignedSize
      return NULL;
    }
    new (sinb) SingleBlock(alignedSize, size);
    res = (void *)roundUpToAlign(
        (char *)sinb + sizeof(SingleBlock) + sizeof(void *), alignment);
    SingleAddrs[saCnt] = res;
    saCnt++;
    setSelfPtr(res, sinb);
    PERSIST_RANGE(res, sizeof(SingleBlock));
    return res;
  } else {
    int sc = getSizeClass(alignedSize);
    void **cacheListHead = threadHeap->threadCache + sc;
    if (alignedSize <= MAX_SMALL_OBJ && *cacheListHead != NULL) {
      res = *cacheListHead;
      *cacheListHead = *(void **)res;
      *(void **)res = NULL;
      threadHeap->freeTCSize -= alignedSize;
      return res;
    }
    lock(threadHeap->mtx);
    void *b = threadHeap->allocBlock(alignedSize, size);
    unlock(threadHeap->mtx);
    if (b == NULL) { // need SB from Global Heap
      lock(globalHeap->mtx);
      b = globalHeap->allocBlock(alignedSize, size);
      unlock(globalHeap->mtx);
      if (b != NULL) {
        SuperBlock *sb = getSuperBlock(b);
        lock(sb->mtx);
        if (sb->parent == globalHeap) {
          lock(globalHeap->mtx);
          sb->detachHeap();
          unlock(globalHeap->mtx);
          lock(threadHeap->mtx);
          sb->attachHeap(threadHeap);
          unlock(threadHeap->mtx);
        }
        unlock(sb->mtx);
      } else { // global heap is full
        SuperBlock *sb = (SuperBlock *)PM_internal_mmap(SUPERBLOCK_SIZE);
        if (sb == NULL || sb == MAP_FAILED) {
          return NULL;
        }
        new (sb) SuperBlock(alignedSize, sc);
        lock(threadHeap->mtx);
        sb->attachHeap(threadHeap);
        b = sb->allocBlock(size);
        unlock(threadHeap->mtx);
      }
    }
    if (b != NULL) {
      res = (void *)roundUpToAlign(b, alignment);
    }
    return res;
  }
  return NULL; // impossible to reach here
}

void *PM_malloc(size_t size) {
  return PM_align_malloc(size, 1);
}

void *PM_calloc(size_t n, size_t size) {
  __init();
  if (size == 0) {
    return NULL;
  }
  void *res = PM_align_malloc(n * size, 1);
  if (res != NULL) {
    memset(res, 0, n * size);
  }
  return res;
}

void PM_free(void *addr) {
  __init();
  if (addr == NULL) {
    return;
  }

  for (int i = 0; i < saCnt; i++) {
    if (SingleAddrs[i] == addr) {
      SingleBlock *sinb = *(SingleBlock **)((char *)addr - sizeof(void *));
      munmap(sinb, sizeof(SingleBlock) + sizeof(void *) + sinb->alignedSize);
      return;
    }
  }

  SuperBlock *sb = getSuperBlock(addr);
  if (sb->magicBytes ==
      (MAGIC_BYTES_SUPERBLOCK ^ (size_t)sb)) { // small mem block is more common
    size_t obj_size = sb->blockSize;
    if (obj_size <= MAX_SMALL_OBJ &&
        threadHeap->freeTCSize < MAX_MEMORY_PER_TLAB) {
      void **cacheListHead = threadHeap->threadCache + getSizeClass(obj_size);
      *(void **)addr = *cacheListHead;
      *cacheListHead = addr;
      threadHeap->freeTCSize += obj_size;
      return;
    }

    Heap *h;
    lock(sb->mtx);
    sb->freeBlock(addr);
    h = sb->parent;
    unlock(sb->mtx);

    // check transfer SuperBlock
    lock(h->mtx);
    if (h != globalHeap && h->inuse_mem + K * SUPERBLOCK_SIZE < h->alloc_mem &&
        h->inuse_mem < (1 - FRACTION) * h->alloc_mem) {
      // this heap is too empty, return SB to global
      SuperBlock *emptiestSB = NULL;
      for (int i = 0; i < CLASS_COUNT; i++) {
        if (h->SBTails[i] != NULL &&
            (emptiestSB == NULL ||
             emptiestSB->inuse_mem > h->SBTails[i]->inuse_mem)) {
          emptiestSB = h->SBTails[i];
        }
      }
      if (emptiestSB) {
        lock(emptiestSB->mtx);
        emptiestSB->detachHeap();
        unlock(h->mtx);
        lock(globalHeap->mtx);
        emptiestSB->attachHeap(globalHeap);
        unlock(globalHeap->mtx);
        unlock(emptiestSB->mtx);
      } else {
        unlock(h->mtx);
      }
    } else {
      unlock(h->mtx);
    }
    return;
  }
}

void *PM_realloc(void *addr, size_t size) {
  __init();
  void *res = PM_align_malloc(size, 1);
  if (addr != NULL && res != NULL) {
    SuperBlock *sb = getSuperBlock(addr);
    if (sb->magicBytes == (MAGIC_BYTES_SUPERBLOCK ^ (size_t)sb)) {
      memcpy(res, addr, min(size, sb->blockSize));
    } else {
      SingleBlock *sinb = *(SingleBlock **)((char *)addr - sizeof(void *));
      memcpy(res, addr, min(size, sinb->size));
    }
    PM_free(addr);
  }
  return res;
}

int PM_memalign(void **addrPtr, size_t alignment, size_t size) {
  __init();
  void *res = NULL;
  *addrPtr = NULL;
  if (!isValidAlign(alignment)) {
    return EINVAL;
  }
  res = PM_align_malloc(size, alignment);
  if (res == NULL) {
    return ENOMEM;
  }
  *addrPtr = res;
  return 0;
}

// bitmap
void setBit(char *bitmap, int idx) {
  bitmap[idx >> 3] |= (1 << (idx & BYTE_MASK));
  PERSIST(bitmap + (idx >> 3));
}

void clrBit(char *bitmap, int idx) {
  bitmap[idx >> 3] &= ~(1 << (idx & BYTE_MASK));
  PERSIST(bitmap + (idx >> 3));
}

bool tstBit(char *bitmap, int idx) {
  return bitmap[idx >> 3] & (1 << (idx & BYTE_MASK));
}

// persistent related functions
void PM_init() {
  __map_persistent_region();
  __init();
}

void *PM_internal_mmap(size_t size) {
  // mock
  if (size == SUPERBLOCK_SIZE) { // try to mmap SuperBlock with alignment.
    void *pre_addr = mmap(NULL, 2 * SUPERBLOCK_SIZE, PROT_READ | PROT_WRITE,
                          MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    void *addr = (void *)roundUpToAlign(pre_addr, SUPERBLOCK_SIZE);
    size_t prelog = (size_t)addr - (size_t)pre_addr;
    size_t epilog = SUPERBLOCK_SIZE - prelog;
    munmap(pre_addr, prelog);
    munmap((char *)addr + SUPERBLOCK_SIZE, epilog);
    return addr;
  }
  return mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS,
              -1, 0);
}

int PM_internal_munmap(void *addr, size_t len) {
  // mock
  return munmap(addr, len);
}

void PM_stop() { __close_persistent_region(); }
void PM_recover() { __remap_persistent_region(); }

// from Makalu
static int FD = 0;
void __map_transient_region() {
  char *ret = (char *)mmap((void *)0, FILESIZE, PROT_READ | PROT_WRITE,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (ret == MAP_FAILED) {
    __PM_printf("Mmap failed");
    exit(1);
  }
  region = (Region *)ret;
  clearRegion(region);
  region->base_addr = ret;
  region->curr_addr = (char *)((size_t)region + sizeof(Region));
  __PM_printf("Addr: %p\n", ret);
  __PM_printf("Base_addr: %p\n", region->base_addr);
  __PM_printf("Current_addr: %p\n", region->curr_addr);
}

void __map_persistent_region() {
  FD = open(HEAPFILE, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
  off_t offt = lseek(FD, FILESIZE - 1, SEEK_SET);
  assert(offt != -1);
  int result = write(FD, "", 1);
  assert(result != -1);

  void *addr = mmap(0, FILESIZE, PROT_READ | PROT_WRITE, MAP_SHARED, FD, 0);
  assert(addr != MAP_FAILED);

  *(void **)addr = addr;
  clflush(addr);
  region = (Region *)addr;
  region->base_addr = (char *)addr;
  // address to remap to, the root pointer to gc metadata,
  // and the curr pointer at the end of the day
  region->curr_addr = (char *)((size_t)addr + sizeof(Region));
  region->close_addr = region->curr_addr;
  clflush(region->close_addr);
  __PM_printf("Addr: %p\n", addr);
  __PM_printf("Base_addr: %p\n", region->base_addr);
  __PM_printf("Current_addr: %p\n", region->curr_addr);
}

void __remap_persistent_region() {
  FD = open(HEAPFILE, O_RDWR, S_IRUSR | S_IWUSR);
  off_t offt = lseek(FD, FILESIZE - 1, SEEK_SET);
  assert(offt != -1);
  int result = write(FD, "", 1);
  assert(result != -1);
  offt = lseek(FD, 0, SEEK_SET);
  assert(offt == 0);
  void *forced_addr;

  int bytes_read = read(FD, &forced_addr, sizeof(void *));
  if (bytes_read <= 0) {
    __PM_printf(
        "Something went wrong when trying to retrieve the forced address.\n");
  }

  void *addr = mmap(forced_addr, FILESIZE, PROT_READ | PROT_WRITE,
                    MAP_SHARED, FD, 0);
  assert(addr != MAP_FAILED);
  assert(forced_addr == addr);

  region = (Region *)addr;
  region->curr_addr = region->close_addr;
  __PM_printf("Forced Addr: %p\n", (void *)forced_addr);
  __PM_printf("Addr: %p\n", addr);
  __PM_printf("Base_addr: %p\n", region->base_addr);
  __PM_printf("Curr_addr: %p\n", region->curr_addr);
}

void __close_persistent_region() {
  region->close_addr = region->curr_addr;
  clflush(region->close_addr);

  __PM_printf("At the end current addr: %p\n", region->curr_addr);

  unsigned long space_used =
      ((unsigned long)region->curr_addr - (unsigned long)region->base_addr);
  unsigned long remaining_space =
      ((unsigned long)FILESIZE - space_used) / (1024 * 1024);
  __PM_printf("Space Used(rounded down to MiB): %ld, Remaining(MiB): %ld\n",
              space_used / (1024 * 1024), remaining_space);
  close(FD);
  munmap(region->base_addr, FILESIZE);
}

void __close_transient_region() {
  __PM_printf("At the end current addr: %p\n", region->curr_addr);
  unsigned long space_used =
      ((unsigned long)region->curr_addr - (unsigned long)region->base_addr);
  unsigned long remaining_space =
      ((unsigned long)FILESIZE - space_used) / (1024 * 1024);
  __PM_printf("Space Used(rounded down to MiB): %ld, Remaining(MiB): %ld\n",
              space_used / (1024 * 1024), remaining_space);
}

void __store_heap_start(void *root) {
  region->heap_start = root;
  clflush(region->heap_start);
}

void *__fetch_heap_start() { return region->heap_start; }

int __nvm_region_allocator(void **memptr, size_t alignment, size_t size) {
  char *next;
  char *res;
  if (size == 0)
    return 1;

  if (((alignment & (~alignment + 1)) !=
       alignment) || // should be multiple of 2
      (alignment < sizeof(void *)))
    return 1; // should be atleast the size of void*
  size_t aln_adj = (size_t)region->curr_addr & (alignment - 1);

  if (aln_adj != 0)
    region->curr_addr += (alignment - aln_adj);

  res = region->curr_addr;
  next = region->curr_addr + size;
  if (next > region->base_addr + FILESIZE) {
    __PM_printf("\n----Region Manager: out of space in mmaped file-----\n");
    return 1;
  }
  region->curr_addr = next;
  region->close_addr = next;
  clflush(region->close_addr);
  *memptr = res;

  return 0;
}

// printf func without malloc
void __PM_printf(const char *format, ...) {
#define BUFSZ 1024
  va_list args;
  char buf[BUFSZ + 1];

  va_start(args, format);
  buf[BUFSZ] = 0x15;
  (void)vsnprintf(buf, BUFSZ, format, args);
  va_end(args);
  if (buf[BUFSZ] != 0x15)
    abort();
  if (write(STDOUT_FILENO, buf, strlen(buf)) < 0)
    abort();
#undef BUFSZ
}
} // namespace PMALLOC
