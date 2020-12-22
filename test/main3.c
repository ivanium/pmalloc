///-*-C++-*-//////////////////////////////////////////////////////////////////
//
// Hoard: A Fast, Scalable, and Memory-Efficient Allocator
//        for Shared-Memory Multiprocessors
// Contact author: Emery Berger, http://www.cs.utexas.edu/users/emery
//
// Copyright (c) 1998-2000, The University of Texas at Austin.
//
// This library is free software; you can redistribute it and/or modify
// it under the terms of the GNU Library General Public License as
// published by the Free Software Foundation, http://www.fsf.org.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
//////////////////////////////////////////////////////////////////////////////

/**
 * @file threadtest.cpp
 *
 * This program does nothing but generate a number of kernel threads
 * that allocate and free memory, with a variable
 * amount of "work" (i.e. cycle wasting) in between.
 */

#ifndef _REENTRANT
#define _REENTRANT
#endif

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

// #include <google/profiler.h>

int niterations = 50; // Default number of iterations.
int nobjects = 30000; // Default number of objects.
int nthreads = 1;     // Default number of threads.
int work = 0;         // Default number of loop iterations.
int objSize = 1;

typedef struct Foo {
  int x;
  int y;
} Foo;

void init(Foo *f) {
  f->x = 14;
  f->y = 29;
}

void *worker() {
  int i, j;
  Foo **a;
  a = (Foo **)malloc(sizeof(Foo *) * (nobjects / nthreads));

  for (j = 0; j < niterations; j++) {
    for (i = 0; i < (nobjects / nthreads); i++) {
      a[i] = (Foo *)malloc(sizeof(Foo));
      init(a[i]);
#if 1
      for (volatile int d = 0; d < work; d++) {
        volatile int f = 1;
        f = f + f;
        f = f * f;
        f = f + f;
        f = f * f;
      }
#endif
      assert(a[i]);
    }

    for (i = 0; i < (nobjects / nthreads); i++) {
      free(a[i]);
#if 1
      for (volatile int d = 0; d < work; d++) {
        volatile int f = 1;
        f = f + f;
        f = f * f;
        f = f + f;
        f = f * f;
      }
#endif
    }
  }

  free(a);
  return NULL;
}

int main(int argc, char *argv[]) {
  pthread_t *threads;

  if (argc >= 2) {
    nthreads = atoi(argv[1]);
  }
  if (argc >= 3) {
    niterations = atoi(argv[2]);
  }
  if (argc >= 4) {
    nobjects = atoi(argv[3]);
  }
  if (argc >= 5) {
    work = atoi(argv[4]);
  }
  if (argc >= 6) {
    objSize = atoi(argv[5]);
  }

  printf("Running threadtest for %d threads, %d iterations, %d objects, %d "
         "work and %d objSize...\n",
         nthreads, niterations, nobjects, work, objSize);

  threads = (pthread_t *)malloc(sizeof(pthread_t) * nthreads);

  struct timeval stt;
  gettimeofday(&stt, NULL);

  // ProfilerStart("test3.prof");

  for (int i = 0; i < nthreads; i++) {
    pthread_create(threads + i, NULL, worker, NULL);
  }

  for (int i = 0; i < nthreads; i++) {
    pthread_join(threads[i], NULL);
  }

  double duration = 0.0;
  struct timeval edt;
  gettimeofday(&edt, NULL);
  duration = (edt.tv_sec - stt.tv_sec) + (edt.tv_usec - stt.tv_usec) / 1e6;
  printf("Time elapsed = %.2lfs\n", duration);

  // ProfilerStop();

  free(threads);

  return 0;
}
