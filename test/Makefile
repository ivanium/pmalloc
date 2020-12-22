include ../Makefile.inc

# export DYLD_LIBRARY_PATH=/Users/ivanium/learn/thesis/pmalloc/bin:DYLD_LIBRARY_PATH

ifeq ($(OS), Darwin)
	LINK_FLAGS=$(LD_FLAGS) -lpmalloc
else #linux
	LINK_FLAGS=
endif

ifeq ($(PROFILE), true)
	PROF_LIB=-lprofiler
else
	PROF_LIB=
endif

PMALLOC=true
ifeq ($(PMALLOC), true)
	PMALLOC_FLAGS=-I../src -L../bin -Wl,-rpath,../bin
else
	PMALLOC_FLAGS=
endif


all: main2 main3 main5

main2: main2.cpp
	$(CXX) $(CXXFLAGS) -o ../bin/test2 $(LINK_FLAGS) main2.cpp -lpthread

main3: main3.c
	$(CC) $(CFLAGS) -o ../bin/test3 $(LINK_FLAGS) main3.c -lpthread $(PROF_LIB)

main3_persist: main3_persist.cpp
	$(CXX) $(CXXFLAGS) $(PMALLOC_FLAGS) -o ../bin/test3_persist $(LINK_FLAGS) main3_persist.cpp -lpthread -lpmalloc $(PROF_LIB)

main5: main5.c
	$(CC) $(CFLAGS) -o ../bin/test5 $(LINK_FLAGS) main5.c -lpthread

scalability: scalability.c
	$(CC) $(CFLAGS) -Wno-unused-parameter -o ../bin/scalability $(LINK_FLAGS) scalability.c -lpthread -lm

larson: larson.cpp
	$(CXX) $(CXXFLAGS) -o ../bin/larson $(LINK_FLAGS) larson.cpp -lpthread

persist_region: persist_region.cpp
	$(CXX) $(CXXFLAGS) $(PMALLOC_FLAGS) -o ../bin/persist_region $(LINK_FLAGS) persist_region.cpp -lpthread -lpmalloc

# for Mac OS evaluation
baseline: main2.cpp main3.c main5.c scalability.c larson.cpp
	$(CXX) $(CXXFLAGS) -o ../bin/test2_baseline main2.cpp -lpthread
	$(CC) $(CFLAGS) -o ../bin/test3_baseline main3.c -lpthread $(PROF_LIB)
	$(CC) $(CFLAGS) -o ../bin/test5_baseline main5.c -lpthread
	$(CC) $(CFLAGS) -Wno-unused-parameter -o ../bin/scalability_baseline scalability.c -lpthread -lm
	$(CXX) $(CXXFLAGS) -o ../bin/larson larson.cpp -lpthread

run:
	../bin/test2
	../bin/test3
	../bin/test5

run2:
	../bin/test2

run3:
	../bin/test3

run5:
	../bin/test5

clean:
	rm ../bin/test*
