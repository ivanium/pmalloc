include ../Makefile.inc

ifeq ($(PROFILE), true)
	LD_LIB=-lpthread -lprofiler
else
	LD_LIB=-lpthread
endif

all: libpmalloc

libpmalloc: pmalloc.h pmalloc.cpp malloc-intercept.cpp
	$(CXX) $(CXXFLAGS) $(DYLIB_FLAGS) -o $(CXX_DYNAMIC_LIB) -I. malloc-intercept.cpp pmalloc.cpp $(LD_LIB)

clean:
	-rm -rf ../bin/*
