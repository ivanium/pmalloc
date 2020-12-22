OS := $(shell sh -c 'uname -s 2>/dev/null || echo not')

# Clean the default value of CC, CXX.
CC=
CXX=
# Detect proper C compiler by system OS.
ifndef CC
	ifeq ($(OS), Darwin)
		CC=clang
	else
		CC=gcc
	endif
endif
ifndef CXX
	ifeq ($(OS), Darwin)
		CXX=clang++
	else
		CXX=g++
	endif
endif

TARGET=Debug
# TARGET=Release

# Set proper CFLAGS by both CC and TARGET.
CFLAGS=-Wall -Wextra -std=gnu11
CXXFLAGS=-Wall -Wextra -std=gnu++11
# disable annoying warnings for debug
CFLAGS+= -Wwrite-strings -Woverloaded-virtual -Wno-sign-compare -fno-builtin-malloc -fno-builtin-free -fno-builtin-realloc -fno-builtin-calloc -fno-builtin-cfree -fno-builtin-memalign -fno-builtin-posix_memalign -fno-builtin-valloc -fno-builtin-pvalloc  -Wno-unused-result -fsized-deallocation   -DNO_FRAME_POINTER -fno-builtin
CXXFLAGS+= -Wwrite-strings -Woverloaded-virtual -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-reorder -fno-builtin-malloc -fno-builtin-free -fno-builtin-realloc -fno-builtin-calloc -fno-builtin-cfree -fno-builtin-memalign -fno-builtin-posix_memalign -fno-builtin-valloc -fno-builtin-pvalloc  -Wno-unused-result -fsized-deallocation -DNO_FRAME_POINTER -fno-builtin

ifeq ($(OS), Darwin)
	ifeq ($(TARGET), Debug)
		# CFLAGS for debug.
		ifeq ($(CC), clang)
			CFLAGS+= -O1 -g -fsanitize=address -fno-omit-frame-pointer
			CXXFLAGS+= -O1 -g -fsanitize=address -fno-omit-frame-pointer
		else
			CFLAGS+= -g
			CXXFLAGS+= -g
		endif  # CC
	else
		# CFLAGS for release.
		CFLAGS+= -O3
		CXXFLAGS+= -O3
	endif  # TARGET
else
#linux
	ifeq ($(TARGET), Debug)
		CFLAGS+= -g
		CXXFLAGS+= -g
	else
		CFLAGS+= -O3
		CXXFLAGS+= -O3
	endif  # TARGET
endif  # CC

# Set proper DYNAMIC_LIB by system OS.
ifeq ($(OS), Darwin)
	C_DYNAMIC_LIB=../bin/libpmalloc.dylib
	CXX_DYNAMIC_LIB=../bin/libpmalloc.dylib
	DYLIB_FLAGS=-dynamiclib
	LD_FLAGS=-L../bin -Xlinker -rpath -Xlinker ../bin
else
	C_DYNAMIC_LIB=../bin/libpmalloc.so
	CXX_DYNAMIC_LIB=../bin/libpmalloc.so
	DYLIB_FLAGS=--shared -fPIC
	LD_FLAGS=-L../bin -Wl,-rpath=../bin
endif

PROFILE=false
# PROFILE=true

ifeq ($(PROFILE), true)
	CFLAGS+= -lprofiler
	CXXFLAGS+= -lprofiler
endif