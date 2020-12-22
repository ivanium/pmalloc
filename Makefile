all: lib test


lib:
	cd src && make

test:
	cd test && make

clean:
	cd src && make clean
	cd test && make clean


.PHONY: all lib test
