
.PHONY: build
build:
	mkdir -p build
	cd build && cmake ../src && make
