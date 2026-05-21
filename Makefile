
.PHONY: build
build:
	@echo "Make sure you have: sudo apt install build-essential git git-lfs cmake libvtk9-dev"
	mkdir -p build
	cd build && cmake ../ && make
