
.PHONY: build
build:
	@echo "Make sure you have: sudo apt install build-essential git git-lfs cmake libvtk9-dev"
	mkdir -p build
	cd build && cmake ../ && make

.PHONY: vtk
vtk:
	@echo "sudo apt install build-essential cmake cmake-curses-gui mesa-common-dev mesa-utils freeglut3-dev ninja-build"
	mkdir -p build
	cd build && if ! [ -f VTK-9.4.2.tar.gz ]; then wget https://vtk.org/files/release/9.4/VTK-9.4.2.tar.gz; fi
	cd build && tar -xvf VTK-9.4.2.tar.gz
	cd build && mkdir -p vtk-build
	cd build/vtk-build && cmake ../VTK-9.4.2/ -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
	cd build/vtk-build && make -j8
	cd build/vtk-build && sudo make install
