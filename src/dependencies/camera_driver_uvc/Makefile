.PHONY: build

all:
	build

init:
	cmake -B build -G Ninja -DCMAKE_COLOR_DIAGNOSTICS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

build:
	cmake --build build

run: build
	./build/camera_fix_uvc