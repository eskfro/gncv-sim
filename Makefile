.PHONY: 2 v1

v1:
	cmake --build build -j8
	chmod +x build/simulator_v1
	./build/simulator_v1

2:
	chmod +x build/sim2d
	./build/sim2d 