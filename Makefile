.PHONY: 2d v1 playback playback3d clean-v1 test

v1:
	cmake --build build -j8
	chmod +x build/simulator_v1
	./build/simulator_v1

2d:
	chmod +x build/sim2d
	./build/sim2d

playback:
	cmake --build build -j8 --target playback_2d
	./build/playback_2d

playback3d:
	cmake --build build -j8 --target playback_3d
	./build/playback_3d

# Removes every simulator_v1 run: run folders and csv files from before them
clean-v1:
	rm -rf simdata/*_simulator_v1 simdata/simulator_v1

test:
	cmake --build build -j8
	cd build && ctest --output-on-failure
