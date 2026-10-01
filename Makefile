.PHONY: 2d v1 playback clean-v1 test

v1:
	cmake --build build -j8
	chmod +x build/simulator_v1
	./build/simulator_v1

2d:
	chmod +x build/sim2d
	./build/sim2d

playback:
	cmake --build build -j8 --target playback-2d
	./build/playback-2d

clean-v1:
	rm -f simdata/simulator_v1/*.csv

test:
	./build/test_common