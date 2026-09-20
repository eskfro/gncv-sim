.PHONY: 2 v1

v1:
	cmake --build build -j8
	chmod +x build/simulator_v1
	./build/simulator_v1

2d:
	chmod +x build/sim2d
	./build/sim2d

clean-v1:
	rm -f simdata/simulator_v1/*.csv

test:
	./build/test_common