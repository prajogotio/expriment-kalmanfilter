GTEST_DIR=~/third_party/gunit/googletest

all: driver kalman_test matrix_test

driver: kalman.o matrix.o
	g++ -std=c++0x driver.cc out/kalman.o out/matrix.o -o out/driver

matrix_test: matrix.o matrix_test.o
	g++ -std=c++0x out/matrix_test.o out/matrix.o $(GTEST_DIR)/libgtest.a \
	    -o out/matrix_test

matrix_test.o:
	g++ -std=c++0x -c matrix_test.cc -o out/matrix_test.o \
	    -isystem $(GTEST_DIR)/include

matrix.o:
	g++ -std=c++0x -c matrix.cc -o out/matrix.o

kalman_test: kalman.o kalman_test.o matrix.o
	g++ -std=c++0x out/kalman.o out/kalman_test.o out/matrix.o \
	               $(GTEST_DIR)/libgtest.a \
	    -o out/kalman_test

kalman_test.o:
	g++ -std=c++0x -c kalman_test.cc -o out/kalman_test.o \
	    -isystem $(GTEST_DIR)/include

kalman.o:
	g++ -std=c++0x -c kalman.cc -o out/kalman.o

clean:
	rm out/*.o; rm out/driver; rm out/matrix_test; rm out/kalman_test