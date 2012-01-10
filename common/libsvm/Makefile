CXX ?= g++
CFLAGS = -Wall -Wconversion -O3 -fPIC -I./include
SHVER = 2

all: svm-train svm-predict svm-scale

lib: svm.o
	$(CXX) -shared -dynamiclib svm.o -o libsvm.so.$(SHVER)

svm-predict: src/svm-predict.c svm.o
	$(CXX) $(CFLAGS) src/svm-predict.c svm.o -o svm-predict -lm
svm-train: src/svm-train.c svm.o
	$(CXX) $(CFLAGS) src/svm-train.c svm.o -o svm-train -lm
svm-scale: src/svm-scale.c
	$(CXX) $(CFLAGS) src/svm-scale.c -o svm-scale
svm.o: src/svm.cpp include/libsvm/svm.h
	$(CXX) $(CFLAGS) -c src/svm.cpp
clean:
	rm -f *~ svm.o svm-train svm-predict svm-scale libsvm.so.$(SHVER)
