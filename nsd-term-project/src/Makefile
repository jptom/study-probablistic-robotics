CXX := g++

ENV := -O3 -Wall -shared -std=c++11 \
	-fPIC `python3 -m pybind11 --includes` _env.cpp \
	-o _env`python3-config --extension-suffix`
    
KF := -O3 -Wall -shared -std=c++11 \
	-fPIC `python3 -m pybind11 --includes` _kf.cpp \
	-o _kf`python3-config --extension-suffix`

all: target_env target_kf

target_env: 
	${CXX} ${ENV}

target_kf:
	${CXX} ${KF}

clean:
	rm -rf *.so 
