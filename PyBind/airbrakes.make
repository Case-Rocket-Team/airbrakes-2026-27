#C complier
CXX = g++
C = gcc

#C header to add
BASE = -O2
CFLAGS = -std=c11 $(BASE)
CXXFLAGS = -std=c++17 $(BASE)
LFLAGS = -shared -fPIC

#include paths
INCLUDE_PYTHON = -I/usr/include/python3.14/
INCLUDE_LIB= $(INCLUDE_PYTHON) -I.
INCLUDE_EXE = $(INCLUDE_PYTHON) -I..

#files
LIB_EKF_FILES = base/base.c kalman_filter/extended_kalman_filter.c
LIB_AIRBREAKS_CONTROL_FILES =  control/controller/slew_mpc.cpp control/controller/ideal_mpc.cpp control/actuator/actuator_model.cpp control/physics/dynamics.cpp control/aero/brake_aero.cpp control/predictor/apogee_predictor.cpp control/physics/drag.cpp control/physics/atmosphere.cpp

#library paths
LIB_PATHS_EKF = -L. -lEKF -Wl,-rpath,'.'
LIB_PATHS_CONTROL = -L. -lEKF -lairbrakesControl -Wl,-rpath,'.'

libControl: libekf
	cd .. && $(CXX) $(CXXFLAGS) $(LFLAGS) $(INCLUDE_LIB)  $(LIB_PATHS_EKF) $(LIB_AIRBREAKS_CONTROL_FILES) -o libairbrakesControl.so
libekf:
	cd .. && $(C) $(CFLAGS) $(LFLAGS) $(INCLUDE_LIB) $(LIB_AIRBREAKS_FILES) -o libEKF.so

mvekf: libekf
	mv ../libEKF.so libEKF.so
mvcontrol: libControl
	mv ../libairbrakesControl.so libairbrakesControl.so

buildControl: mvcontrol
	 $(CXX) $(CXXFLAGS) $(LFLAGS) $(INCLUDE_EXE) $(LIB_PATHS_CONTROL) ControlBind.cpp -o ControlBind$(shell python3-config --extension-suffix)
buildair: mvekf
	 $(CXX) $(CXXFLAGS) $(LFLAGS) $(INCLUDE_EXE) $(LIB_PATHS_EKF) PyBind.cpp -o PyBind$(shell python3-config --extension-suffix)

clean:
	rm -f libEKF.so \
    rm -f libairbrakesControl.so \
    rm -f ControlBind$(shell python3-config --extension-suffix) \
    rm -f PyBind$(shell python3-config --extension-suffix)



.PHONY : clean libair libair mvcontrol mvair buildControl buildair
