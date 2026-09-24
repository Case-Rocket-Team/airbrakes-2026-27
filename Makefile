
CC = clang
CXX = clang++
CFLAGS = -std=c11 -I.
CXXFLAGS = -std=c++17 -I.
DEBUG_CFLAGS = -DDEBUG -g -O0 -fsanitize=address
RELEASE_CFLAGS = -DNDEBUG -O2

CFLAGS += -Wall -Wextra -pedantic -Wconversion
CFLAGS += -Wno-gnu-binary-literal -Wno-c23-extensions

CXXFLAGS += -Wall -Wextra -pedantic -Wconversion

config ?= debug
 
ifeq ($(config), debug)
	CFLAGS += $(DEBUG_CFLAGS)
	CXXFLAGS += $(DEBUG_CFLAGS)
else
	CFLAGS += $(RELEASE_CFLAGS)
	CXXFLAGS += $(RELEASE_CFLAGS)
endif

# OS-Specific Stuff
LFLAGS =
MKDIR_BIN = 
RM_BIN = 
BIN_EXT = 

ifeq ($(OS), Windows_NT)
	LFLAGS += 
	MKDIR_BIN = if not exist bin\$(config) mkdir bin\$(config)
	RM_BIN = rd /s /q bin
	BIN_EXT = .exe
else
	LFLAGS += -lm 
	MKDIR_BIN = mkdir -p bin/$(config)
	RM_BIN = rm -r bin
endif

BIN_DIR = bin/$(config)/

test_kf:
	@$(MKDIR_BIN)
	$(CC) kalman_filter/test_kf.c $(CFLAGS) $(LFLAGS) -o $(BIN_DIR)test_kf$(BIN_EXT)

test_ekf:
	@$(MKDIR_BIN)
	$(CC) kalman_filter/test_ekf.c $(CFLAGS) $(LFLAGS) -o $(BIN_DIR)test_ekf$(BIN_EXT)

test_control:
	@$(MKDIR_BIN)
	$(CXX) control/tests/test_control_interface.cpp $(CXXFLAGS) $(LFLAGS) -o $(BIN_DIR)test_control$(BIN_EXT)
	$(BIN_DIR)test_control$(BIN_EXT)

test_atmosphere:
	@$(MKDIR_BIN)
	$(CXX) control/tests/test_atmosphere.cpp control/physics/atmosphere.cpp $(CXXFLAGS) $(LFLAGS) -o $(BIN_DIR)test_atmosphere$(BIN_EXT)
	$(BIN_DIR)test_atmosphere$(BIN_EXT)

test_drag:
	@$(MKDIR_BIN)
	$(CXX) control/tests/test_drag.cpp control/physics/drag.cpp $(CXXFLAGS) $(LFLAGS) -o $(BIN_DIR)test_drag$(BIN_EXT)
	$(BIN_DIR)test_drag$(BIN_EXT)

test_dynamics:
	@$(MKDIR_BIN)
	$(CXX) control/tests/test_dynamics.cpp control/physics/dynamics.cpp control/physics/atmosphere.cpp control/physics/drag.cpp $(CXXFLAGS) $(LFLAGS) -o $(BIN_DIR)test_dynamics$(BIN_EXT)
	$(BIN_DIR)test_dynamics$(BIN_EXT)

test_apogee_predictor:
	@$(MKDIR_BIN)
	$(CXX) control/tests/test_apogee_predictor.cpp control/predictor/apogee_predictor.cpp control/physics/dynamics.cpp control/physics/atmosphere.cpp control/physics/drag.cpp $(CXXFLAGS) $(LFLAGS) -o $(BIN_DIR)test_apogee_predictor$(BIN_EXT)
	$(BIN_DIR)test_apogee_predictor$(BIN_EXT)

test: test_kf test_control test_atmosphere test_drag test_dynamics test_apogee_predictor

clean:
	$(RM_BIN)
.PHONY: clean test_kf test_ekf test_control test_atmosphere test_drag test_dynamics test_apogee_predictor test
