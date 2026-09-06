
CC = clang
CFLAGS = -std=c11 -I.
DEBUG_CFLAGS = -DDEBUG -g -O0 -fsanitize=address
RELEASE_CFLAGS = -DNDEBUG -O2

CFLAGS += -Wall -Wextra -pedantic -Wconversion
CFLAGS += -Wno-gnu-binary-literal -Wno-c23-extensions

config ?= debug
 
ifeq ($(config), debug)
	CFLAGS += $(DEBUG_CFLAGS)
else
	CFLAGS += $(RELEASE_CFLAGS)
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

clean:
	$(RM_BIN)

.PHONY: clean
