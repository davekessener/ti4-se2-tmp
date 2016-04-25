CC=g++
CPPFLAGS=-Wall -std=c++03 -ggdb -I. $(FLAGS)
LDFLAGS=-lpthread -lrt
SRC=$(shell find -name "*.cc")
OBJ=$(SRC:.cc=.o)
DEP=$(SRC:.cc=.d)
TARGET=exe

.PHONY: all clean tidy

%.d: %.cc
	@set -e; rm -f $@; \
	$(CC) -MM $(CPPFLAGS) $< -o $@; \
	sed -i 's,\($*\)\.o[ :]*,\1.o $@ : ,g' $@;

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

tidy:
	rm -f $(OBJ) $(DEP)

clean: tidy
	rm -f $(TARGET)

-include $(DEP)

