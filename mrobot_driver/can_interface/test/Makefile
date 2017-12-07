SRC_DIR := ../src
INC_DIR := ../include
#INC_DIR += /usr/include
DLIB := ../include/can_interface

LINK_TARGET = test

INCLUDE_DIRS = -I$(INC_DIR)
SOURCES = $(SRC_DIR)/CanVCI.cpp \
          ./test.cpp

OBJS = $(SOURCES:%.cpp=%.o)
CC = g++
CFLAGS = -Wall -std=c++11 $(INCLUDE_DIRS)
LDFLAGS = -L$(DLIB) -lcontrolcan -lboost_system -lpthread

.PHONY:all clean

all: clean $(LINK_TARGET)

clean:
	rm -f $(OBJS) $(LINK_TARGET)

%.o: %.cpp
	$(CC) $(CFLAGS) -MP -MMD -c $< -o $@

$(LINK_TARGET): $(OBJS)
	$(CC) $^ -o $@ $(LDFLAGS) 
	 
