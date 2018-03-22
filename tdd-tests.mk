#---------
#
# CppUTest Examples Makefile
#
#----------

#Set this to @ to keep the makefile quiet
ifndef SILENCE
	SILENCE = @
endif

CPPUTEST_CXXFLAGS += -Wno-old-style-cast

#--- Inputs ----#
COMPONENT_NAME = PizzaDuinoTests
CPPUTEST_HOME = ~/source/cpputest

CPPUTEST_USE_GCOV = Y

CPPUTEST_USE_EXTENSIONS = Y
CPP_PLATFORM = Gcc

# This line is overriding the default new macros.  This is helpful
# when using std library includes like <list> and other containers
# so that memory leak detection does not conflict with stl.
#CPPUTEST_MEMLEAK_DETECTOR_NEW_MACRO_FILE = -include ApplicationLib/ExamplesNewOverrides.h
#SRC_DIRS = \

SRC_FILES = PizzaOvenControllerSketch/crc.cpp \
	    PizzaOvenControllerSketch/serialCommWrapper.cpp \
	    PizzaOvenControllerSketch/ringbuf.cpp \
	    PizzaOvenControllerSketch/ftoa.cpp \
	    PizzaOvenControllerSketch/tcLimitCheck.cpp \
	    PizzaOvenControllerSketch/utility.cpp \
	    
TEST_SRC_DIRS = \
	Tests \
	Tests/AllTests \
	Tests/crc \
	Tests/serialCommWrapper \
	Tests/ringBuffer \
	Tests/ftoa \
	Tests/tcLimits \

MOCKS_SRC_DIRS = Tests/Mocks \

INCLUDE_DIRS =\
	Tests/Mocks \
	PizzaOvenControllerSketch \
	$(CPPUTEST_HOME)/include \

include $(CPPUTEST_HOME)/build/MakefileWorker.mk


