# makefile outline provided by Roger Ferrer Ibáñez, August 16, 2015, as found on
# https://thinkingeek.com/2015/08/16/a-simple-plugin-for-gcc-part-1/ (accessed 2020-01-01)

LIBNAME = sans_t_scoped.so

SDIR = src
IDIR = include
ODIR = obj
PDIR = plugin

DYNLIB = $(PDIR)/$(LIBNAME)

_DEPS = sans_t_scoped.h blockinterpreter.h blockstorage.h genericparser.h common.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = sans_t_scoped.o blockinterpreter.o blockstorage.o genericparser.o common.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

CXX = g++
# Flags for the C++ compiler: enable C++11 and all the warnings, -fno-rtti is required for GCC plugins
CXXFLAGS = -std=c++11 -Wall -fno-rtti 
# Workaround for an issue of -std=c++11 and the current GCC headers
CXXFLAGS += -Wno-literal-suffix
 
# Determine the plugin-dir and add it to the flags
PLUGINDIR=$(shell $(CXX) -print-file-name=plugin)
CXXFLAGS += -I$(PLUGINDIR)/include
CXXFLAGS += -I$(IDIR)

 
# top level goal: build our plugin as a shared library
all: $(DYNLIB)
 
$(DYNLIB): $(OBJ)
	$(CXX) -shared -o $@ $^

$(ODIR)/%.o: $(SDIR)/%.cc $(DEPS)
	$(CXX) $(CXXFLAGS) -fPIC -c -o $@ $<
 
clean:
	rm -f $(OBJ) $(DYNLIB)
 
check: $(DYNLIB)
	$(CXX) -fplugin=./$(DYNLIB) -c -x c++ /dev/null -o /dev/null
 
.PHONY: all clean check
