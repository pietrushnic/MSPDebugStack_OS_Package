CXX:= g++

DEFINES := -DUNIX

CXXFLAGS := -fPIC

ifdef BIT32
CXXFLAGS += -m32
endif

ifdef DEBUG
CXXFLAGS += -g -O0
else
CXXFLAGS += -Os
DEFINES += -DNDEBUG
endif

INCLUDES := \
	-I../include \
	-I./BSL430_DLL \
	-I./BSL430_DLL/Utility_Classes \
	-I./BSL430_DLL/Physical_Interfaces \
	-I./BSL430_DLL/Packet_Handlers \
	-I./BSL430_DLL/Connections

LIBDIRS :=

ifdef BOOST_DIR
INCLUDES += -I$(BOOST_DIR)
LIBDIRS += -L$(BOOST_DIR)/stage/lib
endif

LIBS :=

SRC := \
	./BSL430_DLL/MSPBSL_Factory.cpp \
	$(wildcard ./BSL430_DLL/Connections/*.cpp) \
	$(wildcard ./BSL430_DLL/Physical_Interfaces/*.cpp) \
	$(wildcard ./BSL430_DLL/Packet_Handlers/*.cpp) \
	$(wildcard ./BSL430_DLL/Utility_Classes/*.cpp)

OBJS := $(patsubst %.cpp, %.o, $(SRC))

OUTPUT := ../lib/libbsl430.a

all: $(OBJS)
	ar -cq $(OUTPUT) $(OBJS)

%.o: %.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(INCLUDES) $(DEFINES)

install:
	cp $(OUTPUT) /usr/lib/

clean:
	@for i in $(OBJS); do rm -f $$i; done
	@rm -f $(OUTPUT)
	@rm -f build.log
