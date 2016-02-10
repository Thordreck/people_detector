



LFLAGS += 

IFLAGS += -I./ 

CFLAGS += -O0 -Wall
CXXFLAGS += $(CFLAGS)

LIB = libcommon.a

OBJS += Segment.o
OBJS += MultiScan.o

all: $(LIB) 

$(LIB): $(OBJS)
	rm -f $(LIB)
	ar -r $(LIB) $(OBJS)
	ranlib $(LIB)

Segment.o: Segment.h Segment.cpp
	g++ $(CXXFLAGS) $(IFLAGS) -c -o Segment.o Segment.cpp 

MultiScan.o: MultiScan.h MultiScan.cpp
	g++ $(CXXFLAGS) $(IFLAGS) -c -o MultiScan.o MultiScan.cpp 


clean:
	rm -f $(LIB)  *.o *.*~ 








