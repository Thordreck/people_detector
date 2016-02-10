



LFLAGS += 

IFLAGS += -I./ -I../common -I../linalg

CFLAGS += -O3 -Wall

CXXFLAGS += $(CFLAGS)

LIB += libfeatures.a 

OBJS += Feature.o
OBJS += Feature_1.o
OBJS += Feature_2.o
OBJS += Feature_3.o
OBJS += Feature_6.o
OBJS += Feature_7.o
OBJS += Feature_8.o
OBJS += Feature_9.o
OBJS += Feature_10.o
OBJS += Feature_11.o
OBJS += Feature_12.o
OBJS += Feature_13.o


#DEFFLAGS += -DDEBUG

$(LIB): $(OBJS)
	rm -f $(LIB)
	ar -r $(LIB) $(OBJS)
	ranlib $(LIB)


all: $(LIB) 

Feature.o: Feature.h Feature.cpp
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature.o Feature.cpp 

Feature_1.o: Feature_1.h Feature_1.cpp
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_1.o Feature_1.cpp 

Feature_2.o: Feature_2.h Feature_2.cpp
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_2.o Feature_2.cpp 

Feature_3.o: Feature_3.h Feature_3.cpp
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_3.o Feature_3.cpp 

Feature_6.o: Feature_6.h Feature_6.cpp
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_6.o Feature_6.cpp 

Feature_7.o: Feature_7.h Feature_7.cpp
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_7.o Feature_7.cpp 

Feature_8.o: Feature_8.h Feature_8.cpp
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_8.o Feature_8.cpp 

Feature_9.o: Feature_9.h Feature_9.cpp Feature_8.h Feature_8.cpp 
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_9.o Feature_9.cpp 

Feature_10.o: Feature_10.h Feature_10.cpp  
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_10.o Feature_10.cpp 

Feature_11.o: Feature_11.h Feature_11.cpp  
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_11.o Feature_11.cpp 

Feature_12.o: Feature_12.h Feature_12.cpp  
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_12.o Feature_12.cpp 

Feature_13.o: Feature_13.h Feature_13.cpp  
	g++ $(CXXFLAGS) $(IFLAGS) $(DEFFLAGS) -c -o Feature_13.o Feature_13.cpp 

clean:
	rm -f $(LIB)  *.o *.*~








