

BASE_DIR =../

CC=g++

CFLAGS= -O3 -Wall

IFLAGS= -I./
 

LIB=./liblinalg.a

OBJS= \
Matrix.o \
Vector.o \
Exception.o \
Arguments.o


all: $(LIB)

$(LIB): $(OBJS)
	rm -f $(LIB)
	ar -r $(LIB) $(OBJS)
	ranlib $(LIB)

Matrix.o: Matrix.cc Matrix.hh
	$(CC) $(CFLAGS) $(IFLAGS) -c -o Matrix.o Matrix.cc	 

Vector.o: Vector.cc Vector.hh
	$(CC) $(CFLAGS) $(IFLAGS) -c -o Vector.o Vector.cc	 
	
Exception.o: Exception.cc Exception.hh
	$(CC) $(CFLAGS) $(IFLAGS) -c -o Exception.o Exception.cc
	
Arguments.o: Arguments.cc Arguments.hh
	$(CC) $(CFLAGS) $(IFLAGS) -c -o Arguments.o Arguments.cc	 
	
		
clean:
	rm -f *.o  $(LIB) core *.*~
	