hallnavigator: hallnavigator.c PIDlib.o FIRlib.o
	gcc `pkg-config --cflags playerc` -o hallnavigator `pkg-config --libs playerc` hallnavigator.c PIDlib.o FIRlib.o
	
PIDlib.o: ../PIDlib/PIDlib.c ../PIDlib/PIDlib.h
	gcc `pkg-config --cflags playerc` -c `pkg-config --libs playerc` ../PIDlib/PIDlib.h ../PIDlib/PIDlib.c

FIRlib.o: ../FIRlib/FIRlib.c ../FIRlib/FIRlib.h
	gcc `pkg-config --cflags playerc` -c `pkg-config --libs playerc` ../FIRlib/FIRlib.h ../FIRlib/FIRlib.c

