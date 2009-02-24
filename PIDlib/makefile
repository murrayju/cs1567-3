PIDlib: PIDlib.c PIDlib.h FIRlib.o
	gcc `pkg-config --cflags playerc` -c `pkg-config --libs playerc` PIDlib.c PIDlib.h FIRlib.o
	
FIRlib.o: ../FIRlib/FIRlib.c ../FIRlib/FIRlib.h
	gcc `pkg-config --cflags playerc` -c `pkg-config --libs playerc` ../FIRlib/FIRlib.h ../FIRlib/PIDlib.c
	