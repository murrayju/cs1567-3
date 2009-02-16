hallnavigator: hallnavigator.c ../PIDlib/PIDlib.h PIDlib.o
	gcc `pkg-config --cflags playerc` -o hallnavigator `pkg-config --libs playerc` hallnavigator.c ../PIDlib/PIDlib.h PIDlib.o
	
PIDlib.o: ../PIDlib/PIDlib.c ../PIDlib/PIDlib.h
	gcc `pkg-config --cflags playerc` -c `pkg-config --libs playerc` ../PIDlib/PIDlib.h ../PIDlib/PIDlib.c

