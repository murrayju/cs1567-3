#*******************************************************************
#
# Makefile template  for gumstix/ARM cross compiler flow
#  
# STEP 1: replace the list below with the names of your source file(s) 
SRCS=mazesolver.c FIRlib.c PIDlib.c Mazelib.c
#
# STEP 2: enter the name of your target executable below
TARGET=mazesolver
# STEP 3: (optional .. if you enter the name of the target robot below
# than typing "make install" will copy the target executable there
ROBOT=johnny5
#
#*******************************************************************
# changing stuff below this line is at your own risk..
# send comments to don@cs.pitt.edu
#
CDIR=/opt/gumstix-buildroot/build_arm_nofpu/staging_dir/arm-linux-uclibcgnueabi/bin
CFLAGS=-Os -march=iwmmxt -mtune=iwmmxt -mcpu=iwmmxt -I include -I /usr/share/pittCreate-API/include
LIBS=-lm /usr/share/pittCreate-API/lib/libpittCreate.a -lpthread
CPP=${CDIR}/g++
CC=${CDIR}/gcc 

${TARGET}: ${SRCS}
	${CC} ${CFLAGS} -o ${TARGET} ${SRCS} ${LIBS}

install: ${TARGET}
	scp ${TARGET} csbot@${ROBOT}:

