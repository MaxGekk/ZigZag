all : sample.exe

OBJS=win.o scrlib.o drawcontrol.o sample.o FontBDF6x9.o FontBDF7x13.o charset_cp1251.o carriagescrlib.o eventpresskey.o textscrlib.o eventclick.o
LIBS = -lgdi32 -luser32
CFLAGS = -g -ansi
LFLAGS = -g -Xlinker --subsystem -Xlinker windows 

.c.o :
	gcc $(CFLAGS) -c $<

sample.exe : $(OBJS)
	gcc $(LFLAGS) $(OBJS) $(LIBS) -o sample.exe

clean :
	rm $(OBJS) sample.exe
