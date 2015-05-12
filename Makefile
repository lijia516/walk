#
#  Makefile for fltk applications
#

LOCAL = /usr/local

FLTK = /opt/local

CC = c++

INCLUDE = -I$(LOCAL)/include -I$(FLTK)/include
LIBDIR = -L$(FLTK)/lib -L$(LOCAL)/lib -L$(FLTK)/X11/lib

LDLIBS = -L/usr/local/lib/ -framework Carbon -framework ApplicationServices -framework Cocoa
GLDLIBS = -framework AGL -framework OpenGL
LIBS  = $(LDLIBS) $(GLDLIBS) -lfltk_gl -lfltk -lfltk_images -lfltk_forms -lfltk_jpeg -lpng -lm

CFLAGS = -g

.SUFFIXES: .o .cpp .cxx

.cpp.o: 
	$(CC) $(CFLAGS) $(INCLUDE) -c -o $*.o $<

ALL.O = bitmap.o camera.o color.o curve.o curveevaluator.o \
        graphwidget.o indicatorwindow.o linearcurveevaluator.o \
        modelerapp.o modelerdraw.o modelerui.o animatoruiwindows.o \
        modelerview.o particleSystem.o point.o \
        rect.o human.o rulerwindow.o beziercurveevaluator.o \
	bsplinescurveevaluator.o catmullromcurveevaluator.o \
	c2interpolatingcurveevaluator.o ik.o

animator: $(ALL.O)
	$(CC) $(CFLAGS) -o $@ $(ALL.O) $(INCLUDE) $(LIBDIR) $(LIBS)

clean:  
	rm -f $(ALL.O)

clean_all:
	rm -f $(ALL.O) animator

