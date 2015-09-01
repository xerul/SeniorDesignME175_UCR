#
# $Id: Makefile,v 1.11 2001/10/16 21:05:57 jdeneux Exp $
#
# Makefile for Linux input utilities
#
# (c) 1998-2000 Vojtech Pavlik
#
# Sponsored by SuSE
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#
# Should you need to contact me, the author, you can do so either by
# e-mail - mail your message to <vojtech@ucw.cz>, or by paper mail:
# Vojtech Pavlik, Simunkova 1594, Prague 8, 182 00 Czech Republic
#
#
# Edit the options below to suit your needs
#

CPP		= g++
CFLAGS		= -g -O2 -W
CPPFLAGS	= -I.
SOURCES         = evtest.cpp serialib.cpp BT_MP.cpp


evtest: serialib.o BT_MP.o evtest.o
	$(CPP) $(CFLAGS) $^ -o $@ -g -lm

serialib.o: serialib.h serialib.cpp 
	$(CPP) $(CPPFLAGS) $(CFLAGS) -c $^

BT_MP.o:  serialib.h BT_MP.h BT_MP.cpp
	$(CPP) $(CPPFLAGS) $(CFLAGS) -c  $^

evtest.o: evtest.cpp BT_MP.h
	$(CPP) $(CPPFLAGS) $^ -c  


compile: $(PROGRAMS)
	g++ $(CFLAGS) $(CPPFLAGS) -o $@ $^

clean:
	$(RM) *.o *.swp $(PROGRAMS) *.orig *.rej map *~


#ffcfstress: ffcfstress.c
#	$(CC) -O2 -fno-implement-inlines -funsigned-char \
#	 -lm ffcfstress.c -o ffcfstress $(CPPFLAGS)

#ffmvforce.o: ffmvforce.c
#	$(CC) -c $(CFLAGS) $(CPPFLAGS) $^ -o $@ `sdl-config --cflags`
#
#ffmvforce: ffmvforce.o
#	$(CC) $^ -o $@ $(LDFLAGS) -g -lm `sdl-config --libs`
#
#jscal: jscal.o
#	$(CC) $(CFLAGS) $(CPPFLAGS) -lm $^ -o $@
#
#gencodes: gencodes.c scancodes.h
#	$(CC) $(CFLAGS) $(CPPFLAGS) gencodes.c -o gencodes
#
#acceltest: acceltest.c
#	$(CC) $(CFLAGS) $(CPPFLAGS) -lm $^ -o $@
