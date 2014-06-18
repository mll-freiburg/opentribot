PREFIX = /usr/local
BINDIR = $(PREFIX)/bin
HDRDIR = /usr/include/linux

X86  := $(shell if echo - | $(CC) -E -dM - | egrep -e '__i386__|__x86_64__' >/dev/null; \
	then echo y; else echo n; fi)

ifeq ($(X86),y)
X86_EXES := ivtvplay ivtv-mpegindex #ivtv-encoder
endif

EXES := Tribots_v4l v4l2-ctl-simple v4l2-ctl ivtv-ctl ivtv-radio $(X86_EXES)

BIN := $(EXES) ivtv-tune/ivtv-tune cx25840ctl/cx25840ctl


HEADERS := linux/ivtv.h linux/ivtvfb.h

CFLAGS = -D_GNU_SOURCE -O2 -Wall -g -I.
CXXFLAGS = $(CFLAGS)

all:: $(EXES)
	$(MAKE) CFLAGS="$(CFLAGS)" -C ivtv-tune
	$(MAKE) CFLAGS="$(CFLAGS)" -C cx25840ctl

ivtv-ctl: ivtv-ctl.o
	$(CC) -lm -o $@ $^

v4l2-ctl: v4l2-ctl.o
	$(CXX) -lm -o $@ $^
Tribots_v4l: Tribots_v4l.o
	$(CXX) -lm -o $@ $^

v4l2-ctl-simple: v4l2-ctl-simple.o
	$(CXX) -lm -o $@ $^

v4l2-dbg: v4l2-dbg.o v4l2-driverids.o v4l2-chipids.o
	$(CXX) -lm -o $@ $^

ivtvplay: ivtvplay.cc
	$(CXX) $(CXXFLAGS) -lm -lpthread -o $@ $^

encoder.o: encoder.c
	$(CC) $(CFLAGS) -DVIDEO_PORT=0 -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -c $^

ivtv-encoder: enc_mindex.o enc_chann.o encoder.o
	$(CC) -lpthread -o $@ $^

install: all
	install -d $(DESTDIR)/$(HDRDIR)
	install -m 0644 $(HEADERS) $(DESTDIR)/$(HDRDIR)
	install -d $(DESTDIR)/$(BINDIR)
	install -m 0755 $(BIN) $(DESTDIR)/$(BINDIR)

clean: 
	rm -f *.o $(EXES)
	$(MAKE) -C ivtv-tune clean
	$(MAKE) -C cx25840ctl clean
