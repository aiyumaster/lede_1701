CC = gcc
websocket_CFLAGS= $(CFLAGS) -std=gnu99 -fstrict-aliasing 
OBJS = speexsendbrxy.o wav_io.o wave_out.o
BINARY:=speexsendbrxy
all: $(BINARY)

$(BINARY) : *.c
	$(CC) $(websocket_CFLAGS) -c -o $@ $< $(LDFLAGS)
$(BINARY): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS) 

clean:
	rm -f $(BINARY) *.o
