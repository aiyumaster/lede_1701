CC = gcc
websocket_CFLAGS= $(CFLAGS) -std=gnu99 -fstrict-aliasing 
OBJS = main.o Com.o
BINARY:=test
all: $(BINARY)

$(BINARY) : *.c
	$(CC) $(websocket_CFLAGS) -c -o $@ $< $(LDFLAGS)
$(BINARY): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS) 

clean:
	rm -f $(BINARY) *.o
