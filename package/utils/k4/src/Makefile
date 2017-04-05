CC = gcc
websocket_CFLAGS= $(CFLAGS) -std=gnu99 -fstrict-aliasing 
OBJS = k4.o cJSON.o md5Checksum.o
BINARY:=k4
all: $(BINARY)

$(BINARY) : *.c
	$(CC) $(websocket_CFLAGS) -c -o $@ $< $(LDFLAGS)
$(BINARY): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS) 

clean:
	rm -f $(BINARY) *.o
