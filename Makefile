CC=gcc
CFLAGS=-Wall -Wpedantic -Werror -ggdb
LDFLAGS=-lm -lraylib

main: main.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm main
