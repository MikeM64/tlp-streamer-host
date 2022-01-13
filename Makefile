.PHONY: clean

all: tlp_bench

tlp_bench: tlp_bench.c
	gcc -o tlp_bench tlp_bench.c -Wall -g -levent

clean:
	rm tlp_bench || 1
