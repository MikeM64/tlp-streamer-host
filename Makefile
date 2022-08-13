.PHONY: clean

all: tlp_bench tlp_16550_emu

tlp_16550_emu: tlp_16550_emu.c
	gcc -o tlp_16550_emu tlp_16550_emu.c -Wall -g -levent

tlp_bench: tlp_bench.c
	gcc -o tlp_bench tlp_bench.c -Wall -g -levent

clean:
	rm tlp_bench || 1
	rm tlp_16550_emu || 1
