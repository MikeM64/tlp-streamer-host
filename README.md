# TLP Streamer - Host Software

This is a collection of utilities to use with the [TLP Streamer Gateware](https://github.com/MikeM64/tlp-streamer-fpga).

# Requirements
- libevent
- [ft60x_driver](https://github.com/lambdaconcept/ft60x_driver) from LambdaConcept

# Build
- `make all` - Build everything
- `make clean` - Delete all built objects

## tlp_bench
Use this to measure round trip latency from the host, to the Screamer and back again.

### Sample Output
```
$ ./tlp_bench -u /dev/ft60x0 
Starting benchmark run with 1000 packets
Writing packets
Received 1000 of 1000 packets
Average latency is 1314771ns

[0 < 1us]       : 0
[1us < 50us]    : 0
[50us < 250us]  : 32
[250us < 1ms]   : 304
[1ms < 5ms]     : 664
[5ms < 25ms]    : 0
[25ms < 50ms]   : 0
[> 50ms]        : 0
```
