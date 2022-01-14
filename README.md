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

## tlp_16550_emu
This is a device emulator for an (Exar XR17V352)[https://assets.maxlinear.com/web/documents/xr17v352.pdf].

### Setup
Set the VID:PID of the TLP Streamer PCIe core to 13a8:0352 and flash to the Screamer. This will be detected in Linux as a 2-port 16550 Serial Interface and should show up in lspci with similar output:

```
$ sudo lspci -nn -d 13a8:0352 -v
0b:00.0 Serial controller [0700]: Exar Corp. XR17V3521 Dual PCIe UART [13a8:0352] (prog-if 02 [16550])
        Flags: bus master, fast devsel, latency 0, IOMMU group 27
        Memory at fcc00000 (32-bit, non-prefetchable) [size=4K]
        Capabilities: [40] Power Management version 3
        Capabilities: [60] Express Endpoint, MSI 00
        Capabilities: [9c] MSI-X: Enable+ Count=1 Masked-
        Capabilities: [100] Device Serial Number 00-00-00-00-00-00-00-00
        Kernel driver in use: exar_serial
        Kernel modules: 8250_exar
```

