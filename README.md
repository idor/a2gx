Arria II GX (EP2AGX260FF35I3)
===

The PCIe device shows up in "lspci -v" like this:

09:00.0 Non-VGA unclassified device: Altera Corporation Device 1986 (rev 01) (prog-if 02)
        Subsystem: Altera Corporation Device 1413
        Flags: bus master, fast devsel, latency 0, IRQ 10
        Memory at d0000000 (64-bit, prefetchable) [size=128M]
        Memory at d8000000 (32-bit, non-prefetchable) [size=128M]
        Capabilities: [50] MSI: Enable- Count=1/1 Maskable- 64bit+
        Capabilities: [78] Power Management version 3
        Capabilities: [80] Express Endpoint, MSI 00
        Capabilities: [100] Virtual Channel

After (re-)programming, ask Linux to re-scan the PCIe bus instead of rebooting:

      echo 1 > /sys/bus/pci/rescan

If you get lucky, everything will be alright. But chances are that
Linux won't be able to allocate I/O memory for a new device and the
reboot will be required.

