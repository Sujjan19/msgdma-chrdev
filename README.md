# MSGDMA Character Device Linux Driver

This repository provides a Linux kernel module that implements a character device interface for Altera/Intel Modular Scatter-Gather DMA (MSGDMA) cores. The driver exposes each DMA instance as a character device so user-space programs can trigger transfers using standard file I/O.

## Features

- Character device interface: `/dev/msgdmaX` per detected DMA instance.
- Supports MM2S (memory->stream) and S2MM (stream->memory) directions (selectable via device tree).
- Multiple independent MSGDMA instances supported (up to the hardware/driver limit).
- Interrupt-driven completion with waitqueue handling.
- Per-instance coherent DMA buffer allocation (default 1 MiB).
- Timeout and error handling with helpful debug dumps.

## File operations

- open: bind a file handle to a DMA instance
- read: for S2MM devices, performs a DMA read into kernel buffer and copies to userspace
- write: for MM2S devices, copies user data into DMA buffer and triggers a DMA write
- release: cleanup per-open state

## Device tree properties

- `dma-direction`: string, either `mm2s` or `s2mm`. This selects the direction the driver will operate in for that node.
- The device node must provide memory regions (`reg`) for the CSR register block and the descriptor slave port, and an `interrupt` entry for the MSGDMA IRQ.

Note: The ordering and cell counts in `reg` must match the SoC's `#address-cells` and `#size-cells`. The driver expects the CSR region first followed by the descriptor-region mapping.

Corrected example (single `reg` entry formed from two ranges; adjust for your `#address-cells`):

```dts
msgdma0: dma@f9000020 {
   compatible = "msgdma_chrdev";
   /* CSR then descriptor slave region: adjust cell counts to your DT */
   reg = <0x0 0xf9000020 0x0 0x00000020  0x0 0xf90000d0 0x0 0x00000010>;
   #dma-cells = <1>;
   interrupts = <0 17 4>;
   interrupt-parent = <&intc>;
   dma-direction = "mm2s"; /* or "s2mm" */
   status = "okay";
};
```

If your DTS uses separate `reg` entries per resource (some toolchains/layouts do), ensure the CSR and descriptor addresses are passed in the order your platform expects and that the driver probe parses them accordingly.

## Build & install

The included `Makefile` supports overriding variables on the command line so you don't have to edit the file. Typical examples:

- Build against an external kernel tree:

```sh
make KBUILD_DIR=/path/to/linux KERNEL_VERSION=<kernel-version> CROSS_COMPILE=aarch64-none-linux-gnu- ARCH=arm64
```

- Install the built module into a target root filesystem (uses `ROOTFS_DIR` and `KERNEL_VERSION`):

```sh
sudo make install ROOTFS_DIR=/path/to/rootfs KERNEL_VERSION=6.6.51-g0447da78ed3c-dirty
```

Notes:

- `KBUILD_DIR` must point to a full kernel source tree configured for your target (same version/config as `KERNEL_VERSION`).
- `CROSS_COMPILE`/`ARCH` should match your target toolchain/architecture.
- You can pass `KERNEL_VERSION=$(uname -r)` to `make install` if you are installing into the currently running kernel's module tree.

Loading the module manually on the target (example):

```sh
sudo insmod /lib/modules/$(uname -r)/extra/msgdma.ko
# or use modprobe after copying the module and running depmod
```

## Quick usage examples

- Verify device nodes exist after loading:

```sh
ls -l /dev/msgdma*
dmesg | tail -n 50
```

- Trigger an MM2S transfer (write):

```sh
dd if=input.bin of=/dev/msgdma0 bs=4096 count=1
```

- Trigger an S2MM transfer (read):

```sh
dd if=/dev/msgdma0 of=out.bin bs=4096 count=1
```

- Inspect the result:

```sh
hexdump -C out.bin | head
dmesg | tail -n 50
```

Adjust `bs`/`count` to match transfer sizes supported by your DMA configuration.

## Troubleshooting & debug tips

- If the device node is missing: check driver probe messages in `dmesg` and confirm the device tree node is present and `status = "okay"`.
- Check IRQs and routing: `dmesg` and `/proc/interrupts` can show whether interrupts are delivered.
- Permission problems: ensure the device node permissions allow your user or use `sudo` for testing.
- Timeouts or stalled transfers: enable more logging in the driver (add `dev_dbg` / `dev_err` prints) and recompile; the driver also emits debug dumps for CSR/descriptor state on errors.
- Common root causes: incorrect `reg` addresses, incorrect `dma-direction`, IRQ not connected, or cache / DMA mapping mismatches on the platform.

## Makefile / build gotchas

- `install` copies the built `.ko` into `ROOTFS_DIR` using the `KERNEL_VERSION` you provided—make sure that kernel version matches the modules layout in the rootfs.
- If you get symbol/version mismatches when insmod, verify the module was built against the same kernel version and config as the target.

## License

This project is licensed under MIT. See source files for detail.

## Author

Sujan SM
