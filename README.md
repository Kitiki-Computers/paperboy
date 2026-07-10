# Paperboy

A Game Boy emulator for the M5Stack PaperS3 (ESP32-S3).

## Build

The dependency lock was generated with ESP-IDF 5.4.3. Install that release (or
use Espressif's VS Code extension), then open an ESP-IDF-enabled terminal. In
the project directory run:

```sh
idf.py set-target esp32s3
idf.py build
```

`set-target` only needs to be run for a new build directory. A successful build
creates `build/paperboy.bin` plus the bootloader and partition-table images.
The checked-in `sdkconfig.defaults` selects the PaperS3's 16 MB flash, octal
PSRAM, 240 MHz CPU, performance optimization, and FAT long-filename support.

## Flash a PaperS3 over USB

You do not need M5Burner for a custom development build. Connect the PaperS3
with a USB **data** cable, then put it in download mode:

1. Turn the device off if necessary.
2. Long-press the power button.
3. Release it when the rear status LED flashes red.

Find the serial port after entering download mode:

```sh
# macOS
ls /dev/cu.*

# Linux
ls /dev/ttyACM* /dev/ttyUSB*
```

Then build and flash the complete image set:

```sh
idf.py -p /dev/cu.usbmodemXXXX flash
```

Replace the example port with the one on your machine. On Windows it will look
like `COM3`. You can append `monitor` to watch logs after flashing:

```sh
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

Exit the monitor with `Ctrl-]`. If automatic reset fails after flashing, press
the power/reset button once. If connection fails, re-enter download mode and
try a lower baud rate:

```sh
idf.py -p /dev/cu.usbmodemXXXX -b 115200 flash
```

Do not flash only `build/paperboy.bin` at address zero: an ESP-IDF application
also needs its bootloader and partition table at their generated offsets.
`idf.py flash` reads those offsets from the build and handles all images safely.

## M5Burner

M5Burner has a **USER CUSTOM → Publish** facility for uploading and sharing a
firmware package. It is useful for distributing a stable release, but direct
`idf.py flash` is faster and less error-prone while developing because it uses
the exact bootloader, partition table, application, and offsets from this build.

## ROMs and saves

Place `.gb` or `.gbc` ROM files on the SD card. Paperboy scans the card and its
first-level subdirectories. Save RAM uses a `.sav` file beside the ROM; emulator
save states use `.state`.
