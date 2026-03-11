# aurora-rs

My second attempt at writting custom firmware for my aurora split keyboard https://github.com/pierrechevalier83/ferris. Using nrf52840 microcontroller.

Inpiration for the code
- https://github.com/KOBA789/rusty-keys
- https://github.com/bschwind/key-ripper


## V 0.1 Goals
- Works on aurora split keyboard
- Basic keymap support
- Layer support
- Per Key RGB support


## Testing code

### No Debug Probe
For some reason it's very difficult to flash the nrf52840 without a debug probe. I have tried using the built in usb bootloader but it doesn't work for some reason. I have also tried using the jlink commander but it also doesn't work. If you have any tips on how to flash the nrf52840 without a debug probe. Currently what works is building with `cargo build --release` and then using [elf2flash](https://github.com/BjornTheProgrammer/elf2flash) to convert the elf executable to uf2. The just dragging that into the device, I am using nicenanos. 
