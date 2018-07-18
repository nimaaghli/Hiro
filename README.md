# Hiro
Hiro Firmware for NRF52832

## Requirements:
nRF5 SDK 15.0.0

Softdevices132 6.0.0


## Perpare SDK Files:
Download nRF5 SDK 15.0.0 and unzip it into sdk folder in root.

sdk folder should look like this:

sdk/components

sdk/documentation

sdk/external

sdk/integration

sdk/modules

...

## Prepare ARM GCC Compiler:
You must have GCC compiler installed on your Mac OS or Ubuntu in order to compile the code.
Run this command to check if have arm gcc installed:
```
arm-none-eabi-gcc --version
```

if you have armgcc installed this command will return you the information including the version of the armgcc,
next edit the file in this path and modify the path to your armgcc and its version:
```
sdk/components/toolchain/gcc/Makefile.posix
```

You can find the path to your armgcc by running this command:
```

```
### Example
If your armgcc path is at */usr/local/bin/arm-none-eabi-gcc* and your gccarm version is *7.2.1* then your *Makefile.posix* 
should look like this:
```
GNU_INSTALL_ROOT ?= /usr/local/bin/
GNU_VERSION ?= 7.2.1
GNU_PREFIX ?= arm-none-eabi
```

## Install ArmGCC
### Mac
```
brew install arm-none-eabi
```
### Ubunut Ubuntu ≥ 16.04
```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
```
### Ubuntu < 16.04
```
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install arm-gcc-none-eabi
```

## Compile and flash:
### First flash the softdevice 
```
Make flash_softdevice
```
### Second flash the application 
```
Make flash
```

