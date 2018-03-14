# Android_O_MTK_GMP109
=============================

Requirements
-----------
- MTK MT6575 
- Version: Android O
- GMP109

I2C Connections
-----------
- Use I2C1
- Slave address 0x76
- SA0---GND

Kernel Build
-----------
- source ./build/envsetup.sh
- lunch full_k57pv1_pre-eng
- make bootimage -j4

File Location
-----------
- put all files into a dirctory named "gmp109"
- put this directory as below
- alps/kernel-4.4/drivers/misc/mediatek/sensors-1.0/barometer/gmp109
