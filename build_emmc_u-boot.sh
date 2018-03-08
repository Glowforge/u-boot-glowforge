#!/bin/bash
source /opt/poky/1.8.1/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi
make distclean
make glowforge_defconfig
make
