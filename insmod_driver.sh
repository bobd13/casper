#!/bin/bash

sudo insmod ~/c/linux_drivers/quad_enc/quad_enc.ko
sudo chown pi.pi /dev/quad_enc*
