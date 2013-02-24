#!/bin/sh

set -e

module="a2gx"
device="a2gx"
mode="664"
group="wheel"
major="198"

/sbin/insmod ./${module}.ko a2gx_cdev_major=${major}

rm -f /dev/${device}_io
mknod /dev/${device}_io c ${major} 0

chmod ${mode} /dev/${device}_io
chgrp ${group} /dev/${device}_io
