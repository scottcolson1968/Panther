# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the IIO Dummy Driver
#

obj-$(CONFIG_IIO_SIMPLE_DUMMY) += iio_dummy.o
iio_dummy-y := iio_simple_dummy.o
#iio_dummy-y += xtacho.o
#iio_dummy-y += xtacho_linux.o
#iio_dummy-y += xdrivescale.o
#iio_dummy-y += xdrivescale_linux.o
#//iio_dummy-$(CONFIG_IIO_SIMPLE_DUMMY_EVENTS) += iio_simple_dummy_events.o
#iio_dummy-$(CONFIG_IIO_SIMPLE_DUMMY_BUFFER) += iio_simple_dummy_buffer.o

#obj-$(CONFIG_IIO_DUMMY_EVGEN) += iio_dummy_evgen.o

obj-y += iio_adccountstofloat.o
iio_adccountstofloat-y := adccountstofloat.o
iio_adccountstofloat-y += xadccountstofloat.o

obj-y += iio_drivescale.o
iio_drivescale-y := drivescale.o
iio_drivescale-y += xdrivescale.o
# xdrivescale_linux.o removed: UIO scanning replaced by platform_driver

obj-y += iio_synch.o
iio_synch-y := synch.o
iio_synch-y += xsynch.o

obj-y += iio_tacho_control.o
iio_tacho_control-y := tacho_control.o
iio_tacho_control-y += xtacho_control.o

obj-y += iio_tacho.o
iio_tacho-y := tacho.o
iio_tacho-y += xtacho.o

# Panther FMC FRU (IPMI/VITA-57) IIO attributes over at24 nvmem
obj-y += panther_fru.o
