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
iio_adccountstofloat-y += xadccountstofloat_linux.o

obj-y += iio_drivescale.o
iio_drivescale-y := drivescale.o
iio_drivescale-y += xdrivescale.o
iio_drivescale-y += xdrivescale_linux.o

obj-y += iio_synch.o
iio_synch-y := synch.o
iio_synch-y += xsynch.o
iio_synch-y += xsynch_linux.o
