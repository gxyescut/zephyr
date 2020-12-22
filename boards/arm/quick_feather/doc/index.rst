.. _quickfeather:

QuickFeather
############

Overview
********

The QuickFeather development board is a platform with an on-board QuickLogic
EOS S3 Sensor Processing Platform.


.. figure:: img/feather-board.png
   :width: 500px
   :align: center
   :alt: QuickFeather

   QuickFeather (Credit: QuickLogic)

Hardware
********

- QuickLogic EOS S3 MCU Platform
- mCube MC3635 accelerometer
- Infineon DPS310 pressure sensor
- Infineon IM69D130 MEMS microphone
- 16 Mbit of on-board flash memory
- User button
- RGB LED
- Powered from USB or a single Li-Po battery
- Integrated battery charger
- USB data signals tied to programmable logic

Supported Features
==================

The QuickFeather configuration supports the following hardware
features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| UART      | on-chip    | serial port                         |
+-----------+------------+-------------------------------------+

Other hardware features are not currently supported by Zephyr.

The default configuration can be found in the Kconfig
:zephyr_file:`boards/arm/quick_feather/quick_feather_defconfig`.

Connections and IOs
===================

Detailed information about pinouts is available on the
`QuickLogic github repository`_ in the `schematics PDF document`_.

Programming and Debugging
*************************

Flashing
========

EOS S3 platform contains preconfigured DMA which tries to load a program
from SPI flash to SRAM. Currently the QuickFeather Zephyr port only enables
loading program directly to SRAM using either OpenOCD and an SWD programmer
or SEGGER JLink.

- `OpenOCD QuickFeather definition`_
- JLink flow described in `QuickFeather User Guide`_

Debugging
=========

To debug the QuickFeather board please connect to the target with either
OpenOCD or JLink and use GDB provided in Zephyr toolchain in *arm-zephyr-eabi*
directory. Sample can be built in the usual way:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: quickfeather
   :goals: build


References
**********

.. target-notes::

.. _QuickLogic github repository:
    https://github.com/QuickLogic-Corp/quick-feather-dev-board

.. _schematics PDF document:
    https://github.com/QuickLogic-Corp/quick-feather-dev-board/blob/master/doc/quickfeather-board.pdf

.. _OpenOCD QuickFeather definition:
    https://sourceforge.net/p/openocd/code/ci/master/tree/tcl/board/quicklogic_quickfeather.cfg

.. _QuickFeather User Guide:
    https://github.com/QuickLogic-Corp/quick-feather-dev-board/blob/master/doc/QuickFeather_UserGuide.pdf
