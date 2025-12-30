XScopes-Firmware
================

Source code for the XScopes:

    - Xprotolab
    - Xminilab
    - Xprotolab Portable
    - Xminilab Portable

The project is developed using Atmel Studio 6.2, available from atmel.com.
Newer compilers generate less compact code and the firmware can't fit in the program memory.

The XScopes are a combination of three electronic instruments: a mixed signal oscilloscope, an arbitrary waveform generator and a protocol sniffer. The XScopes can also be used as development boards for the AVR XMEGA microcontroller. The Xprotolab and Xminilab can be plugged directly on a breadboard.

Main Features:

    Mixed Signal Oscilloscope: Simultaneous sampling of 2 analog and 8 digital signals.
    Arbitrary Waveform Generator with advanced sweep options on all the wave parameters.
    Protocol Sniffer: SPI, I2C, UART
    Advanced Triggering System: Normal / Single / Auto / Free, with many trigger modes; adjustable trigger level, and ability to view signals prior to the trigger.
    Meter Mode: VDC, VPP and Frequency readout.
    XY Mode: For plotting Lissajous figures, V/I curves or checking the phase difference between two waveforms.
    Spectrum Analyzer with different windowing options and selectable vertical log and IQ visualization.
    Channel Math: add, multiply, invert, and average.
    Horizontal and Vertical Cursors with automatic waveform measurements, and waveform references.
    USB connectivity: Windows, Linux, MAC, Android

Overall hardware block diagram:
<img width="1152" height="644" alt="block_diagram1" src="https://github.com/user-attachments/assets/6e3b2a96-5775-4670-9c4b-7e0bb118d0ad" />

System architecture:
<img width="1152" height="865" alt="block_diagram2" src="https://github.com/user-attachments/assets/d11ea721-8a8b-46f6-adb0-a5151ff2d673" />

Frequency Counter:
<img width="1152" height="374" alt="block_diagram3" src="https://github.com/user-attachments/assets/fcd7b1bd-77ba-4a5d-b414-8ffe46d9d998" />

