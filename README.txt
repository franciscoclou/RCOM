INSTRUCTIONS FOR SERIAL PORT PROTOCOL
=====================================

This folder contains the base code of the serial port protocol.

Project Structure
-----------------

- bin/: Compiled binaries.
- src/: Source code for the implementation of the link-layer and application layer protocols. Students should edit these files to implement the project.
- cable/: Virtual cable program to help test the serial port. This file must not be changed.
- Makefile: Makefile to build the project and run the application.
- penguin.gif: Example file to be sent through the serial port.

Instructions to Run the Project
-------------------------------

1. Edit the source code in the src/ directory.
2. Compile the application and the virtual cable program using the provided Makefile.
3. Run the virtual cable program (either by running the executable manually or using the Makefile target).
   Note that the virtual cable program requires the installation of "socat".
    (Option 1) $ ./bin/cable_app
    (Option 2) $ make run_cable

4. Test the protocol without cable disconnections and noise
    4.1 Run the receiver (either by running the executable manually or using the Makefile target):
        (Option 1) $ ./bin/main /tmp/ttyS11 9600 rx penguin-received.gif
        (Option 2) $ make run_rx

    4.2 Run the transmitter (either by running the executable manually or using the Makefile target):
        (Option 1) $ ./bin/main /tmp/ttyS10 9600 tx penguin.gif
        (Option 2) $ make run_tx

    4.3 Check if the file received matches the file sent, using the diff Linux command or using the Makefile target:
        (Option 1) $ diff -s penguin.gif penguin-received.gif
        (Option 2) $ make check_files

5. Test the protocol with cable disconnections and noise
    5.1. Run receiver and transmitter again
    5.2. Quickly move to the cable program console and disable the connection and/or add noise. Check the instructions printed at the start of the program.
    5.3. Check if the file received matches the file sent, even with cable disconnections or with noise

Running with the Virtual Cable using the Makefile
-------------------------------------------------

The Makefile targets use the lab serial ports by default (/dev/ttyUSB0 and /dev/ttyS0).
To use the virtual cable, override the ports on the command line (the Makefile is not changed):
    $ make run_cable
    $ make run_rx RX_SERIAL_PORT=/tmp/ttyS11
    $ make run_tx TX_SERIAL_PORT=/tmp/ttyS10
    $ make check_files

To use another baud rate, set it in the cable console first ("baud 2400") and then use
the same value for the receiver and the transmitter:
    $ make run_rx RX_SERIAL_PORT=/tmp/ttyS11 BAUD_RATE=2400
    $ make run_tx TX_SERIAL_PORT=/tmp/ttyS10 BAUD_RATE=2400

Useful cable console commands: on, off, ber <ber>, baud <rate>, prop <usec>, quit.

Protocol Statistics
-------------------

When the connection is closed, llCloseTx() and llCloseRx() print the protocol statistics:
I frames sent/received, retransmissions, timeouts, RR/REJ sent/received, duplicates and bytes.

Measuring the Transfer Time
---------------------------

The statistics do not include the elapsed time. To measure it, start the receiver first and
run the transmitter with the shell's "time" command (build with "make" before, so that the
compilation is not counted):
    $ time ./bin/main /tmp/ttyS10 9600 tx penguin.gif
    or
    $ time make run_tx TX_SERIAL_PORT=/tmp/ttyS10

The elapsed time is the "real" value printed at the end. For the efficiency study:
    R = file size (bytes) * 8 / real      (penguin.gif has 10968 bytes)
    S = R / C                             (C = baud rate)
