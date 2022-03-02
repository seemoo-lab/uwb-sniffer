sensniff
========

This code is based on the sensniff project: https://github.com/g-oikonomou/sensniff 

*Live Traffic Capture and Sniffer for IEEE 802.15.4 networks.*

This tool helps you perform live traffic capture and analysis for IEEE 802.15.4 networks. sensniff has two components:

 * **Peripheral**: This is an embedded device with a .15.4 trasceiver which captures all network frames and streams them over to the host.
 * **Host**: This is a python script which runs on a PC. It reads network packets captured by the peripheral, converts them to PCAP and pipes them to [wireshark](http://www.wireshark.org/).

Other than network packet capture, the host can send commands to the peripheral to achieve secondary functionality e.g. change radio channel.


sensniff has been developed and tested on Ubuntu and Mac OS X. sensniff does not work on Windows (and most likely never will).

How to Use
==========
In a nutshell, three steps are involved:
 * Program your peripheral with the UART compatible firmware for the DW3000 boarfd 
 * Run the sensniff python script `python3 sensniff.py -d /dev/tty...`
 * Run wireshark, start a capture at `/tmp/sensniff` pipe and enjoy

Run the Host Tool
-----------------
The host-side tool assumes that the peripheral appears as a serial port on the host PC. If your embedded device has a native USB interface, it will have to enumerate as a CDC-ACM device (e.g. the CC2531 USB dongle running the Contiki sniffer example).

The best way to start:
`python3 sensniff.py -h`

Some examples:
 * To read captures from `/dev/ttyUSB1`:
   `python3 sensniff.py -d /dev/ttyUSB1`
 * To run in non-interactive mode:
   `python3 sensniff.py --non-interactive`
 * To increase verbosity:
   `python3 sensniff.py -D INFO`
 * Use the `-p` argument to save the capture in a pcap file:
   `python3 sensniff.py -p`

**The current version does not support channel or preamble switches as this would use too much time** 

The host-side script will also print out peripheral debugging output. Any data received not starting with the correct MAGIC (see protocol specification) will be considered to be debugging output from the peripheral and will be printed verbatim, prefixed by 'Peripheral: '. Thus, you may see something like this:

    Peripheral: sniffer: Command 0x82
    Peripheral: sniffer: SET_CHANNEL command
    Peripheral: sniffer: Channel 12
    Received a command response: [01 0c]
    Sniffing in channel: 12
    Peripheral: sniffer: Response [ 53 6e 69 66 01 01 01 0c ]

Run Wireshark
-------------
The host-side tool will convert the frames to PCAP format and pipe them to a FIFO file. All you need to do is to set wireshark to start a capture, using this FIFO file as the capture 'interface'. By default, sensniff will use `/tmp/sensniff`.

Go to Capture -> options -> Manage Interfaces -> New (under Pipes) -> type `/tmp/sensniff` and save. The pipe will then appear as an interface. Start a capture on it.

In older versions of Wireshark, go to Capture -> Options and type `/tmp/sensniff` in the Interface field.

You don't need root priviledges.

The first time your run this, you will need to open Wireshark's preferences and select 'TI CC24xx FCS format' under Protocols -> IEEE 802.15.4. You will also need to correctly configure contexts under 6LoWPAN.


sensniff Host-to-Peripheral protocol
====================================
sensniff uses a minimalistic protocol for the communication between the host and the peripheral. All packets (in both directions) follow this format:

                         1                   2                   3
     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |                             MAGIC                             |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |    VERSION    |      CMD      |              LEN              |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |                             DATA                              |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

 * **MAGIC**: The following 4 bytes (hex): C1 1F FE 72 ('S'+'n' 1F FE 'r')
 * **VERSION**: (1 byte). Currently 2.
 * **CMD**: (1 byte) Command. See below for possible values.
 * **LEN**: (2 bytes) Length of the DATA field in number of bytes. Optional.
 Network byte order.
 * **DATA**: Variable length specified in LEN. Only transmitted if LEN exists and has value > 0. Contains the payload, depending on the value of CMD.

Commands
--------
Generally speaking, frames with the MS bit of the CMD field set are host-to-peripheral. The MS bit is clear for peripheral-to-host packets.

The CMD field can take the following values:
   * **CMD==0x00 (CMD_FRAME)**: LEN will contain the length of a captured .15.4 frame. DATA will contain the frame itself, including the .15.4 MAC layer header, payload and FCS. This command is peripheral-initiated. 
   * **CMD==0x01 (CMD_CHANNEL)**: The current RF channel used by the peripheral's transceiver. LEN will be 1. DATA will be 1 byte long and will contain the value of the channel. Valid values in [11,26]. Packets of this type are always a response to either CMD_GET_CHANNEL or CMD_SET_CHANNEL. *Not supported by current UWB Sniffer*
   * **CMD==0x02 (CMD_CHANNEL_MIN)**: The minimum RF channel supported by the peripheral's transceiver. LEN will be 1. DATA will be 1 byte long and will contain the value of the channel. Packets of this type are a response to CMD_GET_CHANNEL_MIN. *Not supported by current UWB Sniffer*
   * **CMD==0x03 (CMD_CHANNEL_MAX)**: The maximum RF channel supported by the peripheral's transceiver. LEN will be 1. DATA will be 1 byte long and will contain the value of the channel. Packets of this type are a response to CMD_GET_CHANNEL_MAX. *Not supported by current UWB Sniffer*
   * **CMD==0x04 (CMD_PREAMBLE_CODE)**: The currently used preamble code. *Not supported by current UWB Sniffer*
   * **CMD==0x05 (CMD_PRF)**: The currently used prf. *Not supported by current UWB Sniffer*
   * **CMD==0x10 (CMD_DEBUG_PRINT)**: Print any debug message. LEN is the length of the debug message sent. *Not supported by current UWB Sniffer*
   * **CMD==0x20 (CMD_FRAME_TIME)**: LEN will contain the length of a captured 15.4 frame **plus** the 64bit (8byte) uint timestamp. DATA will contain the frame itself, including the .15.4 MAC layer header, payload and FCS followed by a 64bit timestamp value. This command is peripheral-initiated. 
   * **CMD==0x7F (CMD_ERR_NOT_SUPPORTED)**: The peripheral parsed the command successfully, but it could not execute it. A typical example of when this could happen is when a CMD_SET_CHANNEL requested a channel outside the range supported by the peripheral.
   * **CMD==0x81 (CMD_GET_CHANNEL)**: Used by the host to query the current radio channel used by the peripheral's RF chip. LEN and DATA are omitted. The Peripheral will respond with a CMD_CHANNEL. *Not supported by current UWB Sniffer*
   * **CMD==0x82 (CMD_GET_CHANNEL_MIN)**: Used by the host to query the minimum radio channel supported by the peripheral's RF chip. LEN and DATA are omitted. The Peripheral will respond with a CMD_CHANNEL_MIN. *Not supported by current UWB Sniffer*
   * **CMD==0x83 (CMD_GET_CHANNEL_MAX)**: Used by the host to query the maximum radio channel supported by the peripheral's RF chip. LEN and DATA are omitted. The Peripheral will respond with a CMD_CHANNEL_MAX. *Not supported by current UWB Sniffer*
   * **CMD==0x84 (CMD_SET_CHANNEL)**: Used by the host to request a change to a new radio channel. LEN will be 1. DATA will be 1 byte long and will contain the value of the new channel. Valid values depend on the peripheral and can be retrieved through CMD_GET_CHANNEL_MIN & CMD_GET_CHANNEL_MAX. The peripheral will respond with a CMD_CHANNEL. *Not supported by current UWB Sniffer*

**The current version does not support channel or preamble switches as this would use too much time** 