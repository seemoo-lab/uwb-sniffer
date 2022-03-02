#!/usr/bin/env python3

# Copyright (c) 2012, George Oikonomou (oikonomou@users.sf.net)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the owner nor the names of its contributors may be
#     used to endorse or promote products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Read IEEE802.15.4 frames from a serial line (captured by a sniffer) and pipe
# them to wireshark. At the same time, the frames can be logged to a file for
# subsequent offline processing
# In interactive mode, the user can also input commands from stdin

# ToDo:
# * Configuration file support (ConfigParser ?)
import serial
import argparse
import os
import sys
import select
import time
import stat
import errno
import logging
import logging.handlers
import struct
import typing
import time 
from threading import Thread
from threading import Timer
#####################################
### Constants
#####################################
__version__ = '1.3'
#####################################
### Default configuration values
#####################################
defaults = {
    #'device': '/dev/ttyUSB0',
    'device': None,
    'channels': [1,3,5],
    #'baud_rate': 460800,
    'baud_rate': 921600,
    'out_file': 'sensniff.hexdump',
    'out_fifo': '/tmp/sensniff',
    'out_pcap': 'sensniff.pcap',
    'debug_level': 'WARN',
    'log_level': 'INFO',
    'log_file': 'sensniff.log',
}
#####################################
### PCAP and Command constants
#####################################
LINKTYPE_IEEE802_15_4_NOFCS = 230
LINKTYPE_IEEE802_15_4 = 195
LINKTYPE_IEEE802_15_4_TAP = 283
MAGIC_NUMBER = 0xA1B2C3D4
VERSION_MAJOR = 2
VERSION_MINOR = 4
THISZONE = 0
SIGFIGS = 0
SNAPLEN = 0xFFFF
NETWORK = LINKTYPE_IEEE802_15_4

PCAP_GLOBAL_HDR_FMT = '<LHHlLLL'
PCAP_FRAME_HDR_FMT = '<LLLL'

pcap_global_hdr = bytearray(struct.pack(PCAP_GLOBAL_HDR_FMT, MAGIC_NUMBER,
                                        VERSION_MAJOR, VERSION_MINOR,
                                        THISZONE, SIGFIGS, SNAPLEN, NETWORK))

def get_global_header(network:int) -> bytearray: 
    return bytearray(struct.pack(PCAP_GLOBAL_HDR_FMT, MAGIC_NUMBER,
                                        VERSION_MAJOR, VERSION_MINOR,
                                        THISZONE, SIGFIGS, SNAPLEN, network))

CMD_FRAME             = 0x00
CMD_CHANNEL           = 0x01
CMD_CHANNEL_MIN       = 0x02
CMD_CHANNEL_MAX       = 0x03
CMD_PREAMBLE_CODE     = 0x04
CMD_PRF               = 0x05
CMD_DEBUG_MSG         = 0x10
CMD_FRAME_TIME        = 0x20
CMD_ERR_NOT_SUPPORTED = 0x7F
CMD_GET_CHANNEL       = 0x81
CMD_GET_CHANNEL_MIN   = 0x82
CMD_GET_CHANNEL_MAX   = 0x83
CMD_SET_CHANNEL       = 0x84
CMD_SET_PREAMBLE_CODE = 0x85
CMD_SET_PRF_CODE = 0x86
SNIFFER_PROTO_VERSION = 2
#####################################
### Globals
#####################################
logger = logging.getLogger(__name__)

stats = {}
stop = False
#####################################

#TODO: Extend pcap output with channel informtion (https://github.com/jkcko/ieee802.15.4-tap/blob/master/IEEE%20802.15.4%20TAP%20Link%20Type%20Specification.pdf) 
class Frame(object):
    def __init__(self, raw, timestamp, channel=-1):
        self.__raw = raw
        self.__t = timestamp
        self.len = len(self.__raw)
        self.rx_channel = channel 

        self.__pcap_hdr = self.__generate_frame_hdr(self.len)

        self.pcap = bytearray(self.__pcap_hdr) + self.__raw
        self.hex = ''.join('%02x ' % c for c in self.__raw).rstrip()

    def __generate_frame_hdr(self, len: int):
        sec = int(self.__t)
        usec = int((self.__t - sec) * 1000000)
        return struct.pack(PCAP_FRAME_HDR_FMT,
                           sec, usec, len, len)

    def get_pcap(self):
        return self.pcap

    def get_pcap_tap(self):
        """
        Creates a pcap format that includes more state (currently only rx channel) information 
        """

        tap_tlvs = bytearray([])
        # FCS type TLV 
        fcs_tlv = bytearray((0).to_bytes(2,byteorder='little')) + bytearray((1).to_bytes(2, byteorder='little')) + bytearray([1]) + bytearray([0,0,0])
        tap_tlvs += fcs_tlv
        
        # Channel TLV
        if self.rx_channel > 0: 
            # Got channel information. Add this as a TLV 
            channel_tlv = bytearray((3).to_bytes(2, byteorder='little')) + bytearray((3).to_bytes(2, byteorder='little')) # Type 3 (Channel), Length 3 (excluding padding)
            channel_tlv += bytearray(self.rx_channel.to_bytes(2,byteorder='little')) #channel number
            channel_tlv += bytearray([0]) # Channel page
            channel_tlv += bytearray([0]) # padding
            tap_tlvs += channel_tlv

        header_len = len(tap_tlvs) + 4
        pcap_tap = bytearray([0,0])
        pcap_tap += bytearray(header_len.to_bytes(2, byteorder="little"))
        pcap_tap += tap_tlvs
        pcap_tap += self.__raw

        # GENERATE THE PACKET HEADER 
        
        pcap_header = self.__generate_frame_hdr(len(pcap_tap)) 
        pcap_tap = bytearray(pcap_header) + pcap_tap

        return pcap_tap


    def get_hex(self):
        return self.hex
#####################################
class SerialInputHandler(object):
    """
    Creates a serial connection to the SenSniff supported device. 
    The connection is used to receive sniffed frames and send commands to the device. 
    """
    def __init__(self,
                 port = defaults['device'],
                 baudrate = defaults['baud_rate'],
                 rts_cts = False):
        self.__sensniff_magic_legacy = bytearray((0x53, 0x6E, 0x69, 0x66))
        self.__sensniff_magic = bytearray((0xC1, 0x1F, 0xFE, 0x72))
        stats['Captured'] = 0
        stats['Non-Frame'] = 0
        stats['Timestamps'] = list()
        self.rx_channel = 0
        try:
            self.port = serial.Serial(port = port,
                                      baudrate = baudrate,
                                      bytesize = serial.EIGHTBITS,
                                      parity = serial.PARITY_NONE,
                                      stopbits = serial.STOPBITS_ONE,
                                      xonxoff = False,
                                      rtscts = rts_cts,
                                      timeout = 0.1)
            self.port.flushInput()
            self.port.flushOutput()
        except (serial.SerialException, ValueError, IOError, OSError) as e:
            logger.error('Error opening port: %s' % (port,))
            logger.error('The error was: %s' % (e.args,))
            sys.exit(1)
        logger.info('Serial port %s opened' % (self.port.name))

    def read_frame(self):
        try:
            # Read the magic + 1 more byte
            b = self.port.read(5)
            size = len(b)
        except (IOError, OSError) as e:
            logger.error('Error reading port: %s' % (self.port.port,))
            logger.error('The error was: %s' % (e.args,))
            sys.exit(1)

        if size == 0:
            return b
        if size < 5:
            logger.warning('Read %d bytes but not part of a frame'
                         % (size,))
            self.port.flushInput()
            return ''

        if b[0:4] not in (self.__sensniff_magic, self.__sensniff_magic_legacy):
            # Peripheral UART output - print it
            per_out = self.port.readline().rstrip()
            try:
                logger.info("Peripheral: %s%s" % (b.decode(), per_out.decode()))
            except UnicodeDecodeError as e:
                logger.info("Error decoding peripheral output: %s"%e)
            stats['Non-Frame'] += 1
            return ''

        # If we reach here:
        # Next byte == 1: Proto version 1, header follows
        # Next Byte != 1 && < 128. Old proto version. Frame follows, length == the byte
        b = bytearray(b)
        if b[4] != SNIFFER_PROTO_VERSION:
            # Legacy contiki sniffer support. Will slowly fade away
            size = b[4]
            try:
                b = self.port.read(size)
            except (IOError, OSError) as e:
                logger.error('Error reading port: %s' % (self.port.port,))
                logger.error('The error was: %s' % (e.args,))
                sys.exit(1)

            if len(b) != size:
                # We got the magic right but subsequent bytes did not match
                # what we expected to receive
                logger.warning('Read correct magic not followed by a frame')
                logger.warning('Expected %d bytes, got %d' % (size, len(b)))
                self.port.flushInput()
                return ''

            logger.info('Read a frame of size %d' % (len(b),))
            stats['Captured'] += 1
            return b

        # If we reach here, we have a packet of proto ver SNIFFER_PROTO_VERSION
        # Read CMD and LEN
        try:
            size = 0
            b = self.port.read(3)
            size = len(b)

        except (IOError, OSError) as e:
            logger.error('Error reading port: %s' % (self.port.port,))
            logger.error('The error was: %s' % (e.args[0],))
            sys.exit(1)

        if size < 3:
            logger.warning('Read correct magic not followed by a frame header')
            logger.warning('Expected 3 bytes, got %d' % (len(b), ))
            self.port.flushInput()
            return ''

        b = bytearray(b)
        cmd = b[0]
        length = b[1] << 8 | b[2]

        if cmd == CMD_ERR_NOT_SUPPORTED:
            logger.warning("Peripheral reports unsupported command")
            return ''

        # Read the frame or command response
        b = self.port.read(length)
        if len(b) != length:
            # We got the magic right but subsequent bytes did not match
            # what we expected to receive
            logger.warning('Read correct header not followed by a frame')
            logger.warning('Expected %d bytes, got %d' % (length, len(b)))
            self.port.flushInput()
            return ''

        # If we reach here, b holds a frame or a command response of length len
        if cmd == CMD_FRAME:
            logger.info('Read a frame of size %d' % (length,))
            stats['Captured'] += 1
            return b
        elif cmd == CMD_FRAME_TIME: 
            logger.info('Read frame of size %d' % (length-8,))
            stats['Captured'] +=1 

            # The last 8 bytes are used as a timestamp 
            timestamp_bytes = b[(len(b)-8):len(b)]
            timestamp = struct.unpack("Q", timestamp_bytes)[0]
            stats['Timestamps'].append(timestamp)
            frame = b[0:len(b)-8]

            return frame
        elif cmd == CMD_DEBUG_MSG:
            print('DBG: {}'.format(b.decode('ascii'))) 
            return ''

        # If we reach here, we have a command response
        b = bytearray(b)
        logger.debug('Received a command response: [%02x %02x]' % (cmd, b[0]))
        # We'll only ever see one of these if the user asked for it, so we are
        # running interactive. Print away
        if cmd == CMD_CHANNEL:
            self.rx_channel = b[0]
            print('Sniffing in channel: %d' % (b[0],))
        elif cmd == CMD_CHANNEL_MIN:
            print('Min channel: %d' % (b[0],))
        elif cmd == CMD_CHANNEL_MAX:
            print('Max channel: %d' % (b[0],))
        elif cmd == CMD_PREAMBLE_CODE:
            print('Current preamble code %d' % (b[0],))
        elif cmd == CMD_PRF: 
            print('Current PRF %dMHz' % (b[0],))
        
        else:
            logger.warning("Received a command response with unknown code")
        return ''

    def __write_command(self, cmd, len = 0, data = bytearray()):
        bytes = bytearray((SNIFFER_PROTO_VERSION, cmd))
        if len > 0:
            bytes += bytearray((len >> 8, len & 0xFF))
        if data is not None:
            bytes += data

        self.port.write(self.__sensniff_magic)
        self.port.write(bytes)
        self.port.flush()
        logger.debug('Sent bytes: [' +
                     ''.join('%02x ' % c for c in self.__sensniff_magic) +
                     ''.join('%02x ' % c for c in bytes).rstrip() +
                     ']')

    def set_channel(self, channel):
        self.__write_command(CMD_SET_CHANNEL, 1, bytearray((channel,)))

    def set_preamble_code(self, code): 
        self.__write_command(CMD_SET_PREAMBLE_CODE, 1, bytearray((code,)))

    def set_pulse_repetition_frequency(self, prf): 
        self.__write_command(CMD_SET_PRF_CODE, 1, bytearray((prf,)))

    def get_channel(self):
        self.__write_command(CMD_GET_CHANNEL)

    def get_channel_min(self):
        self.__write_command(CMD_GET_CHANNEL_MIN)

    def get_channel_max(self):
        self.__write_command(CMD_GET_CHANNEL_MAX)

#####################################
class AutodetectSerialInterfaces(object): 
    """
    Automatically finds sensniff supported devices. Is used when the user does not supply the serial ports
    """

    # def __init__(self): 

    def find_interfaces(self) -> typing.List[str]: 
        """
        Autodetect sensniff devices
        """
        all_devices = os.listdir("/dev")

        supported_devices = list(filter(lambda x: "cu.usb" in x, all_devices))
        return ["/dev/"+d for d in supported_devices]



#####################################
class FifoOutHandler(object):
    """
    pcap_tap = If true the TAP Format with channel information will be used 
    """
    def __init__(self, out_fifo, pcap_tap=False):
        self.out_fifo = out_fifo
        self.of = None
        self.needs_pcap_hdr = True
        self.pcap_tap = pcap_tap
        stats['Piped'] = 0
        stats['Not Piped'] = 0

        self.__create_fifo()

    def __create_fifo(self):
        try:
            os.mkfifo(self.out_fifo)
            logger.info('Opened FIFO %s' % (self.out_fifo,))
        except OSError as e:
            if e.errno == errno.EEXIST:
                if stat.S_ISFIFO(os.stat(self.out_fifo).st_mode) is False:
                    logger.error('File %s exists and is not a FIFO'
                                 % (self.out_fifo,))
                    sys.exit(1)
                else:
                    logger.warning('FIFO %s exists. Using it' % (self.out_fifo,))
            else:
                raise

    def __open_fifo(self):
        try:
            fd = os.open(self.out_fifo, os.O_NONBLOCK | os.O_WRONLY)
            self.of = os.fdopen(fd, 'wb')
        except OSError as e:
            if e.errno == errno.ENXIO:
                logger.warning('Remote end not reading')
                stats['Not Piped'] += 1
                self.of = None
                self.needs_pcap_hdr = True
            elif e.errno == errno.ENOENT:
                logger.error('%s vanished under our feet' % (self.out_fifo,))
                logger.error('Trying to re-create it')
                self.__create_fifo_file()
                self.of = None
                self.needs_pcap_hdr = True
            else:
                raise

    def handle(self, data):
        if self.of is None:
            self.__open_fifo()

        if self.of is not None:
            try:
                if self.needs_pcap_hdr is True:
                    if self.pcap_tap is True: 
                        self.of.write(get_global_header(LINKTYPE_IEEE802_15_4_TAP))
                    else: 
                        self.of.write(get_global_header(LINKTYPE_IEEE802_15_4))

                    # self.of.write(pcap_global_hdr)
                    self.needs_pcap_hdr = False

                if self.pcap_tap:
                    self.of.write(data.get_pcap_tap())
                else:
                    self.of.write(data.pcap)
                self.of.flush()
                logger.info('Wrote a frame of size %d bytes' % (data.len))
                stats['Piped'] += 1
            except IOError as e:
                if e.errno == errno.EPIPE:
                    logger.info('Remote end stopped reading')
                    stats['Not Piped'] += 1
                    self.of = None
                    self.needs_pcap_hdr = True
                else:
                    raise
#####################################
class PcapDumpOutHandler(object):
    def __init__(self, out_pcap, pcap_tap=False):
        """
        Handler to dump the received frames in a pcap file that should be read by wireshark. 
        pcap_tap: If true the TAP frame format will be used to deliver channel information. 
        """
        self.out_pcap = out_pcap
        self.pcap_tap = pcap_tap
        stats['Dumped to PCAP'] = 0

        try:
            self.of = open(self.out_pcap, 'wb')
            self.of.write(pcap_global_hdr)
            logger.info("Dumping PCAP to %s" % (self.out_pcap,))
        except IOError as e:
            self.of = None
            logger.warning("Error opening %s to save pcap. Skipping"
                         % (out_pcap))
            logger.warning("The error was: %d - %s"
                         % (e.args))

    def handle(self, frame):
        if self.of is None:
            return
        if self.pcap_tap:
            self.of.write(frame.get_pcap_tap())
        else: 
            self.of.write(frame.get_pcap())
        self.of.flush()
        logger.info('PcapDumpOutHandler: Dumped a frame of size %d bytes'
                     % (frame.len))
        stats['Dumped to PCAP'] += 1
#####################################
class HexdumpOutHandler(object):
    def __init__(self, of):
        stats['Dumped as Hex'] = 0
        try:
            self.of = open(of, 'w')
            logger.info("Dumping hex to %s" % (of,))
        except IOError as e:
            logger.warning("Error opening %s for hex dumps. Skipping"
                         % (of))
            logger.warning("The error was: %d - %s" % (e.args))
            self.of = None

    def handle(self, frame):
        if self.of is None:
            return

        try:
            self.of.write('0000 ')
            self.of.write(frame.get_hex())
            self.of.write('\n')
            self.of.flush()
            stats['Dumped as Hex'] += 1
            logger.info('HexdumpOutHandler: Dumped a frame of size %d bytes'
                         % (frame.len))
        except IOError as e:
            logger.warning("Error writing hex to %s for hex dumps. Skipping"
                     % (self.of))
            logger.warning("The error was: %d - %s" % (e.args))
#####################################
def arg_parser():
    debug_choices = ('DEBUG', 'INFO', 'WARN', 'ERROR')

    parser = argparse.ArgumentParser(add_help = False,
                                     description = 'Read IEEE802.15.4 frames \
    from a sensniff enabled device, convert them to pcap and pipe them \
    into wireshark over a FIFO pipe for online analysis. Frames \
    can also be saved in a file in hexdump and/or pcap format for offline \
    analysis.')

    in_group = parser.add_argument_group('Serial Line Options')
    in_group.add_argument('-b', '--baud', type = int, action = 'store',
                          default = defaults['baud_rate'],
                          help = 'Set the line\'s baudrate to BAUD. \
                                  Only makes sense with -d. \
                                  (Default: %s)' % (defaults['baud_rate'],))
    


    in_group.add_argument('-r', '--rts-cts', action = 'store_true',
                          default = False,
                          help = 'Set to enable H/W flow control \
                                  (Default: Flow control disabled.)')

    out_group = parser.add_argument_group('Output Options')
    out_group.add_argument('-o', '--out-file', action = 'store', nargs = '?',
                           const = defaults['out_file'], default = False,
                           help = 'Save the capture (hexdump) file OUT_FILE. \
                                   If -o is specified but OUT_FILE is omitted, \
                                   stdout will be used. If the argument is \
                                   omitted altogether, the capture will not \
                                   be saved.')
    out_group.add_argument('-p', '--pcap', action = 'store', nargs = '?',
                           const = defaults['out_pcap'], default = False,
                           help = 'Save the capture (pcap format) in PCAP. \
                                   If -p is specified but PCAP is omitted, \
                                   %s will be used. If the argument is \
                                   omitted altogether, the capture will not \
                                   be saved.' % (defaults['out_pcap'],))

    out_group.add_argument('-pt', '--pcap-tap', action='store_true', default=False,
                            help="""
                                Use a pcap tap header format that includes more
                                information on the received frames,
                                like the channel it has been received ob 
                                 """)
    out_group.add_argument('-F', '--fifo', action = 'store',
                           default = defaults['out_fifo'],
                           help = 'Pipe the capture through FIFO \
                                   (Default: %s)' % (defaults['out_fifo'],))
    out_group.add_argument('-O', '--offline', action = 'store_true',
                           default = False,
                           help = 'Disable piping (Mainly used for debugging) \
                                   (Default: Piping enabled)')

    log_group = parser.add_argument_group('Verbosity and Logging')
    log_group.add_argument('-n', '--non-interactive', action = 'store_true',
                           default = False,
                           help = 'Run in non-interactive mode, without \
                                   accepting user input. (Default Disabled)')
    log_group.add_argument('-D', '--debug-level', action = 'store',
                           choices = debug_choices,
                           default = defaults['debug_level'],
                           help = 'Print messages of severity DEBUG_LEVEL \
                                   or higher (Default %s)'
                                   % (defaults['debug_level'],))
    log_group.add_argument('-L', '--log-file', action = 'store', nargs = '?',
                           const = defaults['log_file'], default = False,
                           help = 'Log output in LOG_FILE. If -L is specified \
                                   but LOG_FILE is omitted, %s will be used. \
                                   If the argument is omitted altogether, \
                                   logging will not take place at all.'
                                   % (defaults['log_file'],))
    log_group.add_argument('-l', '--log-level', action = 'store',
                           choices = debug_choices,
                           default = defaults['log_level'],
                           help = 'Log messages of severity LOG_LEVEL or \
                                   higher. Only makes sense if -L is also \
                                   specified (Default %s)'
                                   % (defaults['log_level'],))


    dev_group = parser.add_argument_group('Device options')

    # Multidevice
    dev_group.add_argument('-d', '--device', action = 'store',
                          default = defaults['device'], nargs="*", type=str,
                          help = 'Read from device DEVICE \
                                  (Default: %s). \
                                  Add multiple devices if you want to sniff on multiple channels.\
                                  Multiple devices will automatically activate non-interactive mode ' % (defaults['device'],))

    # Autodetect 
    dev_group.add_argument('-a', '--autodetect', action='store_true',
                            default=False,
                            help = 'In this mode the connect sniffer device\
                                 will switch through all available PRFs\
                                      (16 MHz & 64MHz) and preamble codes (0-31)\
                                           and try to find other devices sending data.'
    )


    gen_group = parser.add_argument_group('General Options')
    gen_group.add_argument('-v', '--version', action = 'version',
                           version = 'sensniff v%s' % (__version__))
    gen_group.add_argument('-h', '--help', action = 'help',
                           help = 'Shows this message and exits')



    return parser.parse_args()
#####################################
def dump_stats():
    print('Frame Stats:')
    for k, v in list(stats.items()):
        print('%20s: %d' % (k, v))
#####################################
def log_init():
    logger.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setLevel(getattr(logging, args.debug_level))
    cf = logging.Formatter('%(message)s')
    ch.setFormatter(cf)
    logger.addHandler(ch)

    if args.log_file is not False:
        fh = logging.handlers.RotatingFileHandler(filename = args.log_file,
                                                  maxBytes = 5000000)
        fh.setLevel(getattr(logging, args.log_level))
        ff = logging.Formatter(
            '%(asctime)s - %(levelname)8s - %(message)s')
        fh.setFormatter(ff)
        logger.addHandler(fh)

#####################################

def interact_with_serial(input_handler: SerialInputHandler, out_handlers: typing.List):

    while 1:
        if args.non_interactive is False:
            try:
                if select.select([sys.stdin, ], [], [], 0.0)[0]:
                    cmd = sys.stdin.readline().strip()
                    logger.info('User input: "%s"' % (cmd,))
                    if cmd in ('h', '?'):
                        print(help_str)
                    elif cmd == 'c':
                        input_handler.get_channel()
                    elif cmd == 'm':
                        input_handler.get_channel_min()
                    elif cmd == 'M':
                        input_handler.get_channel_max()
                    elif cmd == 'n':
                        f.needs_pcap_hdr = True
                    elif cmd == 'q':
                        logger.info('User requested shutdown')
                        dump_stats()
                        sys.exit(0)
                    elif "p " in cmd:
                        code = cmd.split(" ")[1]
                        if int(code) in range(0, 0xFFFF): 
                            input_handler.set_preamble_code(int(code))
                    elif "r " in cmd: 
                        rate = cmd.split(" ")[1]
                        if int(rate) in range(0,128):
                            input_handler.set_pulse_repetition_frequency(int(rate))
                    elif int(cmd) in range(0, 0xFFFF):
                        input_handler.set_channel(int(cmd))
                    else:
                        raise ValueError
            except select.error:
                logger.warning('Error while trying to read stdin')
            except ValueError:
                print(err_str)
            except UnboundLocalError:
                # Raised by command 'n' when -o was specified at command line
                pass
        

        try:
            raw = input_handler.read_frame()
            
            if len(raw) > 0:
                t = time.time()
                timestamps = stats['Timestamps']
                t_len = len(timestamps)
                if t_len > 0 and timestamps[t_len-1] > 0: 
                    # Fetch the last timestamp and convert it to seconds. 
                    t = timestamps[t_len-1] / 1000000 

                frame = Frame(bytearray(raw), t, channel=input_handler.rx_channel)
                for h in out_handlers:
                    h.handle(frame)
        except (KeyboardInterrupt, SystemExit):
            logger.info('Shutting down')
            dump_stats()
            sys.exit(0)

#####################################

def serial_read(input_handler: SerialInputHandler, out_handlers: typing.List): 
    """
    Read from serial without interaction 
    """
    while not stop: 
        raw = input_handler.read_frame() 
        
        if len(raw) > 0: 
            t = time.time() 
            timestamps = stats['Timestamps']
            t_len = len(timestamps)
            if t_len > 0 and timestamps[t_len-1] > 0: 
                # Fetch the last timestamp and convert it to seconds.
                t = timestamps[t_len-1] / 1000000

            frame = Frame(bytearray(raw), t, channel=input_handler.rx_channel)
            for h in out_handlers: 
                h.handle(frame)


#####################################



if __name__ == '__main__':
    args = arg_parser()
    log_init()

    logger.info('Started logging')

    input_handler: SerialInputHandler = None 
    in_handlers = []
    devices: typing.List[str] = list()
    if not args.device: 
        # Find devices 
        devices = AutodetectSerialInterfaces().find_interfaces() 
    else: 
        devices = args.device
    
    if len(devices) == 1:  
        input_handler = SerialInputHandler(port = devices[0], baudrate = args.baud,
                                    rts_cts = args.rts_cts)
        time.sleep(2)
        input_handler.set_channel(args.channels[0])
        in_handlers.append(input_handler)

    else: 
        # Sniff on multiple devices 
        for device in devices: 
            s = SerialInputHandler(port = device, baudrate = args.baud, rts_cts = args.rts_cts)
            time.sleep(2)

            in_handlers.append(s)
        
        args.non_interactive = True
        

    out_handlers = []
    if args.offline is not True:
        f = FifoOutHandler(out_fifo = args.fifo, pcap_tap=args.pcap_tap)
        out_handlers.append(f)
    if args.out_file is not False:
        out_handlers.append(HexdumpOutHandler(of = args.out_file))
    if args.pcap is not False:
        out_handlers.append(PcapDumpOutHandler(args.pcap, args.pcap_tap))

    if args.non_interactive is False:
        help_str = ("Commands:\n"
                    "h,?: Print this message\n"
                    "q: Quit")

        err_str = 'Unknown Command. Type h or ? for help'

        print(help_str)

    threads = list() 
    if len(in_handlers) == 1: 
        if not args.non_interactive:
           interact_with_serial(input_handler, out_handlers)
        else: 
            t = Thread(target=serial_read, args=(i,out_handlers))
            threads.append(t)
            t.start() 
    else: 
        for i in in_handlers: 
            t = Thread(target=serial_read, args=(i,out_handlers))
            threads.append(t)
            t.start() 

    try: 
        logger.info("Start sniffing")
        # for t in threads: 
        #     t.join()
        while 1: 
            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit): 
        logger.info('Shutting down')
        stop = True
        dump_stats()
        sys.exit(0)


