import asyncio
import functools
import logging
import re
import serial
import socket
from functools import wraps
from serial_asyncio import create_serial_connection
from threading import RLock

_LOGGER = logging.getLogger(__name__)
ZONE_PATTERN_ON = re.compile("\D\D\D\s(\d\d)\D\D\d\d\s\s\D\D\D\s(\d\d)\D\D\d\d\s")
ZONE_PATTERN_OFF = re.compile("\D\D\DOFF\D\D\d\d\s\s\D\D\D\D\D\D\D\D\d\d\s")
ZONE_OUTPUT_STATE_PATTERN = re.compile("=    Video Output (?P<outNumber>\d+) : Input = (?P<inputNumber>\d+), Output = (?P<outputState>\w+) , LINK = (?P<linkState>\w+)")

EOL = b"\r"
LEN_EOL = len(EOL)
TIMEOUT = 2  # Number of seconds before serial operation timeout
TYPE_0_PORT = 4001  # used by more modern 8x8 matrix
TYPE_1_PORT = 23  # used by some older 4x4 matrix like the PID 15779
SOCKET_RECV = 2048

# commands available for TYPE 1 interface
# =========================================================================================================
# =*********************************************Systems HELP**********************************************=
# =-------------------------------------------------------------------------------------------------------=
# =              Systems information       System ID : 0008(H)             F/W Version : 1.00             =
# =-------------------------------------------------------------------------------------------------------=
# =-------------------------------------------------------------------------------------------------------=
# =    RH                           :  Help                                                               =
# =    WSPF                         :  Power Off                                                          =
# =    WSPN                         :  Power On                                                           =
# =    RSTA                         :  Show Global System Status                                          =
# =    WS DBG EN/DIS                :  DEBUG Mode Enable/Disable                                          =
# =    WS DF                        :  Reset to Factory Defaults                                          =
# =-------------------------------------------------------------------------------------------------------=
# = Video Output Setup Commands:                                                                          =
# =          (x1 = [01 - 04], x2 = [01 -04] [EN=Enable, DIS=Disable] )                                    =
# =    WVSO[x1]I[x2]                :  Set Output x1 to Video Input x2                                    =
# =    WVSOA[x2]                    :  Set All Output to Video Input x2                                   =
# =    WVSO[x1]ON/OFF               :  Set Output x1 ON/OFF                                               =
# =    WVSOAON/OFF                  :  Set All Output ON/OFF                                              =
# =-------------------------------------------------------------------------------------------------------=
# = Audio Output Setup Commands: (x1 = [01-04], [EN=Enable, DIS=Disable])                                 =
# =    WASO[x1]E EN/DIS             :  Enable/Disable External Audio Output x1                            =
# =    WASOAE EN/DIS                :  Enable/Disable All External Audio Output x1                        =
# =-------------------------------------------------------------------------------------------------------=
# = EDID Setup: (x1 = [01-04], x2 = [01-04], x3 = [01 - 12], x4 = [01 - 03])                              =
# =    WECO [x1]I[x2]               :  Copy EDID from Ouput x1 to Input x2                                =
# =    WECO [x1]A                   :  Copy EDID from Ouput x1 to All Input                               =
# =    WECD [x3]I[x2]               :  Copy EDID from Default EDID x3 to Input x2                         =
# =    WECD [x3]A                   :  Copy EDID from Default EDID x3 to All Input                        =
# =    WECU [x4]I[x2]               :  Copy EDID from User[x4] EDID of Input[x2] to Input x2              =
# =    WECU [x4]A                   :  Copy EDID from each Input's User[x4] EDID to each Input            =
# =    WEWI[x2]U[x4]O[x1]           :  Copy EDID from Output[x1] to User[x4] EDID of Input x2             =
# =    WEWIAU[x4]O[x1]              :  Copy EDID from Output[x1] to User[x4] EDID of each Input           =
# =    WEWI[x2]U[x4]L[x5]D{XX,▒▒}   :  Copy EDID Data{XX,...} that the lenght is x5 to  User[x4] EDID of  =
# =                                 :   Input x2. The format of EDID Data is Hex.                         =
# =    WEWIAU[x4]L[x5]D{XX,▒▒}      :  Copy EDID Data{XX,...} that the lenght is x5 to  User[x4] EDID of  =
# =                                 :   each Input. The format of EDID Data is Hex.                       =
# =-------------------------------------------------------------------------------------------------------=
# = Network Setup: ( xxx=[000-255], zzzz=[0001~9999] )                                                    =
# =    WIPDP ON/OFF                 :  Set DHCP ON/OFF                                                    =
# =    WIPH xxx.xxx.xxx.xxx         :  Set Host IP Address to xxx.xxx.xxx.xxx                             =
# =    WIPN xxx.xxx.xxx.xxx         :  Set Net Mask to xxx.xxx.xxx.xxx                                    =
# =    WIPR xxx.xxx.xxx.xxx         :  Set Route IP Address to xxx.xxx.xxx.xxx                            =
# =    WIPP zzzz                    :  Set TCP/IP Port to zzzz                                            =
# =-------------------------------------------------------------------------------------------------------=
# = Read Status: ([D x1]=Default edid x1 [U x1]=User EDID x1 )                                            =
# =    R8001                        :  Read INPUT Link States                                             =
# =    R8002                        :  Read OUTPUT Link States                                            =
# =    R8003                        :  Read INPUT HDCP States                                             =
# =    R8004                        :  Read OUTPUT HDCP States                                            =
# =    R8006                        :  Read OUTPUT Channel Set States                                     =
# =    R8007                        :  Read OUTPUT ON/OFF States                                          =
# =    R8008                        :  Read External Audio Output Enable States                           =
# =    R8009                        :  Read INPUT EDID Set States                                         =
# =    R8010[x1]                    :  Read INPUT x1 EDID Data                                            =
# =    R8011[x1]                    :  Read OUTPUT x1 EDID Data                                           =
# =    R8012                        :  Read Network States                                                =
# =-------------------------------------------------------------------------------------------------------=
# =*******************************************************************************************************=
# =========================================================================================================


class ZoneStatus(object):
    def __init__(self, zone: int, power: bool, av: int, ir: int):
        self.zone = zone
        self.power = power
        self.av = av
        self.ir = ir

    @classmethod
    def from_string(cls, matrix_type: int, zone: int, string: str):
        if not string:
            return None
        if matrix_type == 0:
            match_on = re.search(ZONE_PATTERN_ON, string)
            if not match_on:
                match_off = re.search(ZONE_PATTERN_OFF, string)
                if not match_off:
                    return None
                return ZoneStatus(zone, 0, None, None)
            return ZoneStatus(zone, 1, *[int(m) for m in match_on.groups()])
        elif matrix_type == 1:
            match_out_set = ZONE_OUTPUT_STATE_PATTERN.findall(string)
            if match_out_set:
                p = match_out_set[zone-1]
                if not p or int(p[0]) != zone:
                    return None

                if p[2] == "ON":
                    return ZoneStatus(zone, 1, int(p[1]), 0)
                else:
                    return ZoneStatus(zone, 0, int(p[1]), 0)
        else:
            return None


class LockStatus(object):
    def __init__(self, lock: bool):
        self.lock = lock

    @classmethod
    def from_string(cls, string: str):
        if not string:
            return None
        # string = string[7:].rstrip()
        if string.startswith("System Locked"):
            return True
        else:
            return False


class Blackbird(object):
    """
    Monoprice blackbird amplifier interface
    """

    def zone_status(self, zone: int):
        """
        Get the structure representing the status of the zone
        :param zone: zone 1..8
        :return: status of the zone or None
        """
        raise NotImplemented()

    def set_zone_power(self, zone: int, power: bool):
        """
        Turn zone on or off
        :param zone: Zone 1-8
        :param power: True to turn on, False to turn off
        """
        raise NotImplemented()

    def set_zone_source(self, zone: int, source: int):
        """
        Set source for zone
        :param zone: Zone 1-8
        :param source: integer from 1-8
        """
        raise NotImplemented()

    def set_all_zone_source(self, source: int):
        """
        Set source for all zones
        :param source: integer from 1-8
        """
        raise NotImplemented()

    def lock_front_buttons():
        """
        Lock front panel buttons
        """
        raise NotImplemented()

    def unlock_front_buttons():
        """
        Unlock front panel buttons
        """
        raise NotImplemented()

    def lock_status():
        """
        Report system locking status
        """
        raise NotImplemented()


# Helpers
def _format_zone_status_request(matrix_type: int, zone: int) -> bytes:
    if matrix_type == 0:
        return "Status{}.\r".format(zone).encode()
    elif matrix_type == 1:
        #return f">@R8006\r\n".encode()
        #this will return status for all zones, so not optimal to single zone reqeust
        return f">@RSTA\r\b".encode()
    else:
        raise NotImplemented()


def _format_set_zone_power(matrix_type: int, zone: int, power: bool) -> bytes:
    if matrix_type == 0:
        return "{}{}.\r".format(zone, "@" if power else "$").encode()
    elif matrix_type == 1:
        if power:
            strCmd = "ON"
        else:
            strCmd = "OFF"
        return f">@WVSO[{zone}]{strCmd}\r\n".encode()
    else:
        raise NotImplemented()


def _format_set_zone_source(matrix_type, zone: int, source: int) -> bytes:
    if matrix_type == 0:
        source = int(max(1, min(source, 8)))
        return "{}B{}.\r".format(source, zone).encode()
    elif matrix_type == 1:
        source = int(max(1, min(source, 4)))
        return f">@WVSO[{zone}]I[{source}]\r\n".encode()
    else:
        raise NotImplemented()


def _format_set_all_zone_source(matrix_type: int, source: int) -> bytes:
    if matrix_type == 0:
        source = int(max(1, min(source, 8)))
        return "{}All.\r".format(source).encode()
    elif matrix_type == 1:
        source = int(max(1, min(source, 4)))
        return f">@WVSOA[{source}]\r\n".encode()
    else:
        raise NotImplemented()


def _format_lock_front_buttons(matrix_type: int) -> bytes:
    if matrix_type == 0:
        return "/%Lock;\r".encode()
    else:
        raise NotImplemented()


def _format_unlock_front_buttons(matrix_type: int) -> bytes:
    if matrix_type == 0:
        return "/%Unlock;\r".encode()
    else:
        raise NotImplemented()


def _format_lock_status(matrix_type: int) -> bytes:
    if matrix_type == 0:
        return "%9961.\r".encode()
    else:
        raise NotImplemented()


def get_blackbird(url, use_serial=True, matrix_type:int=0):
    """Return synchronous version of Blackbird interface.

    :param port_url: serial port, i.e. '/dev/ttyUSB0'
    :param matrix_type: type of matrix swtich to communicate to
    :return: synchronous implementation of Blackbird interface
    """
    lock = RLock()
    print(serial)
    if matrix_type == 1:
        EOL = b"\r\n"
        LEN_EOL = len(EOL)
    else:
        EOL = b"\r"
        LEN_EOL = len(EOL)

    def synchronized(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            with lock:
                return func(*args, **kwargs)

        return wrapper

    class BlackbirdSync(Blackbird):
        def __init__(self, url, matrix_type):
            """Initialize the client."""
            self._matrix_type = matrix_type

            if use_serial:
                self._port = serial.serial_for_url(url, do_not_open=True)
                self._port.baudrate = 9600
                self._port.stopbits = serial.STOPBITS_ONE
                self._port.bytesize = serial.EIGHTBITS
                self._port.parity = serial.PARITY_NONE
                self._port.timeout = TIMEOUT
                self._port.write_timeout = TIMEOUT
                self._port.open()

            else:
                self.host = url
                if self._matrix_type == 0:
                    self.port = TYPE_0_PORT
                elif self._matrix_type == 1:
                    self.port = TYPE_1_PORT
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(TIMEOUT)
                self.socket.connect((self.host, self.port))

                # Clear login message
                self.socket.recv(SOCKET_RECV)

        def _process_request(self, request: bytes, skip=0):
            """Send data to socket.

            :param request: request that is sent to the blackbird
            :param skip: number of bytes to skip for end of transmission decoding
            :return: ascii string returned by blackbird.
            """
            _LOGGER.debug('Sending "%s"', request)

            if use_serial:
                # clear
                self._port.reset_output_buffer()
                self._port.reset_input_buffer()
                # send
                self._port.write(request)
                self._port.flush()
                # receive
                result = bytearray()
                while True:
                    c = self._port.read(1)
                    if c is None:
                        break
                    if not c:
                        raise serial.SerialTimeoutException(
                            "Connection timed out! Last received bytes {}".format(
                                [hex(a) for a in result]
                            )
                        )
                    result += c
                    if len(result) > skip and result[-LEN_EOL:] == EOL:
                        break
                ret = bytes(result)
                _LOGGER.debug('Received "%s"', ret)
                return ret.decode("ascii")

            else:
                self.socket.send(request)

                response = ""

                while True:
                    data = self.socket.recv(SOCKET_RECV)
                    response += data.decode("ascii")

                    if EOL in data and len(response) > skip:
                        break

                return response

        @synchronized
        def zone_status(self, zone: int):
            # Returns status of a zone
            if self._matrix_type == 0:
                return ZoneStatus.from_string(
                    self._matrix_type,
                    zone,
                    self._process_request(
                        _format_zone_status_request(self._matrix_type, zone), skip=15
                    ),
                )
            elif self._matrix_type == 1:
                return ZoneStatus.from_string(
                    self._matrix_type,
                    zone,
                    self._process_request(
                        _format_zone_status_request(self._matrix_type, zone), skip=3835
                    ),
                )
            else:
                return None


        @synchronized
        def set_zone_power(self, zone: int, power: bool):
            # Set zone power
            self._process_request(
                _format_set_zone_power(self._matrix_type, zone, power)
            )

        @synchronized
        def set_zone_source(self, zone: int, source: int):
            # Set zone source
            self._process_request(
                _format_set_zone_source(self._matrix_type, zone, source)
            )

        @synchronized
        def set_all_zone_source(self, source: int):
            # Set all zones to one source
            self._process_request(
                _format_set_all_zone_source(self._matrix_type, source)
            )

        @synchronized
        def lock_front_buttons(self):
            # Lock front panel buttons
            self._process_request(_format_lock_front_buttons(self._matrix_type))

        @synchronized
        def unlock_front_buttons(self):
            # Unlock front panel buttons
            self._process_request(_format_unlock_front_buttons(self._matrix_type))

        @synchronized
        def lock_status(self):
            # Report system locking status
            return LockStatus.from_string(
                self._process_request(_format_lock_status(self._matrix_type))
            )

    return BlackbirdSync(url, matrix_type)


async def get_async_blackbird(port_url, loop):
    """
    Return asynchronous version of Blackbird interface
    :param port_url: serial port, i.e. '/dev/ttyUSB0'
    :return: asynchronous implementation of Blackbird interface
    """

    lock = asyncio.Lock()

    def locked_coro(coro):
        @wraps(coro)
        async def wrapper(*args, **kwargs):
            with await lock:
                return await coro(*args, **kwargs)

        return wrapper

    class BlackbirdAsync(Blackbird):
        def __init__(self, blackbird_protocol):
            self._protocol = blackbird_protocol

        @locked_coro
        async def zone_status(self, zone: int):
            if self._matrix_type == 0:
                string = await self._protocol.send(
                    _format_zone_status_request(self._matrix_type, zone), skip=15
                )
            elif self._matrix_type == 1:
                string = await self._protocol.send(
                    _format_zone_status_request(self._matrix_type, zone), skip=3840
                )

            return ZoneStatus.from_string(self._matrix_type, zone, string)

        @locked_coro
        async def set_zone_power(self, zone: int, power: bool):
            await self._protocol.send(
                _format_set_zone_power(self._matrix_type, zone, power)
            )

        @locked_coro
        async def set_zone_source(self, zone: int, source: int):
            await self._protocol.send(
                _format_set_zone_source(self._matrix_type, zone, source)
            )

        @locked_coro
        async def set_all_zone_source(self, source: int):
            await self._protocol.send(
                _format_set_all_zone_source(self._matrix_type, source)
            )

        @locked_coro
        async def lock_front_buttons(self):
            await self._protocol.send(_format_lock_front_buttons(self._matrix_type))

        @locked_coro
        async def unlock_front_buttons(self):
            await self._protocol.send(_format_unlock_front_buttons(self._matrix_type))

        @locked_coro
        async def lock_status(self):
            string = await self._protocol.send(_format_lock_status(self._matrix_type))
            return LockStatus.from_string(string)

    class BlackbirdProtocol(asyncio.Protocol):
        def __init__(self, loop):
            super().__init__()
            self._loop = loop
            self._lock = asyncio.Lock()
            self._transport = None
            self._connected = asyncio.Event(loop=loop)
            self.q = asyncio.Queue(loop=loop)

        def connection_made(self, transport):
            self._transport = transport
            self._connected.set()
            _LOGGER.debug("port opened %s", self._transport)

        def data_received(self, data):
            asyncio.ensure_future(self.q.put(data), loop=self._loop)

        async def send(self, request: bytes, skip=0):
            await self._connected.wait()
            result = bytearray()
            # Only one transaction at a time
            with await self._lock:
                self._transport.serial.reset_output_buffer()
                self._transport.serial.reset_input_buffer()
                while not self.q.empty():
                    self.q.get_nowait()
                self._transport.write(request)
                try:
                    while True:
                        result += await asyncio.wait_for(
                            self.q.get(), TIMEOUT, loop=self._loop
                        )
                        if len(result) > skip and result[-LEN_EOL:] == EOL:
                            ret = bytes(result)
                            _LOGGER.debug('Received "%s"', ret)
                            return ret.decode("ascii")
                except asyncio.TimeoutError:
                    _LOGGER.error(
                        "Timeout during receiving response for command '%s', received='%s'",
                        request,
                        result,
                    )
                    raise

    _, protocol = await create_serial_connection(
        loop, functools.partial(BlackbirdProtocol, loop), port_url, baudrate=9600
    )

    return BlackbirdAsync(protocol)
