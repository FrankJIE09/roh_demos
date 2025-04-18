import asyncio
import struct
from asyncio import Queue
from contextlib import suppress
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Dict, List

import numpy as np
from bleak import (
    BleakScanner,
    BLEDevice,
    AdvertisementData,
    BleakClient,
    BleakGATTCharacteristic,
)

SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
CMD_NOTIFY_CHAR_UUID = "f000ffe1-0451-4000-b000-000000000000"
DATA_NOTIFY_CHAR_UUID = "f000ffe2-0451-4000-b000-000000000000"


@dataclass
class Characteristic:
    uuid: str
    service_uuid: str
    descriptor_uuids: List[str]


class Command(IntEnum):
    GET_PROTOCOL_VERSION = (0x00,)
    GET_FEATURE_MAP = (0x01,)
    GET_DEVICE_NAME = (0x02,)
    GET_MODEL_NUMBER = (0x03,)
    GET_SERIAL_NUMBER = (0x04,)
    GET_HW_REVISION = (0x05,)
    GET_FW_REVISION = (0x06,)
    GET_MANUFACTURER_NAME = (0x07,)
    GET_BOOTLOADER_VERSION = (0x0A,)

    GET_BATTERY_LEVEL = (0x08,)
    GET_TEMPERATURE = (0x09,)

    POWEROFF = (0x1D,)
    SWITCH_TO_OAD = (0x1E,)
    SYSTEM_RESET = (0x1F,)
    SWITCH_SERVICE = (0x20,)

    SET_LOG_LEVEL = (0x21,)
    SET_LOG_MODULE = (0x22,)
    PRINT_KERNEL_MSG = (0x23,)
    MOTOR_CONTROL = (0x24,)
    LED_CONTROL_TEST = (0x25,)
    PACKAGE_ID_CONTROL = (0x26,)
    SEND_TRAINING_PACKAGE = (0x27,)

    GET_ACCELERATE_CAP = (0x30,)
    SET_ACCELERATE_CONFIG = (0x31,)

    GET_GYROSCOPE_CAP = (0x32,)
    SET_GYROSCOPE_CONFIG = (0x33,)

    GET_MAGNETOMETER_CAP = (0x34,)
    SET_MAGNETOMETER_CONFIG = (0x35,)

    GET_EULER_ANGLE_CAP = (0x36,)
    SET_EULER_ANGLE_CONFIG = (0x37,)

    QUATERNION_CAP = (0x38,)
    QUATERNION_CONFIG = (0x39,)

    GET_ROTATION_MATRIX_CAP = (0x3A,)
    SET_ROTATION_MATRIX_CONFIG = (0x3B,)

    GET_GESTURE_CAP = (0x3C,)
    SET_GESTURE_CONFIG = (0x3D,)

    GET_EMG_RAWDATA_CAP = (0x3E,)
    SET_EMG_RAWDATA_CONFIG = (0x3F,)

    GET_MOUSE_DATA_CAP = (0x40,)
    SET_MOUSE_DATA_CONFIG = (0x41,)

    GET_JOYSTICK_DATA_CAP = (0x42,)
    SET_JOYSTICK_DATA_CONFIG = (0x43,)

    GET_DEVICE_STATUS_CAP = (0x44,)
    SET_DEVICE_STATUS_CONFIG = (0x45,)

    GET_EMG_RAWDATA_CONFIG = (0x46,)

    SET_DATA_NOTIF_SWITCH = (0x4F,)
    # Partial command packet, format: [CMD_PARTIAL_DATA, packet number in reverse order, packet content]
    MD_PARTIAL_DATA = 0xFF


class DataSubscription(IntEnum):
    # Data Notify All Off
    OFF = (0x00000000,)

    # Accelerate On(C.7)
    ACCELERATE = (0x00000001,)

    # Gyroscope On(C.8)
    GYROSCOPE = (0x00000002,)

    # Magnetometer On(C.9)
    MAGNETOMETER = (0x00000004,)

    # Euler Angle On(C.10)
    EULERANGLE = (0x00000008,)

    # Quaternion On(C.11)
    QUATERNION = (0x00000010,)

    # Rotation Matrix On(C.12)
    ROTATIONMATRIX = (0x00000020,)

    # EMG Gesture On(C.13)
    EMG_GESTURE = (0x00000040,)

    # EMG Raw Data On(C.14)
    EMG_RAW = (0x00000080,)

    # HID Mouse On(C.15)
    HID_MOUSE = (0x00000100,)

    # HID Joystick On(C.16)
    HID_JOYSTICK = (0x00000200,)

    # Device Status On(C.17)
    DEVICE_STATUS = (0x00000400,)

    # Device Log On
    LOG = (0x00000800,)

    # Data Notify All On
    ALL = 0xFFFFFFFF


class DataType(IntEnum):
    ACC = (0x01,)
    GYO = (0x02,)
    MAG = (0x03,)
    EULER = (0x04,)
    QUAT = (0x05,)
    ROTA = (0x06,)
    EMG_GEST = (0x07,)
    EMG_ADC = (0x08,)
    HID_MOUSE = (0x09,)
    HID_JOYSTICK = (0x0A,)
    DEV_STATUS = (0x0B,)
    LOG = (0x0C,)

    PARTIAL = 0xFF


class SampleResolution(IntEnum):
    BITS_8 = (8,)
    BITS_12 = 12


class SamplingRate(IntEnum):
    HZ_500 = (500,)
    HZ_650 = (650,)


@dataclass
class EmgRawDataConfig:
    fs: SamplingRate = SamplingRate.HZ_500
    channel_mask: int = 0xFF
    batch_len: int = 16
    resolution: SampleResolution = SampleResolution.BITS_8

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<H", self.fs)
        body += struct.pack("<H", self.channel_mask)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_mask, batch_len, resolution = struct.unpack(
            "@HHBB",
            data,
        )
        return cls(fs, channel_mask, batch_len, resolution)


@dataclass
class Request:
    cmd: Command
    has_res: bool
    body: Optional[bytes] = None


class ResponseCode(IntEnum):
    SUCCESS = (0x00,)
    NOT_SUPPORT = (0x01,)
    BAD_PARAM = (0x02,)
    FAILED = (0x03,)
    TIMEOUT = (0x04,)
    PARTIAL_PACKET = 0xFF


@dataclass
class Response:
    code: ResponseCode
    cmd: Command
    data: bytes


class GForce:
    def __init__(self, device_name_prefix="", min_rssi=-128):
        self.device_name = ""
        self.client = None
        self.cmd_char = None
        self.data_char = None
        self.responses: Dict[Command, Queue] = {}
        self.resolution = SampleResolution.BITS_8
        self._num_channels = 8
        self._device_name_prefix = device_name_prefix
        self._min_rssi = min_rssi

        self.packet_id = 0
        self.data_packet = []

    def _match_device(self, _device: BLEDevice, adv: AdvertisementData):
        if (
            SERVICE_GUID.lower() in adv.service_uuids
            and _device.name != None
            and _device.name.startswith(self._device_name_prefix)
            and adv.rssi >= self._min_rssi
        ):
            print("Device found: {0}, RSSI: {1}".format(_device.name, adv.rssi))
            return True

        return False

    async def connect(self):
        device = await BleakScanner.find_device_by_filter(self._match_device)
        if device is None:
            raise Exception("No GForce device found")

        def handle_disconnect(_: BleakClient):
            for task in asyncio.all_tasks():
                task.cancel()

        client = BleakClient(device, disconnected_callback=handle_disconnect)
        await client.connect()

        self.client = client
        self.device_name = device.name

        await client.start_notify(
            CMD_NOTIFY_CHAR_UUID,
            self._on_cmd_response,
        )

    def _on_data_response(self, q: Queue, bs: bytearray):
        bs = bytes(bs)
        full_packet = []

        is_partial_data = bs[0] == ResponseCode.PARTIAL_PACKET
        if is_partial_data:
            packet_id = bs[1]
            if self.packet_id != 0 and self.packet_id != packet_id + 1:
                raise Exception(
                    "Unexpected packet id: expected {} got {}".format(
                        self.packet_id + 1,
                        packet_id,
                    )
                )
            elif self.packet_id == 0 or self.packet_id > packet_id:
                self.packet_id = packet_id
                self.data_packet += bs[2:]

                if self.packet_id == 0:
                    full_packet = self.data_packet
                    self.data_packet = []
        else:
            full_packet = bs

        if len(full_packet) == 0:
            return

        data = None
        data_type = DataType(full_packet[0])
        packet = full_packet[1:]
        match data_type:
            case DataType.EMG_ADC:
                data = self._convert_emg_to_raw(packet)

            case DataType.ACC:
                data = self._convert_acceleration_to_g(packet)

            case DataType.GYO:
                data = self._convert_gyro_to_dps(packet)

            case DataType.MAG:
                data = self._convert_magnetometer_to_ut(packet)

            case DataType.EULER:
                data = self._convert_euler(packet)

            case DataType.QUAT:
                data = self._convert_quaternion(packet)

            case DataType.ROTA:
                data = self._convert_rotation_matrix(packet)

            case DataType.EMG_GEST:  # It is not supported by the device (?)
                data = self._convert_emg_gesture(packet)

            case DataType.HID_MOUSE:  # It is not supported by the device
                pass

            case DataType.HID_JOYSTICK:  # It is not supported by the device
                pass

            case DataType.PARTIAL:
                pass
            case _:
                raise Exception(
                    f"Unknown data type {data_type}, full packet: {full_packet}"
                )

        q.put_nowait(data)

    def _convert_emg_to_raw(self, data: bytes) -> np.ndarray[np.integer]:
        match self.resolution:
            case SampleResolution.BITS_8:
                dtype = np.uint8

            case SampleResolution.BITS_12:
                dtype = np.uint16

            case _:
                raise Exception(f"Unsupported resolution {self.resolution}")

        emg_data = np.frombuffer(data, dtype=dtype)

        return emg_data.reshape(-1, self._num_channels)

    @staticmethod
    def _convert_acceleration_to_g(data: bytes) -> np.ndarray[np.float32]:
        normalizing_factor = 65536.0

        acceleration_data = (
            np.frombuffer(data, dtype=np.int32).astype(np.float32) / normalizing_factor
        )
        num_channels = 3

        return acceleration_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_gyro_to_dps(data: bytes) -> np.ndarray[np.float32]:
        normalizing_factor = 65536.0

        gyro_data = (
            np.frombuffer(data, dtype=np.int32).astype(np.float32) / normalizing_factor
        )
        num_channels = 3

        return gyro_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_magnetometer_to_ut(data: bytes) -> np.ndarray[np.float32]:
        normalizing_factor = 65536.0

        magnetometer_data = (
            np.frombuffer(data, dtype=np.int32).astype(np.float32) / normalizing_factor
        )
        num_channels = 3

        return magnetometer_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_euler(data: bytes) -> np.ndarray[np.float32]:

        euler_data = np.frombuffer(data, dtype=np.float32).astype(np.float32)
        num_channels = 3

        return euler_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_quaternion(data: bytes) -> np.ndarray[np.float32]:

        quaternion_data = np.frombuffer(data, dtype=np.float32).astype(np.float32)
        num_channels = 4

        return quaternion_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_rotation_matrix(data: bytes) -> np.ndarray[np.float32]:

        rotation_matrix_data = np.frombuffer(data, dtype=np.int32).astype(np.float32)
        num_channels = 9

        return rotation_matrix_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_emg_gesture(data: bytes) -> np.ndarray[np.float16]:

        emg_gesture_data = np.frombuffer(data, dtype=np.int16).astype(np.float16)
        num_channels = 6

        return emg_gesture_data.reshape(-1, num_channels)

    def _on_cmd_response(self, _: BleakGATTCharacteristic, bs: bytearray):
        try:
            response = self._parse_response(bytes(bs))
            if response.cmd in self.responses:
                self.responses[response.cmd].put_nowait(
                    response.data,
                )
        except Exception as e:
            raise Exception("Failed to parse response: %s" % e)

    @staticmethod
    def _parse_response(res: bytes) -> Response:
        code = int.from_bytes(res[:1], byteorder="big")
        code = ResponseCode(code)

        cmd = int.from_bytes(res[1:2], byteorder="big")
        cmd = Command(cmd)

        data = res[2:]

        return Response(
            code=code,
            cmd=cmd,
            data=data,
        )

    async def get_protocol_version(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_PROTOCOL_VERSION,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_feature_map(self) -> int:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_FEATURE_MAP,
                has_res=True,
            )
        )
        return int.from_bytes(buf, byteorder="big")  # TODO: check if this is correct

    async def get_device_name(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_DEVICE_NAME,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_firmware_revision(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_FW_REVISION,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_hardware_revision(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_HW_REVISION,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_model_number(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_MODEL_NUMBER,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_serial_number(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_SERIAL_NUMBER,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_manufacturer_name(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_MANUFACTURER_NAME,
                has_res=True,
            )
        )

        return buf.decode("utf-8")

    async def get_bootloader_version(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_BOOTLOADER_VERSION,
                has_res=True,
            )
        )

        return buf.decode("utf-8")

    async def get_battery_level(self) -> int:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_BATTERY_LEVEL,
                has_res=True,
            )
        )
        return int.from_bytes(buf, byteorder="big")

    async def get_temperature(self) -> int:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_TEMPERATURE,
                has_res=True,
            )
        )
        return int.from_bytes(buf, byteorder="big")

    async def power_off(self) -> None:
        await self._send_request(
            Request(
                cmd=Command.POWEROFF,
                has_res=False,
            )
        )

    async def switch_to_oad(self):
        ret = await self._send_request(
            Request(
                cmd=Command.SWITCH_TO_OAD,
                has_res=False,
            )
        )

    async def system_reset(self):
        ret = await self._send_request(
            Request(
                cmd=Command.SYSTEM_RESET,
                has_res=False,
            )
        )

    async def switch_service(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.SWITCH_SERVICE,
                has_res=False,
            )
        )

    async def set_motor(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.MOTOR_CONTROL,
                has_res=True,
            )
        )

    async def set_led(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.LED_CONTROL_TEST,
                has_res=True,
            )
        )

    async def set_log_level(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.SET_LOG_LEVEL,
                has_res=False,
            )
        )

    async def set_log_module(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.SET_LOG_MODULE,
                has_res=False,
            )
        )

    async def print_kernel_msg(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.PRINT_KERNEL_MSG,
                has_res=True,
            )
        )

    async def set_package_id(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.PACKAGE_ID_CONTROL,
                has_res=False,
            )
        )

    async def send_training_package(self):  # TODO: parameter data
        ret = await self._send_request(
            Request(
                cmd=Command.SEND_TRAINING_PACKAGE,
                has_res=False,
            )
        )

    async def set_emg_raw_data_config(self, cfg=EmgRawDataConfig()):
        body = cfg.to_bytes()
        ret = await self._send_request(
            Request(
                cmd=Command.SET_EMG_RAWDATA_CONFIG,
                body=body,
                has_res=True,
            )
        )

        print('_send_request returned:', ret)

        self.resolution = cfg.resolution

        num_channels = 0
        ch_mask = cfg.channel_mask

        while ch_mask != 0:
            if ch_mask & 0x01 != 0:
                num_channels += 1
            ch_mask >>= 1

        self.__num_channels = num_channels

    async def get_emg_raw_data_config(self) -> EmgRawDataConfig:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_EMG_RAWDATA_CONFIG,
                has_res=True,
            )
        )
        return EmgRawDataConfig.from_bytes(buf)

    async def set_subscription(self, subscription: DataSubscription):
        body = [
            0xFF & subscription,
            0xFF & (subscription >> 8),
            0xFF & (subscription >> 16),
            0xFF & (subscription >> 24),
        ]
        body = bytes(body)
        await self._send_request(
            Request(
                cmd=Command.SET_DATA_NOTIF_SWITCH,
                body=body,
                has_res=True,
            )
        )

    async def start_streaming(self) -> Queue:
        q = Queue()
        await self.client.start_notify(
            DATA_NOTIFY_CHAR_UUID,
            lambda _, data: self._on_data_response(q, data),
        )
        return q

    async def stop_streaming(self):
        exceptions = []
        try:
            await self.set_subscription(DataSubscription.OFF)
        except Exception as e:
            exceptions.append(e)
        try:
            await self.client.stop_notify(DATA_NOTIFY_CHAR_UUID)
        except Exception as e:
            exceptions.append(e)
        try:
            await self.client.stop_notify(CMD_NOTIFY_CHAR_UUID)
        except Exception as e:
            exceptions.append(e)

        if len(exceptions) > 0:
            raise Exception("Failed to stop streaming: %s" % exceptions)

    async def disconnect(self):
        with suppress(asyncio.CancelledError):
            await self.client.disconnect()

    def _get_response_channel(self, cmd: Command) -> Queue:
        q = Queue()
        self.responses[cmd] = q
        return q

    async def _send_request(self, req: Request) -> Optional[bytes]:
        q = None
        if req.has_res:
            q = self._get_response_channel(req.cmd)

        bs = bytes([req.cmd])
        if req.body is not None:
            bs += req.body
        await self.client.write_gatt_char(CMD_NOTIFY_CHAR_UUID, bs)

        if not req.has_res:
            return None

        return await asyncio.wait_for(q.get(), 3)
