"""
CANopen通信接口封装类
提供SDO读写、NMT控制等基本CANopen通信功能
"""

import struct
import time
import threading
from enum import IntEnum
from typing import Optional, Tuple


class OperationMode(IntEnum):
    """工作模式枚举"""
    NO_MODE = 0
    PROFILE_POSITION = 1
    PROFILE_VELOCITY = 3
    PROFILE_TORQUE = 4


class ControlWord:
    """控制字定义"""
    SHUTDOWN = 0x06
    SWITCH_ON = 0x07
    DISABLE_VOLTAGE = 0x00
    QUICK_STOP = 0x02
    DISABLE_OPERATION = 0x07
    ENABLE_OPERATION = 0x0F
    FAULT_RESET = 0x80
    
    # 位置模式控制字
    ABSOLUTE_POSITION_START = 0x1F  # 0x0F -> 0x1F
    RELATIVE_POSITION_START = 0x5F  # 0x4F -> 0x5F
    
    # 转矩模式控制字
    TORQUE_START = 0x1F  # 0x0F -> 0x1F


class ObjectDictionary:
    """对象字典索引定义"""
    # 通信参数
    ERROR_REGISTER = 0x1001
    HEARTBEAT_PRODUCER = 0x1017
    SAVE_ALL_PARAMS = 0x2010
    
    # 控制参数
    CONTROL_WORD = 0x6040
    STATUS_WORD = 0x6041
    MODE_OF_OPERATION = 0x6060
    MODE_DISPLAY = 0x6061
    FAULT_CODE = 0x603F
    
    # 位置模式参数
    TARGET_POSITION = 0x607A
    ACTUAL_POSITION = 0x6064
    PROFILE_VELOCITY = 0x6081
    PROFILE_ACCELERATION = 0x6083
    PROFILE_DECELERATION = 0x6084
    
    # 速度模式参数
    TARGET_VELOCITY = 0x60FF
    ACTUAL_VELOCITY = 0x606C
    
    # 转矩模式参数
    TARGET_TORQUE = 0x6071
    ACTUAL_TORQUE = 0x6077
    TORQUE_SLOPE = 0x6087
    
    # 自定义参数
    SYNC_ASYNC_FLAG = 0x200F


class CANopenInterface:
    """
    CANopen通信接口基类
    需要根据实际使用的CAN库（如python-can, canopen等）实现具体方法
    """
    
    def __init__(self, node_id: int = 1, can_interface: str = "can0"):
        """
        初始化CANopen接口
        
        Args:
            node_id: 节点ID (1-127)
            can_interface: CAN接口名称
        """
        self.node_id = node_id
        self.can_interface = can_interface
        self.tx_sdo_cobid = 0x600 + node_id
        self.rx_sdo_cobid = 0x580 + node_id
        self.heartbeat_cobid = 0x700 + node_id
        
        # 状态
        self.is_connected = False
        self.heartbeat_received = False
        
    def connect(self) -> bool:
        """
        连接CAN总线
        
        Returns:
            连接是否成功
        """
        # 需要根据实际CAN库实现
        # 例如使用python-can:
        # import can
        # self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
        raise NotImplementedError("需要在子类中实现connect方法")
    
    def disconnect(self):
        """断开CAN总线连接"""
        # 需要根据实际CAN库实现
        raise NotImplementedError("需要在子类中实现disconnect方法")
    
    def send_nmt(self, command: int, node_id: Optional[int] = None):
        """
        发送NMT命令
        
        Args:
            command: NMT命令 (0x01=启动, 0x02=停止, 0x80=预操作, 0x81=复位应用层, 0x82=复位通信)
            node_id: 目标节点ID，None表示广播(0)
        """
        target_id = node_id if node_id is not None else 0
        data = bytes([command, target_id])
        # 需要实现CAN帧发送
        raise NotImplementedError("需要在子类中实现send_nmt方法")
    
    def sdo_write(self, index: int, subindex: int, data: bytes, data_type: str = "auto") -> bool:
        """
        通过SDO写入对象字典
        
        Args:
            index: 对象字典索引
            subindex: 子索引
            data: 数据字节
            data_type: 数据类型 ("auto", "u8", "i8", "u16", "i16", "u32", "i32")
            
        Returns:
            写入是否成功
        """
        # 根据数据长度选择SDO命令字
        data_len = len(data)
        if data_type == "auto":
            if data_len == 1:
                cmd = 0x2F  # 1字节
            elif data_len == 2:
                cmd = 0x2B  # 2字节
            elif data_len == 3:
                cmd = 0x27  # 3字节
            elif data_len == 4:
                cmd = 0x23  # 4字节
            else:
                raise ValueError(f"不支持的数据长度: {data_len}")
        else:
            # 根据data_type确定命令字
            type_map = {"u8": 0x2F, "i8": 0x2F, "u16": 0x2B, "i16": 0x2B,
                       "u32": 0x23, "i32": 0x23}
            cmd = type_map.get(data_type, 0x23)
        
        # 构造SDO请求帧
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + data
        sdo_data = sdo_data + bytes(8 - len(sdo_data))  # 填充到8字节
        
        # 发送SDO请求并等待响应
        # 需要实现CAN帧发送和接收
        raise NotImplementedError("需要在子类中实现sdo_write方法")
    
    def sdo_read(self, index: int, subindex: int, data_type: str = "u32") -> Optional[bytes]:
        """
        通过SDO读取对象字典
        
        Args:
            index: 对象字典索引
            subindex: 子索引
            data_type: 数据类型 ("u8", "i8", "u16", "i16", "u32", "i32")
            
        Returns:
            读取的数据，失败返回None
        """
        cmd = 0x40  # 读取命令
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + bytes(4)
        
        # 发送SDO请求并等待响应
        # 解析响应帧，提取数据
        raise NotImplementedError("需要在子类中实现sdo_read方法")
    
    def write_u8(self, index: int, subindex: int, value: int) -> bool:
        """写入8位无符号整数"""
        return self.sdo_write(index, subindex, struct.pack('<B', value), "u8")
    
    def write_u16(self, index: int, subindex: int, value: int) -> bool:
        """写入16位无符号整数"""
        return self.sdo_write(index, subindex, struct.pack('<H', value), "u16")
    
    def write_i16(self, index: int, subindex: int, value: int) -> bool:
        """写入16位有符号整数"""
        return self.sdo_write(index, subindex, struct.pack('<h', value), "i16")
    
    def write_u32(self, index: int, subindex: int, value: int) -> bool:
        """写入32位无符号整数"""
        return self.sdo_write(index, subindex, struct.pack('<I', value), "u32")
    
    def write_i32(self, index: int, subindex: int, value: int) -> bool:
        """写入32位有符号整数"""
        return self.sdo_write(index, subindex, struct.pack('<i', value), "i32")
    
    def read_u8(self, index: int, subindex: int) -> Optional[int]:
        """读取8位无符号整数"""
        data = self.sdo_read(index, subindex, "u8")
        return struct.unpack('<B', data)[0] if data and len(data) >= 1 else None
    
    def read_u16(self, index: int, subindex: int) -> Optional[int]:
        """读取16位无符号整数"""
        data = self.sdo_read(index, subindex, "u16")
        return struct.unpack('<H', data[:2])[0] if data and len(data) >= 2 else None
    
    def read_i16(self, index: int, subindex: int) -> Optional[int]:
        """读取16位有符号整数"""
        data = self.sdo_read(index, subindex, "i16")
        return struct.unpack('<h', data[:2])[0] if data and len(data) >= 2 else None
    
    def read_u32(self, index: int, subindex: int) -> Optional[int]:
        """读取32位无符号整数"""
        data = self.sdo_read(index, subindex, "u32")
        return struct.unpack('<I', data[:4])[0] if data and len(data) >= 4 else None
    
    def read_i32(self, index: int, subindex: int) -> Optional[int]:
        """读取32位有符号整数"""
        data = self.sdo_read(index, subindex, "i32")
        return struct.unpack('<i', data[:4])[0] if data and len(data) >= 4 else None


class MockCANopenInterface(CANopenInterface):
    """
    CANopen接口模拟实现，用于测试
    不进行实际的CAN通信，仅模拟响应
    """
    
    def __init__(self, node_id: int = 1, can_interface: str = "can0"):
        super().__init__(node_id, can_interface)
        self.mock_registers = {}
        self.is_connected = True
        # 初始化状态字：左右电机都处于SWITCH_ON_DISABLED状态 (0x0040)
        # 状态字格式：高16位为右电机，低16位为左电机
        # 0x0040 = bit6=1 (SWITCH_ON_DISABLED)
        initial_status = 0x00400040  # 左右电机都是0x0040
        self.mock_registers[(ObjectDictionary.STATUS_WORD, 0)] = struct.pack('<I', initial_status)
        
    def connect(self) -> bool:
        self.is_connected = True
        return True
    
    def disconnect(self):
        self.is_connected = False
    
    def send_nmt(self, command: int, node_id: Optional[int] = None):
        # 模拟NMT命令：启动命令后，状态变为READY_TO_SWITCH_ON
        if command == 0x01:  # NMT启动命令
            # 状态变为READY_TO_SWITCH_ON (0x0021: bit5=1, bit0=1)
            status = 0x00210021  # 左右电机都是0x0021
            self.mock_registers[(ObjectDictionary.STATUS_WORD, 0)] = struct.pack('<I', status)
    
    def sdo_write(self, index: int, subindex: int, data: bytes, data_type: str = "auto") -> bool:
        key = (index, subindex)
        self.mock_registers[key] = data
        
        # 如果写入控制字，根据控制字更新状态字
        if index == ObjectDictionary.CONTROL_WORD and subindex == 0:
            control_word = struct.unpack('<H', data[:2])[0] if len(data) >= 2 else 0
            
            # 获取当前状态字
            current_status_data = self.mock_registers.get((ObjectDictionary.STATUS_WORD, 0), struct.pack('<I', 0x00400040))
            current_status = struct.unpack('<I', current_status_data)[0]
            left_status = current_status & 0xFFFF
            right_status = (current_status >> 16) & 0xFFFF
            
            # 根据控制字更新状态（简化状态机）
            # 0x06 (SHUTDOWN) -> READY_TO_SWITCH_ON (0x0021)
            # 0x07 (SWITCH_ON) -> SWITCHED_ON (0x0023)
            # 0x0F (ENABLE_OPERATION) -> OPERATION_ENABLED (0x0027)
            if control_word == 0x06:  # SHUTDOWN
                new_status = 0x00210021  # READY_TO_SWITCH_ON
            elif control_word == 0x07:  # SWITCH_ON
                new_status = 0x00230023  # SWITCHED_ON
            elif control_word == 0x0F:  # ENABLE_OPERATION
                new_status = 0x00270027  # OPERATION_ENABLED
            elif control_word == 0x00:  # DISABLE_VOLTAGE
                new_status = 0x00400040  # SWITCH_ON_DISABLED
            elif control_word == 0x1F:  # ABSOLUTE_POSITION_START or TORQUE_START
                # 保持OPERATION_ENABLED状态
                new_status = 0x00270027
            elif control_word == 0x5F:  # RELATIVE_POSITION_START
                # 保持OPERATION_ENABLED状态
                new_status = 0x00270027
            else:
                # 其他控制字，保持当前状态
                new_status = current_status
            
            self.mock_registers[(ObjectDictionary.STATUS_WORD, 0)] = struct.pack('<I', new_status)
        
        return True
    
    def sdo_read(self, index: int, subindex: int, data_type: str = "u32") -> Optional[bytes]:
        key = (index, subindex)
        if key in self.mock_registers:
            data = self.mock_registers[key]
            # 填充到4字节
            return data + bytes(4 - len(data))
        
        # 对于状态字，返回默认的SWITCH_ON_DISABLED状态
        if index == ObjectDictionary.STATUS_WORD and subindex == 0:
            return struct.pack('<I', 0x00400040)
        
        # 其他寄存器返回默认值
        return bytes([0, 0, 0, 0])


class SocketCANopenInterface(CANopenInterface):
    """
    基于python-can的SocketCAN接口实现
    用于与真实硬件通信
    """
    
    def __init__(self, node_id: int = 1, can_interface: str = "can0"):
        super().__init__(node_id, can_interface)
        self.bus = None
        self.sdo_timeout = 1.0  # SDO超时时间（秒）
        self.response_event = threading.Event()
        self.response_data = None
        
    def connect(self) -> bool:
        """连接CAN总线"""
        try:
            import can
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=500000
            )
            self.is_connected = True
            
            # 启动接收线程
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            return True
        except ImportError:
            raise ImportError("需要安装python-can库: pip install python-can")
        except Exception as e:
            print(f"连接CAN总线失败: {e}")
            return False
    
    def disconnect(self):
        """断开CAN总线连接"""
        self.is_connected = False
        if self.bus:
            self.bus.shutdown()
            self.bus = None
    
    def _receive_loop(self):
        """接收CAN消息的循环"""
        while self.is_connected and self.bus:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                
                # 处理SDO响应
                if msg.arbitration_id == self.rx_sdo_cobid:
                    self.response_data = msg.data
                    self.response_event.set()
            except Exception as e:
                if self.is_connected:
                    print(f"接收CAN消息错误: {e}")
    
    def send_nmt(self, command: int, node_id: Optional[int] = None):
        """发送NMT命令"""
        if not self.bus:
            return
        
        try:
            import can
            target_id = node_id if node_id is not None else 0
            data = bytes([command, target_id])
            
            msg = can.Message(
                arbitration_id=0x000,  # NMT COB-ID
                data=data,
                is_extended_id=False
            )
            
            self.bus.send(msg)
        except Exception as e:
            print(f"发送NMT命令失败: {e}")
    
    def sdo_write(self, index: int, subindex: int, data: bytes, data_type: str = "auto") -> bool:
        """通过SDO写入对象字典"""
        if not self.bus:
            return False
        
        # 根据数据长度选择SDO命令字
        data_len = len(data)
        if data_type == "auto":
            if data_len == 1:
                cmd = 0x2F  # 1字节
            elif data_len == 2:
                cmd = 0x2B  # 2字节
            elif data_len == 3:
                cmd = 0x27  # 3字节
            elif data_len == 4:
                cmd = 0x23  # 4字节
            else:
                raise ValueError(f"不支持的数据长度: {data_len}")
        else:
            type_map = {"u8": 0x2F, "i8": 0x2F, "u16": 0x2B, "i16": 0x2B,
                       "u32": 0x23, "i32": 0x23}
            cmd = type_map.get(data_type, 0x23)
        
        # 构造SDO请求帧
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + data
        sdo_data = sdo_data + bytes(8 - len(sdo_data))  # 填充到8字节
        
        # 发送SDO请求
        try:
            import can
            msg = can.Message(
                arbitration_id=self.tx_sdo_cobid,
                data=sdo_data,
                is_extended_id=False
            )
            
            self.response_event.clear()
            self.response_data = None
            self.bus.send(msg)
            
            # 等待响应
            if self.response_event.wait(timeout=self.sdo_timeout):
                # 检查响应是否成功
                if self.response_data and len(self.response_data) >= 1:
                    response_cmd = self.response_data[0]
                    # 0x60表示成功写入
                    if response_cmd == 0x60:
                        return True
                    # 0x80表示错误
                    elif (response_cmd & 0xE0) == 0x80:
                        error_code = struct.unpack('<I', self.response_data[4:8])[0]
                        print(f"SDO写入错误: index=0x{index:04X}, subindex=0x{subindex:02X}, error=0x{error_code:08X}")
                        return False
            
            # 超时或响应错误
            return False
        except Exception as e:
            print(f"发送SDO写入请求失败: {e}")
            return False
    
    def sdo_read(self, index: int, subindex: int, data_type: str = "u32") -> Optional[bytes]:
        """通过SDO读取对象字典"""
        if not self.bus:
            return None
        
        cmd = 0x40  # 读取命令
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + bytes(4)
        
        # 发送SDO请求
        try:
            import can
            msg = can.Message(
                arbitration_id=self.tx_sdo_cobid,
                data=sdo_data,
                is_extended_id=False
            )
            
            self.response_event.clear()
            self.response_data = None
            self.bus.send(msg)
            
            # 等待响应
            if self.response_event.wait(timeout=self.sdo_timeout):
                if self.response_data and len(self.response_data) >= 1:
                    response_cmd = self.response_data[0]
                    # 0x4B/0x4F/0x43/0x47表示成功读取
                    if (response_cmd & 0xE0) == 0x40:
                        # 提取数据（根据命令字确定数据位置）
                        if response_cmd == 0x4F:  # 1字节
                            data = self.response_data[4:5]
                        elif response_cmd == 0x4B:  # 2字节
                            data = self.response_data[4:6]
                        elif response_cmd == 0x47:  # 3字节
                            data = self.response_data[4:7]
                        elif response_cmd == 0x43:  # 4字节
                            data = self.response_data[4:8]
                        else:
                            return None
                        return data
                    # 0x80表示错误
                    elif (response_cmd & 0xE0) == 0x80:
                        error_code = struct.unpack('<I', self.response_data[4:8])[0]
                        print(f"SDO读取错误: index=0x{index:04X}, subindex=0x{subindex:02X}, error=0x{error_code:08X}")
                        return None
            
            # 超时或响应错误
            return None
        except Exception as e:
            print(f"发送SDO读取请求失败: {e}")
            return None
