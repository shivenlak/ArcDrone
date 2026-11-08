"""Cross-platform simulated radio bridge for ArcDrone Link layer.

Replaces POSIX pseudo-terminals (openpty/termios) with pure in-memory, thread-safe
virtual serial ports fully compatible with pyserial. Runs on Windows, Linux, and macOS.
"""

from __future__ import annotations

import collections
import random
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

try:
    import serial
    _ORIGINAL_SERIAL = serial.Serial
    SerialException = serial.SerialException
except ImportError:
    serial = None
    _ORIGINAL_SERIAL = None

    class SerialException(IOError):
        """Fallback SerialException when pyserial is not installed."""
        pass

# Global registry for named virtual ports (e.g., "sim://drone", "sim://ground")
_VIRTUAL_REGISTRY: Dict[str, VirtualSerialPort] = {}
_REGISTRY_LOCK = threading.Lock()
_SERIAL_PATCHED = False


class VirtualSerialPort:
    """In-memory virtual serial port conforming to the pyserial Serial interface.

    Connected to a peer VirtualSerialPort or SimLinkBridge.
    """

    def __init__(
        self,
        name: str,
        bridge: Optional[SimLinkBridge] = None,
        timeout: float = 0.05,
        baudrate: int = 115200,
    ) -> None:
        self.name = name
        self.portstr = name
        self.bridge = bridge
        self.timeout = timeout
        self.baudrate = baudrate
        self.is_open = True

        self._rx_buf = bytearray()
        self._cond = threading.Condition()
        self.stats = collections.Counter()

    @property
    def in_waiting(self) -> int:
        with self._cond:
            return len(self._rx_buf)

    def read(self, size: int = 1) -> bytes:
        """Read up to `size` bytes, blocking up to `self.timeout` seconds."""
        if not self.is_open:
            raise SerialException(f"Virtual port '{self.name}' is closed.")
        if size <= 0:
            return b""

        deadline = time.monotonic() + (self.timeout if self.timeout is not None else 0.0)
        with self._cond:
            while len(self._rx_buf) == 0 and self.is_open:
                remaining = deadline - time.monotonic()
                if self.timeout is None:
                    self._cond.wait()
                elif remaining <= 0:
                    break
                else:
                    self._cond.wait(timeout=max(remaining, 0.001))

            if not self.is_open:
                raise SerialException(f"Virtual port '{self.name}' was closed during read.")
            if len(self._rx_buf) == 0:
                return b""

            chunk = bytes(self._rx_buf[:size])
            del self._rx_buf[:size]
            self.stats["bytes_read"] += len(chunk)
            return chunk

    def write(self, data: bytes) -> int:
        """Transmit data through the bridge to the peer port."""
        if not self.is_open:
            raise SerialException(f"Virtual port '{self.name}' is closed.")
        if not data:
            return 0

        self.stats["bytes_written"] += len(data)

        if self.bridge is not None:
            return self.bridge.route_data(self, data)
        return len(data)

    def deliver_bytes(self, data: bytes) -> None:
        """Called by bridge to deliver bytes into this port's RX buffer."""
        if not self.is_open or not data:
            return
        with self._cond:
            self._rx_buf.extend(data)
            self._cond.notify_all()

    def reset_input_buffer(self) -> None:
        """Clear the input RX buffer."""
        with self._cond:
            self._rx_buf.clear()
            self._cond.notify_all()

    def reset_output_buffer(self) -> None:
        """No-op for virtual serial."""
        pass

    def flush(self) -> None:
        """No-op for virtual serial."""
        pass

    def close(self) -> None:
        """Close the virtual port."""
        with self._cond:
            self.is_open = False
            self._cond.notify_all()

    def open(self) -> None:
        """Reopen the virtual port."""
        with self._cond:
            self.is_open = True

    def __repr__(self) -> str:
        return f"<VirtualSerialPort '{self.name}' open={self.is_open} in_waiting={self.in_waiting}>"


class SimLinkBridge:
    """Bidirectional simulated radio link connecting two VirtualSerialPort instances.

    Supports:
    - Dynamic Role Learning: Identifies DRONE vs GROUND automatically from packet types.
    - Dual-Path Frequency Diversity: Simulates simultaneous transmission over ESP-NOW and LoRa.
    - cut() and restore() for deterministic radio blackout simulation.
    - packet_loss_rate (0.0 to 1.0) simulating wireless packet drops.
    - latency (seconds) simulating radio airtime / propagation delay.
    - Wire byte and line transfer statistics.
    """

    def __init__(
        self,
        name_a: str = "sim://drone",
        name_b: str = "sim://ground",
        packet_loss_rate: float = 0.0,
        latency: float = 0.0,
        timeout: float = 0.05,
        dual_path: bool = False,
    ) -> None:
        self.name_a = name_a
        self.name_b = name_b
        self.packet_loss_rate = packet_loss_rate
        self.latency = latency
        self.dual_path = dual_path
        self.radio_mode = "BOTH"  # "BOTH", "LORA_ONLY", "MUTED"
        self._is_cut = False
        self._lock = threading.Lock()

        # Dynamic role learning: learned from observed traffic
        self.roles: Dict[str, str] = {name_a: "UNKNOWN", name_b: "UNKNOWN"}
        self.last_activity: float = float("-inf")

        self.stats = collections.Counter()
        self.tx_hooks: List[Any] = []

        self.port_a = VirtualSerialPort(name=name_a, bridge=self, timeout=timeout)
        self.port_b = VirtualSerialPort(name=name_b, bridge=self, timeout=timeout)

        # Register named ports globally
        with _REGISTRY_LOCK:
            _VIRTUAL_REGISTRY[name_a] = self.port_a
            _VIRTUAL_REGISTRY[name_b] = self.port_b

        patch_serial()

    @property
    def is_cut(self) -> bool:
        with self._lock:
            return self._is_cut or self.radio_mode == "MUTED"

    def cut(self) -> None:
        """Cut the simulated radio link (all radio transmissions are dropped)."""
        with self._lock:
            self._is_cut = True

    def restore(self) -> None:
        """Restore the simulated radio link."""
        with self._lock:
            self._is_cut = False
            self.radio_mode = "BOTH"

    def set_radio_mode(self, mode: str) -> None:
        """Set radio mode: 'BOTH' (ESP-NOW + LoRa), 'LORA_ONLY', or 'MUTED'."""
        with self._lock:
            self.radio_mode = mode.upper()
            if self.radio_mode == "MUTED":
                self._is_cut = True
            else:
                self._is_cut = False

    def get_role(self, port_name: str) -> str:
        """Return dynamically inferred role ('DRONE', 'GROUND', or 'UNKNOWN')."""
        with self._lock:
            return self.roles.get(port_name, "UNKNOWN")

    def _learn_role(self, sender_name: str, receiver_name: str, line: bytes) -> None:
        """Infer DRONE vs GROUND dynamically from packet magic and opcode (plain 0xD2 or secure 0xD3)."""
        try:
            line_str = line.strip().decode("ascii", errors="ignore")
            if len(line_str) >= 4:
                raw = bytes.fromhex(line_str[:8])
                if len(raw) >= 2 and raw[0] in (0xD2, 0xD3):  # 0xD2 = plain, 0xD3 = AES-128-CCM secure
                    ptype = raw[1]
                    if ptype in (0x01, 0x03):  # DETECTION or HEARTBEAT
                        self.roles[sender_name] = "DRONE"
                        self.roles[receiver_name] = "GROUND"
                    elif ptype == 0x04:  # COMMAND
                        self.roles[sender_name] = "GROUND"
                        self.roles[receiver_name] = "DRONE"
        except Exception:
            pass

    def set_packet_loss_rate(self, rate: float) -> None:
        """Update simulated packet loss rate (0.0 - 1.0)."""
        with self._lock:
            self.packet_loss_rate = max(0.0, min(1.0, float(rate)))

    def set_latency(self, latency_s: float) -> None:
        """Update simulated latency in seconds."""
        with self._lock:
            self.latency = max(0.0, float(latency_s))

    def add_tx_hook(self, hook) -> None:
        """Register a callback for every transmission: hook(sender_port, data)."""
        self.tx_hooks.append(hook)

    def route_data(self, sender: VirtualSerialPort, data: bytes) -> int:
        """Route data from sender to receiver, applying cut, loss, and latency."""
        with self._lock:
            cut = self._is_cut
            loss = self.packet_loss_rate
            delay = self.latency

        # Notify hooks (useful for bandwidth monitors)
        for hook in self.tx_hooks:
            try:
                hook(sender, data)
            except Exception:
                pass

        if cut:
            self.stats["dropped_cut"] += 1
            return len(data)

        receiver = self.port_b if sender is self.port_a else self.port_a
        self.last_activity = time.monotonic()

        # In ArcDrone, packets are sent as hex lines ending with \n.
        # We process line-by-line for realistic packet-loss simulation.
        chunks = data.split(b"\n")
        delivered_bytes = bytearray()

        for idx, chunk in enumerate(chunks):
            # Include newline back if not the last segment or if data ended with \n
            is_line = idx < len(chunks) - 1 or (idx == len(chunks) - 1 and data.endswith(b"\n"))
            payload = chunk + (b"\n" if is_line and chunk else b"")
            if not payload:
                continue

            self._learn_role(sender.name, receiver.name, chunk)

            # Dual-path simulation (simultaneous ESP-NOW + LoRa broadcast)
            if self.dual_path and self.radio_mode == "BOTH":
                # Path 1: ESP-NOW (fast 2.4 GHz, baseline loss)
                espnow_delivered = not (loss > 0.0 and random.random() < loss)
                # Path 2: LoRa 915 MHz (narrowband, higher penetration / lower loss)
                lora_delivered = not (loss > 0.0 and random.random() < (loss * 0.6))

                if espnow_delivered:
                    delivered_bytes.extend(payload)
                    self.stats["packets_transferred_espnow"] += 1
                if lora_delivered:
                    # LoRa path delivers with airtime delay
                    if delay > 0 or True:
                        lora_delay = delay + 0.015
                        def lora_delivery(p=payload):
                            time.sleep(lora_delay)
                            if not self.is_cut:
                                receiver.deliver_bytes(bytes(p))
                        threading.Thread(target=lora_delivery, daemon=True).start()
                    self.stats["packets_transferred_lora"] += 1

                if not espnow_delivered and not lora_delivered:
                    self.stats["dropped_loss"] += 1
                    continue
            elif self.dual_path and self.radio_mode == "LORA_ONLY":
                # LoRa only path
                if loss > 0.0 and random.random() < (loss * 0.6):
                    self.stats["dropped_loss"] += 1
                    continue
                delivered_bytes.extend(payload)
                self.stats["packets_transferred_lora"] += 1
            else:
                # Standard single-path transmission
                if loss > 0.0 and random.random() < loss:
                    self.stats["dropped_loss"] += 1
                    continue
                delivered_bytes.extend(payload)
                self.stats["packets_transferred"] += 1

        self.stats["bytes_transferred"] += len(delivered_bytes)

        if delay > 0:
            def delayed_delivery():
                time.sleep(delay)
                receiver.deliver_bytes(bytes(delivered_bytes))
            threading.Thread(target=delayed_delivery, daemon=True).start()
        else:
            receiver.deliver_bytes(bytes(delivered_bytes))

        return len(data)

    def close(self) -> None:
        """Close ports and unregister from global registry."""
        self.port_a.close()
        self.port_b.close()
        with _REGISTRY_LOCK:
            _VIRTUAL_REGISTRY.pop(self.name_a, None)
            _VIRTUAL_REGISTRY.pop(self.name_b, None)

    def __enter__(self) -> Tuple[VirtualSerialPort, VirtualSerialPort]:
        return self.port_a, self.port_b

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


class VirtualSerialPair(SimLinkBridge):
    """Alias for SimLinkBridge matching PROJECT.md interface contract."""
    pass


def create_linked_pair(
    name_a: str = "sim://drone",
    name_b: str = "sim://ground",
    packet_loss_rate: float = 0.0,
    latency: float = 0.0,
    timeout: float = 0.05,
) -> Tuple[VirtualSerialPort, VirtualSerialPort]:
    """Factory creating a connected pair of VirtualSerialPorts registered globally."""
    bridge = SimLinkBridge(
        name_a=name_a,
        name_b=name_b,
        packet_loss_rate=packet_loss_rate,
        latency=latency,
        timeout=timeout,
    )
    return bridge.port_a, bridge.port_b


def patch_serial() -> None:
    """Monkey-patch serial.Serial to transparently intercept VirtualSerialPort and sim:// names."""
    global _SERIAL_PATCHED
    if _SERIAL_PATCHED or serial is None:
        return

    def patched_serial_factory(port: Any, *args, **kwargs) -> Any:
        # If passed an existing VirtualSerialPort instance, return it directly
        if isinstance(port, VirtualSerialPort):
            if "timeout" in kwargs and kwargs["timeout"] is not None:
                port.timeout = kwargs["timeout"]
            if "baudrate" in kwargs and kwargs["baudrate"] is not None:
                port.baudrate = kwargs["baudrate"]
            port.open()
            return port

        # If passed a registered virtual string name, return the port
        if isinstance(port, str):
            with _REGISTRY_LOCK:
                if port in _VIRTUAL_REGISTRY:
                    virtual_port = _VIRTUAL_REGISTRY[port]
                    if "timeout" in kwargs and kwargs["timeout"] is not None:
                        virtual_port.timeout = kwargs["timeout"]
                    if "baudrate" in kwargs and kwargs["baudrate"] is not None:
                        virtual_port.baudrate = kwargs["baudrate"]
                    virtual_port.open()
                    return virtual_port

        # Otherwise delegate to standard serial.Serial
        return _ORIGINAL_SERIAL(port, *args, **kwargs)

    serial.Serial = patched_serial_factory  # type: ignore
    _SERIAL_PATCHED = True


def unpatch_serial() -> None:
    """Restore original serial.Serial."""
    global _SERIAL_PATCHED
    if _SERIAL_PATCHED and serial is not None:
        serial.Serial = _ORIGINAL_SERIAL  # type: ignore
        _SERIAL_PATCHED = False
