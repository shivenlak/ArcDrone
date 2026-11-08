"""Drone-side delivery verifier: proves each unique seq is confirmed exactly once.

This does NOT change the wire protocol. It only observes the Link lifecycle
(send -> buffered during link loss -> ACK 'delivered') so the HUD and the
end-of-run report can show that every message was received by the operator, and
that buffered messages flushed after the link came back.
"""
from __future__ import annotations

import threading
import time
from collections import OrderedDict
from typing import Any, Dict, List, Optional

# Lifecycle states for one reliable message (keyed by its unique seq).
PENDING = "PENDING"      # sent/queued, no ACK yet (in flight or buffered)
DELIVERED = "DELIVERED"  # operator ACKed it


class DeliveryVerifier:
    """Thread-safe tracker of per-seq delivery state driven by Link callbacks."""

    def __init__(self, keep: int = 200) -> None:
        self._lock = threading.Lock()
        self._msgs: "OrderedDict[int, Dict[str, Any]]" = OrderedDict()
        self._keep = keep
        self.sent = 0
        self.delivered = 0
        self.flushing = False
        self.link_up = True

    def on_send(self, seq: int, kind: str, label: str, buffered: bool) -> None:
        with self._lock:
            self._msgs[seq] = {
                "seq": seq,
                "kind": kind,              # "DETECTION" / "COMMAND"
                "label": label,
                "state": PENDING,
                "buffered": buffered,      # was the link down / queue non-empty at send time
                "t_send": time.monotonic(),
                "rtt_ms": None,
            }
            self.sent += 1
            while len(self._msgs) > self._keep:
                self._msgs.popitem(last=False)

    def on_delivered(self, seq: int, rtt_ms: Optional[float]) -> None:
        with self._lock:
            msg = self._msgs.get(seq)
            if msg is None:
                # Delivered without a recorded send (e.g. operator-side); track minimally.
                msg = {
                    "seq": seq, "kind": "?", "label": "", "state": PENDING,
                    "buffered": False, "t_send": time.monotonic(), "rtt_ms": None,
                }
                self._msgs[seq] = msg
            if msg["state"] != DELIVERED:
                self.delivered += 1
            msg["state"] = DELIVERED
            msg["rtt_ms"] = rtt_ms
            if self.flushing:
                still = sum(1 for m in self._msgs.values() if m["state"] == PENDING)
                if still == 0:
                    self.flushing = False

    def mark_buffered(self, seq: int) -> None:
        with self._lock:
            msg = self._msgs.get(seq)
            if msg and msg["state"] == PENDING:
                msg["buffered"] = True

    def on_link_down(self) -> None:
        with self._lock:
            self.link_up = False
            self.flushing = False
            for msg in self._msgs.values():
                if msg["state"] == PENDING:
                    msg["buffered"] = True

    def on_link_up(self) -> None:
        with self._lock:
            self.link_up = True
            pending = sum(1 for m in self._msgs.values() if m["state"] == PENDING)
            self.flushing = pending > 0

    def pending_count(self) -> int:
        with self._lock:
            return sum(1 for m in self._msgs.values() if m["state"] == PENDING)

    def recent(self, n: int = 5) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._msgs.values())[-n:][::-1]

    def hud_line(self) -> str:
        with self._lock:
            pend = sum(1 for m in self._msgs.values() if m["state"] == PENDING)
            if self.flushing and pend:
                extra = " | FLUSHING %d (oldest first)" % pend
            elif not self.link_up and pend:
                extra = " | QUEUED %d (waiting for range)" % pend
            else:
                extra = ""
            return "VERIFIER: sent %d | ACKed %d | pending %d%s" % (
                self.sent, self.delivered, pend, extra
            )

    def report(self, link_summary: Optional[Dict[str, Any]] = None) -> str:
        with self._lock:
            pend = [m for m in self._msgs.values() if m["state"] == PENDING]
            buffered_delivered = [
                m for m in self._msgs.values()
                if m["state"] == DELIVERED and m["buffered"]
            ]
            lines = [
                "===== DELIVERY VERIFIER REPORT =====",
                "sent:              %d" % self.sent,
                "delivered (ACKed): %d" % self.delivered,
                "still pending:     %d" % len(pend),
                "flushed after loss:%d" % len(buffered_delivered),
            ]
        if link_summary is not None:
            lines += [
                "duplicates (dedup):%s" % link_summary.get("duplicates"),
                "dropped (buf full):%s" % link_summary.get("dropped"),
                "retries:           %s" % link_summary.get("retries"),
                "rtt avg/p95 ms:    %s / %s"
                % (link_summary.get("rtt_ms_avg"), link_summary.get("rtt_ms_p95")),
            ]
            delivered = link_summary.get("delivered")
            sent = link_summary.get("sent")
            if sent:
                pct = 100.0 * (delivered or 0) / sent
                lines.append("delivered rate:    %.1f%%" % pct)
        lines.append("====================================")
        return "\n".join(lines)
