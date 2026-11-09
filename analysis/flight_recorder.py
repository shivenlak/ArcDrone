"""Flight recorder: log a live demo run, then turn it into a report to show the judges.

Run it next to the real demo (ground server already running):

    python -m analysis.flight_recorder record --note "8th floor, walker to lobby"
    python -m analysis.flight_recorder record --drone 192.168.1.20   # + drone laptop numbers over Wi-Fi

It listens to the same WebSocket as the dashboard (and, with --drone, the drone's
--preview-port stats). Ctrl+C stops it and writes runs/flight_<time>.jsonl + .html.
Rebuild a report from a recording:

    python -m analysis.flight_recorder report runs/flight_<time>.jsonl

Every number in the report is computed from the recording. Nothing is typed in by hand.
"""
import argparse
import asyncio
import collections
import html
import json
import statistics
import sys
import time
import urllib.request
from pathlib import Path

FRESH_MS = 3000  # a radio counts as "heard" if the ground board received on it in the last 3 s


# ----------------------------------------------------------------------------- recording

def record(args):
    import websockets

    out = Path(args.out)
    out.mkdir(exist_ok=True)
    path = out / time.strftime("flight_%Y%m%d_%H%M%S.jsonl")
    f = path.open("w")
    counts = collections.Counter()
    last_error = {}

    def log(msg):
        f.write(json.dumps(dict(msg, t=round(time.time(), 3))) + "\n")
        f.flush()

    log({"event": "recorder_start", "ws": args.ws, "drone": args.drone, "note": args.note})

    async def ws_loop():
        while True:
            try:
                async with websockets.connect(args.ws) as ws:
                    log({"event": "recorder_connected"})
                    async for text in ws:
                        msg = json.loads(text)
                        if isinstance(msg, dict):
                            log(msg)
                            counts[msg.get("event")] += 1
            except Exception as exc:  # server not up yet, or restarted: keep trying
                log({"event": "recorder_disconnected", "error": str(exc)[:120]})
                await asyncio.sleep(1)

    async def drone_loop():
        if not args.drone:
            return
        host = args.drone if "://" in args.drone else "http://" + args.drone + ("" if ":" in args.drone else ":8081")
        url = host.rstrip("/") + "/stats.json"
        loop = asyncio.get_running_loop()
        while True:
            try:
                body = await loop.run_in_executor(None, lambda: urllib.request.urlopen(url, timeout=2).read())
                log({"event": "drone_stats", "d": json.loads(body)})
                counts["drone_stats"] += 1
            except Exception as exc:
                counts["drone_unreachable"] += 1
                last_error["drone"] = str(exc)[:80]
            await asyncio.sleep(1)

    async def status_loop():
        start = time.time()
        while True:
            await asyncio.sleep(5)
            print("  %4ds  alerts %d  stats %d  commands %d  drone stats %d" % (
                time.time() - start, counts["alert"], counts["stats"], counts["cmd_status"],
                counts["drone_stats"]) + ("  (drone: %s)" % last_error["drone"]
                                          if args.drone and not counts["drone_stats"] and "drone" in last_error else ""), flush=True)

    async def main():
        await asyncio.gather(ws_loop(), drone_loop(), status_loop())

    print("recording to", path, "- press Ctrl+C to stop and build the report")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    log({"event": "recorder_stop"})
    f.close()
    report_path = build_report(path)
    print("\nreport:", report_path)


# ----------------------------------------------------------------------------- analysis

def heard(radio, name):
    age = radio.get(name + "_age")
    return isinstance(age, (int, float)) and 0 <= age < FRESH_MS


def segments(samples):
    """[(s, value)] at ~1 Hz -> [(start, end, value)] merging equal neighbours."""
    out = []
    for i, (s, v) in enumerate(samples):
        end = samples[i + 1][0] if i + 1 < len(samples) else s + 1
        if out and out[-1][2] == v and abs(out[-1][1] - s) < 2.5:
            out[-1][1] = end
        else:
            out.append([s, end, v])
    return [tuple(x) for x in out]


def analyse(rows):
    t0 = rows[0]["t"]
    for r in rows:
        r["s"] = r["t"] - t0
    a = {"duration": rows[-1]["s"], "note": rows[0].get("note") or ""}
    a["started"] = time.strftime("%Y-%m-%d %H:%M", time.localtime(t0))
    by = collections.defaultdict(list)
    for r in rows:
        by[r.get("event")].append(r)

    # Link state: 1 Hz stats plus exact link events.
    link_samples = sorted([(r["s"], bool(r["up"])) for r in by["stats"]] +
                          [(r["s"], bool(r["up"])) for r in by["link"]])
    a["link"] = segments(link_samples)
    first_up = next((s for s, e, up in a["link"] if up), None)
    a["first_contact"] = first_up
    outages = [(s, e) for s, e, up in a["link"] if not up and first_up is not None and s >= first_up]
    a["outages"] = []
    drone_ev = by["drone"]
    alerts = by["alert"]
    for s, e in outages:
        restored = e if e < a["duration"] else None
        o = {"start": s, "end": e, "secs": e - s, "restored": restored is not None}
        if restored is not None:
            after = [r for r in drone_ev if restored <= r["s"] <= restored + 10]
            o["buffered_at_restore"] = max([r.get("buffered") or 0 for r in after], default=None)
            cleared = next((r["s"] for r in drone_ev if r["s"] >= restored and r.get("buffered") == 0), None)
            o["flush_secs"] = (cleared - restored) if cleared is not None else None
            o["delayed_after"] = sum(1 for r in alerts if restored <= r["s"] <= restored + 60 and r.get("delayed"))
        a["outages"].append(o)

    # Radio paths, as heard by the ground board.
    radio_rows = [r for r in by["stats"] if isinstance(r.get("radio"), dict)]
    a["has_radio"] = bool(radio_rows)
    esp = [(r["s"], heard(r["radio"], "espnow")) for r in radio_rows]
    lora = [(r["s"], heard(r["radio"], "lora")) for r in radio_rows]
    a["espnow"], a["lora"] = segments(esp), segments(lora)
    a["rssi"] = [(r["s"], r["radio"].get("espnow_rssi") if heard(r["radio"], "espnow") else None,
                  r["radio"].get("lora_rssi") if heard(r["radio"], "lora") else None) for r in radio_rows]
    a["lora_sf"] = next((r["radio"].get("lora_sf") for r in radio_rows if r["radio"].get("lora_sf")), None)
    lora_alone = [s for (s, e_ok), (_, l_ok) in zip(esp, lora) if l_ok and not e_ok]
    a["lora_alone_secs"] = len(lora_alone)
    a["both_secs"] = sum(1 for (_, e_ok), (_, l_ok) in zip(esp, lora) if e_ok and l_ok)
    a["failovers"] = sum(1 for i in range(1, len(esp))
                         if esp[i - 1][1] and not esp[i][1] and lora[i][1])
    lone = set(int(s) for s in lora_alone)
    a["alerts_on_lora_alone"] = sum(1 for r in alerts if int(r["s"]) in lone or int(r["s"]) - 1 in lone)
    snrs = [r["radio"].get("lora_snr") for r in radio_rows if heard(r["radio"], "lora")]
    a["lora_worst"] = (min(x[2] for x in a["rssi"] if x[2] is not None), min(snrs)) if snrs else None
    esp_vals = [x[1] for x in a["rssi"] if x[1] is not None]
    a["espnow_worst"] = min(esp_vals) if esp_vals else None

    # Alerts at the operator.
    keys = [(r.get("seq"), r.get("ts_ms")) for r in alerts]
    a["alerts"] = [(r["s"], r.get("seq"), r.get("cls"), r.get("conf"), bool(r.get("delayed")), r.get("raw") or "")
                   for r in alerts]
    a["alerts_unique"] = len(set(keys))
    a["shown_twice"] = len(keys) - len(set(keys))
    a["delayed"] = sum(1 for r in alerts if r.get("delayed"))
    a["classes"] = collections.Counter(r.get("cls") for r in alerts).most_common()
    raws = [r.get("raw") or "" for r in alerts if r.get("raw")]
    a["alert_bytes"] = len(raws[0]) // 2 if raws else None
    a["encrypted"] = bool(raws) and all(x.startswith("D3") for x in raws)

    # Ground link counters (deltas over the recording).
    st = by["stats"]

    def delta(rows_, key):
        vals = [r.get(key) for r in rows_ if isinstance(r.get(key), (int, float))]
        return (vals[-1] - vals[0]) if len(vals) > 1 else 0

    a["copies_dropped"] = delta(st, "duplicates")
    a["bad_packets"] = delta(st, "bad_packets")
    rates = []
    for p, q in zip(st, st[1:]):
        if q["s"] - p["s"] > 0.2 and all(isinstance(x.get(k), int) for x in (p, q) for k in ("bytes_tx", "bytes_rx")):
            b = (q["bytes_tx"] + q["bytes_rx"]) - (p["bytes_tx"] + p["bytes_rx"])
            if b >= 0:
                rates.append((q["s"], 8 * b / (q["s"] - p["s"])))
    a["radio_bps"] = rates
    up_rates = [bps for s, bps in rates if any(ls <= s < le and up for ls, le, up in a["link"])]
    a["radio_bps_avg"] = statistics.mean(up_rates) if up_rates else None
    a["radio_bps_peak"] = max(up_rates) if up_rates else None

    # Operator commands: PENDING -> CONFIRMED, timed on this laptop (same clock as the server).
    cmds = collections.OrderedDict()
    for r in by["cmd_status"]:
        c = cmds.setdefault(r.get("cmd_id"), {"cmd": r.get("cmd"), "ref": r.get("ref_seq")})
        if r.get("status") == "PENDING" and "pending" not in c:
            c["pending"] = r["s"]
            c["during_outage"] = any(o["start"] <= r["s"] < o["end"] for o in a["outages"])
        elif r.get("status") == "CONFIRMED":
            c["confirmed"] = r["s"]
    a["commands"] = [c for c in cmds.values() if "pending" in c]
    for c in a["commands"]:
        c["ms"] = (c["confirmed"] - c["pending"]) * 1000 if "confirmed" in c else None
    a["modes"] = []
    last = None
    for r in drone_ev:
        if r.get("mode") and r["mode"] != last:
            a["modes"].append((r["s"], r["mode"]))
            last = r["mode"]

    # Drone laptop numbers (optional, over Wi-Fi).
    ds = [r for r in by["drone_stats"] if isinstance(r.get("d"), dict)]
    a["drone"] = None
    if ds:
        d = [r["d"] for r in ds]

        def dd(key):
            vals = [x.get(key) for x in d if isinstance(x.get(key), (int, float))]
            return (vals[-1] - vals[0]) if len(vals) > 1 else None

        def avg(key):
            vals = [x.get(key) for x in d if isinstance(x.get(key), (int, float))]
            return statistics.mean(vals) if vals else None

        span = ds[-1]["s"] - ds[0]["s"]
        video = dd("video_bytes_sent")
        a["drone"] = {
            "alerts_sent": dd("alerts_sent"), "delivered": dd("delivered"), "retries": dd("retries"),
            "dropped": dd("dropped"), "buffered_end": d[-1].get("buffered"),
            "infer_ms": avg("infer_ms"), "fps": avg("fps"), "objects": avg("objects"),
            "rtt_ms": statistics.median([x["rtt_ms_avg"] for x in d if x.get("rtt_ms_avg")] or [0]) or None,
            "model": d[-1].get("model"), "imgsz": d[-1].get("imgsz"), "conf_gate": d[-1].get("conf_gate"),
            "source": d[-1].get("source"),
            "video_bps": (8 * video / span) if video and span > 1 else None,
        }
    return a


# ----------------------------------------------------------------------------- report

def mmss(s):
    return "%d:%02d" % (int(s) // 60, int(s) % 60)


def fmt(x, unit="", nd=0):
    if x is None:
        return "–"
    return ("{:,.%df}" % nd).format(x) + unit


def bps_text(b):
    if b is None:
        return "–"
    if b >= 1e6:
        return "%.1f Mbit/s" % (b / 1e6)
    if b >= 1e3:
        return "%.1f kbit/s" % (b / 1e3)
    return "%.0f bit/s" % b


def times(n):
    return "once" if n == 1 else "%d times" % n


def conclusions(a):
    out = []
    d = a["drone"]
    if a["alerts_unique"]:
        s = "%d alerts reached the operator; none was shown twice" % a["alerts_unique"]
        if a["copies_dropped"]:
            s += " (%d extra radio copies, the same alert arriving on both radios or a retry, were dropped)" % a["copies_dropped"]
        out.append(s + ".")
    if d and d["alerts_sent"]:
        missing = d["alerts_sent"] - a["alerts_unique"] - (d["buffered_end"] or 0)
        s = "The drone created %d alerts during the recording; the ground received %d" % (d["alerts_sent"], a["alerts_unique"])
        if d["buffered_end"]:
            s += " and %d were still queued when it stopped" % d["buffered_end"]
        if d.get("dropped"):
            s += "; %d were dropped because an outage outlasted the 64-alert buffer" % d["dropped"]
        elif missing <= 0:
            s += "; none was lost"
        out.append(s + ".")
    restored = [o for o in a["outages"] if o["restored"]]
    if a["outages"]:
        longest = max(o["secs"] for o in a["outages"])
        n = len(a["outages"])
        s = "The link was lost %s (longest %.0f s)" % (times(n), longest)
        flushes = [o["flush_secs"] for o in restored if o.get("flush_secs") is not None]
        if restored:
            s += " and came back by itself" + ("" if n == 1 else " every time") if len(restored) == n else ""
        if flushes:
            s += "; buffered alerts were flushed within %.0f s of each restore" % max(flushes)
        delayed = sum(o.get("delayed_after") or 0 for o in restored)
        if delayed:
            s += ", %d of them tagged DELAYED" % delayed
        out.append(s + ".")
    if a["failovers"]:
        out.append("ESP-NOW dropped out %s while LoRa kept the link up; LoRa alone carried the link for %d s "
                   "and %d alerts arrived during that time." % (times(a["failovers"]), a["lora_alone_secs"],
                                                               a["alerts_on_lora_alone"]))
    elif a["has_radio"] and a["both_secs"]:
        out.append("Both radios carried every packet for %d s; neither path dropped out." % a["both_secs"])
    if a["lora_worst"] and a["lora_worst"][1] is not None and a["lora_worst"][1] < 0:
        out.append("LoRa was still decoded at %d dBm with SNR %.1f dB, below the noise floor." % a["lora_worst"])
    cmds = a["commands"]
    if cmds:
        ok = [c for c in cmds if c["ms"] is not None]
        s = "%d of %d operator commands were CONFIRMED by the drone" % (len(ok), len(cmds))
        live = [c["ms"] for c in ok if not c["during_outage"]]
        if live:
            s += "; median PENDING to CONFIRMED %.0f ms with the link up" % statistics.median(live)
        held = [c for c in cmds if c["during_outage"]]
        if held:
            s += "; %d sent during an outage stayed PENDING and %d landed after restore" % (
                len(held), sum(1 for c in held if c["ms"] is not None))
        out.append(s + ".")
    if a["radio_bps_avg"]:
        s = "Radio traffic averaged %s (alerts, ACKs and heartbeats, both directions)" % bps_text(a["radio_bps_avg"])
        if d and d.get("video_bps"):
            s += "; the drone's Wi-Fi video used %s, about %s× more" % (bps_text(d["video_bps"]), fmt(d["video_bps"] / a["radio_bps_avg"]))
        out.append(s + ".")
    if d and d.get("infer_ms"):
        out.append("On the drone, %s at imgsz %s took %.0f ms per frame (%.1f frames/s), ~%.0f objects per frame." % (
            d.get("model") or "the model", d.get("imgsz"), d["infer_ms"], d["fps"] or 0, d["objects"] or 0))
    return out


CSS = """
:root{color-scheme:light;--page:#f9f9f7;--surface:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--muted:#898781;
--grid:#e1e0d9;--axis:#c3c2b7;--ring:rgba(11,11,11,.10);--s1:#2a78d6;--s2:#eb6834;--good:#0ca30c;--warn:#fab219;--crit:#d03b3b}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){color-scheme:dark;--page:#0d0d0d;--surface:#1a1a19;
--ink:#f0efec;--ink2:#c3c2b7;--grid:#2c2c2a;--axis:#383835;--ring:rgba(255,255,255,.10);--s1:#3987e5;--s2:#d95926}}
:root[data-theme=dark]{color-scheme:dark;--page:#0d0d0d;--surface:#1a1a19;--ink:#f0efec;--ink2:#c3c2b7;--grid:#2c2c2a;
--axis:#383835;--ring:rgba(255,255,255,.10);--s1:#3987e5;--s2:#d95926}
*{box-sizing:border-box}body{margin:0;background:var(--page);color:var(--ink);font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1040px;margin:0 auto;padding:32px 16px 64px}h1{font-size:26px;margin:0 0 4px}h2{font-size:17px;margin:36px 0 10px}
.sub{color:var(--ink2);margin:0 0 24px}.card{background:var(--surface);border:1px solid var(--ring);border-radius:10px;padding:16px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}.tile .v{font-size:28px;font-weight:600}
.tile .k{color:var(--ink2);font-size:13px}.tile .n{color:var(--muted);font-size:12px}
ul.concl{margin:0;padding-left:20px}ul.concl li{margin:6px 0}svg{width:100%;height:auto;display:block}
.axis text,.lane{fill:var(--muted);font-size:11px}.lbl{fill:var(--ink2);font-size:12px}.gridl{stroke:var(--grid);stroke-width:1}
.legend{display:flex;gap:16px;flex-wrap:wrap;color:var(--ink2);font-size:13px;margin:0 0 8px}.sw{display:inline-block;width:12px;height:12px;border-radius:3px;margin-right:6px;vertical-align:-1px}
table{border-collapse:collapse;width:100%;font-size:13px;font-variant-numeric:tabular-nums}th,td{text-align:left;padding:6px 8px;border-bottom:1px solid var(--grid)}
th{color:var(--ink2);font-weight:600}.scroll{overflow-x:auto}#tip{position:fixed;pointer-events:none;background:var(--surface);color:var(--ink);
border:1px solid var(--ring);border-radius:6px;padding:6px 8px;font-size:12px;box-shadow:0 2px 8px rgba(0,0,0,.15);display:none;white-space:pre;z-index:9}
.cross{stroke:var(--axis);stroke-width:1;display:none}.foot{color:var(--muted);font-size:12px;margin-top:32px}
"""

JS = """
const tip=document.getElementById('tip');
function show(e,t){tip.textContent=t;tip.style.display='block';const w=tip.offsetWidth;
tip.style.left=Math.min(e.clientX+14,innerWidth-w-8)+'px';tip.style.top=(e.clientY+14)+'px'}
function hide(){tip.style.display='none'}
document.querySelectorAll('[data-tip]').forEach(el=>{el.addEventListener('mousemove',e=>show(e,el.dataset.tip));el.addEventListener('mouseleave',hide)});
document.querySelectorAll('svg[data-line]').forEach(svg=>{const d=JSON.parse(svg.dataset.line);const c=svg.querySelector('.cross');
svg.addEventListener('mousemove',e=>{const p=svg.createSVGPoint();p.x=e.clientX;p.y=e.clientY;const q=p.matrixTransform(svg.getScreenCTM().inverse());
if(q.x<d.L||q.x>d.W-d.R){c.style.display='none';hide();return}
const s=d.x0+(q.x-d.L)/(d.W-d.L-d.R)*(d.x1-d.x0);let i=0,b=1e9;d.xs.forEach((x,j)=>{const k=Math.abs(x-s);if(k<b){b=k;i=j}});
const X=d.L+(d.xs[i]-d.x0)/(d.x1-d.x0)*(d.W-d.L-d.R);c.setAttribute('x1',X);c.setAttribute('x2',X);c.style.display='block';
const m=Math.floor(d.xs[i]/60),sec=Math.floor(d.xs[i]%60);let t=m+':'+String(sec).padStart(2,'0');
d.series.forEach(s=>{const v=s.ys[i];t+='\\n'+s.name+': '+(v==null?'not heard':v+d.unit)});show(e,t)});
svg.addEventListener('mouseleave',()=>{c.style.display='none';hide()})});
"""

W, L, R = 960, 92, 92


def xscale(a):
    span = max(a["duration"], 1)
    return lambda s: L + s / span * (W - L - R)


def time_axis(a, y, top):
    sx = xscale(a)
    step = next(st for st in (5, 10, 15, 30, 60, 120, 300, 600, 1200, 3600) if a["duration"] / st <= 8)
    parts = []
    t = 0
    while t <= a["duration"] + 0.01:
        x = sx(t)
        parts.append('<line class="gridl" x1="%.1f" x2="%.1f" y1="%d" y2="%d"/>' % (x, x, top, y))
        parts.append('<text x="%.1f" y="%d" text-anchor="middle">%s</text>' % (x, y + 16, mmss(t)))
        t += step
    return '<g class="axis">%s</g>' % "".join(parts)


def timeline_svg(a):
    sx = xscale(a)
    lanes = [("Link", 0), ("ESP-NOW", 1), ("LoRa", 2), ("Alerts", 3), ("Commands", 4)]
    lh, gap, top = 22, 12, 8
    H = top + len(lanes) * (lh + gap) + 28
    y = lambda i: top + i * (lh + gap)
    p = [time_axis(a, H - 28, top)]
    for name, i in lanes:
        p.append('<text class="lane" x="%d" y="%.1f" text-anchor="end">%s</text>' % (L - 10, y(i) + lh / 2 + 4, name))
        p.append('<line class="gridl" x1="%d" x2="%d" y1="%.1f" y2="%.1f"/>' % (L, W - R, y(i) + lh / 2, y(i) + lh / 2))

    def bar(i, s, e, color, tipt, h=lh):
        w = max(sx(e) - sx(s) - 2, 1.5)  # 2px surface gap between neighbours
        return ('<rect x="%.1f" y="%.1f" width="%.1f" height="%d" rx="4" fill="%s" data-tip="%s"/>'
                % (sx(s) + 1, y(i) + (lh - h) / 2, w, h, color, html.escape(tipt)))

    for s, e, up in a["link"]:
        if a["first_contact"] is None or e <= a["first_contact"]:
            p.append(bar(0, s, e, "var(--axis)", "Waiting for first contact %s–%s" % (mmss(s), mmss(e))))
        else:
            p.append(bar(0, s, e, "var(--good)" if up else "var(--crit)",
                         "Link %s %s–%s (%.0f s)" % ("UP" if up else "LOST", mmss(s), mmss(e), e - s)))
    for lane, segs, color, name in ((1, a["espnow"], "var(--s1)", "ESP-NOW"), (2, a["lora"], "var(--s2)", "LoRa")):
        for s, e, ok in segs:
            if ok:
                p.append(bar(lane, s, e, color, "%s heard %s–%s (%.0f s)" % (name, mmss(s), mmss(e), e - s)))
    for s, seq, cls, conf, delayed, raw in a["alerts"]:
        x = sx(s)
        tipt = "Alert #%s %s %s%s at %s" % (seq, cls, fmt((conf or 0) * 100, "%"), " DELAYED" if delayed else "", mmss(s))
        p.append('<g data-tip="%s"><rect x="%.1f" y="%.1f" width="8" height="%d" fill="transparent"/>'
                 '<rect x="%.1f" y="%.1f" width="2" height="%d" rx="1" fill="%s"/></g>'
                 % (html.escape(tipt), x - 4, y(3), lh, x - 1, y(3) + 3, lh - 6,
                    "var(--warn)" if delayed else "var(--ink2)"))
    for c in a["commands"]:
        e = c.get("confirmed", a["duration"])
        tipt = "%s %s → %s" % (c["cmd"], mmss(c["pending"]),
                               ("CONFIRMED after %.0f ms" % c["ms"]) if c["ms"] is not None else "never confirmed")
        p.append(bar(4, c["pending"], max(e, c["pending"] + 0.6), "var(--ink2)", tipt, h=10))
    return '<svg viewBox="0 0 %d %d" role="img" aria-label="Timeline">%s</svg>' % (W, H, "".join(p))


def line_svg(a, xs, series, unit, ylo, yhi, label_fmt):
    sx = xscale(a)
    H, top, bot = 260, 14, 30
    sy = lambda v: top + (yhi - v) / (yhi - ylo) * (H - top - bot)
    p = [time_axis(a, H - bot, top)]
    step = next(st for st in (1, 2, 5, 10, 20, 25, 50, 100, 200, 500, 1000, 2000, 5000) if (yhi - ylo) / st <= 6)
    v = ylo - (ylo % step) + (step if ylo % step else 0)
    while v <= yhi:
        p.append('<line class="gridl" x1="%d" x2="%d" y1="%.1f" y2="%.1f"/>' % (L, W - R, sy(v), sy(v)))
        p.append('<text class="lane" x="%d" y="%.1f" text-anchor="end">%s</text>' % (L - 8, sy(v) + 4, label_fmt(v)))
        v += step
    for s in series:
        d, pen = [], "M"
        for x, yv in zip(xs, s["ys"]):
            if yv is None:
                pen = "M"
                continue
            d.append("%s%.1f %.1f" % (pen, sx(x), sy(yv)))
            pen = "L"
        p.append('<path d="%s" fill="none" stroke="%s" stroke-width="2" stroke-linejoin="round"/>' % (" ".join(d), s["color"]))
        last = next(((x, yv) for x, yv in zip(reversed(xs), reversed(s["ys"])) if yv is not None), None)
        if last and len(series) > 1:
            p.append('<text class="lbl" x="%.1f" y="%.1f">%s</text>' % (sx(last[0]) + 6, sy(last[1]) + 4, s["name"]))
    p.append('<line class="cross" x1="0" x2="0" y1="%d" y2="%d"/>' % (top, H - bot))
    data = {"xs": [round(x, 1) for x in xs], "x0": 0, "x1": max(a["duration"], 1), "W": W, "L": L, "R": R, "unit": unit,
            "series": [{"name": s["name"], "ys": [None if v is None else round(v, 1) for v in s["ys"]]} for s in series]}
    return ('<svg viewBox="0 0 %d %d" data-line="%s" role="img">%s</svg>'
            % (W, H, html.escape(json.dumps(data)), "".join(p)))


def class_bars(a):
    if not a["classes"]:
        return ""
    top = a["classes"][0][1]
    rows = []
    for i, (cls, n) in enumerate(a["classes"][:8]):
        w = n / top * (W - 300)
        y = i * 28
        rows.append('<text class="lbl" x="%d" y="%d" text-anchor="end">%s</text>' % (L + 20, y + 15, html.escape(str(cls))))
        rows.append('<rect x="%d" y="%d" width="%.1f" height="16" rx="4" fill="var(--s1)" data-tip="%s: %d alerts"/>'
                    % (L + 30, y + 3, max(w, 2), html.escape(str(cls)), n))
        rows.append('<text class="lbl" x="%.1f" y="%d">%d</text>' % (L + 36 + w, y + 15, n))
    return '<svg viewBox="0 0 %d %d" role="img">%s</svg>' % (W, len(rows) // 3 * 28 + 4, "".join(rows))


def tile(value, key, note=""):
    return '<div class="card tile"><div class="v">%s</div><div class="k">%s</div><div class="n">%s</div></div>' % (
        html.escape(value), html.escape(key), html.escape(note))


def build_report(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    if len(rows) < 2:
        sys.exit("recording is empty: was the ground server running?")
    a = analyse(rows)
    d = a["drone"]
    radio = "LoRa SF%s + ESP-NOW" % a["lora_sf"] if a["lora_sf"] else "ESP-NOW"
    sub = "Recorded %s · %s · %s%s" % (a["started"], mmss(a["duration"]), radio,
                                       " · encrypted (AES-128-CCM)" if a["encrypted"] else "")
    if a["note"]:
        sub += " · " + a["note"]

    delivery_note = ""
    if d and d["alerts_sent"]:
        delivery_note = "drone created %d" % d["alerts_sent"]
    ok_cmds = [c for c in a["commands"] if c["ms"] is not None]
    live_ms = [c["ms"] for c in ok_cmds if not c["during_outage"]]
    tiles = [
        tile(fmt(a["alerts_unique"]), "alerts at the operator", delivery_note),
        tile(fmt(a["shown_twice"]), "alerts shown twice", "%d radio copies dropped" % a["copies_dropped"]),
        tile(fmt(len(a["outages"])), "link outages",
             ("longest %.0f s" % max(o["secs"] for o in a["outages"])) if a["outages"] else "none"),
        tile("%d/%d" % (len(ok_cmds), len(a["commands"])), "commands confirmed",
             ("median %.0f ms with link up" % statistics.median(live_ms)) if live_ms else ""),
        tile(bps_text(a["radio_bps_avg"]), "radio traffic, average",
             ("%d B per alert" % a["alert_bytes"]) if a["alert_bytes"] else ""),
    ]
    if a["has_radio"]:
        tiles.append(tile("%d s" % a["lora_alone_secs"], "carried by LoRa alone", "ESP-NOW dropped out %s" % times(a["failovers"])))
    if d and d.get("video_bps"):
        tiles.append(tile(bps_text(d["video_bps"]), "Wi-Fi video (not radio)", "for comparison"))

    body = ['<h1>ArcDrone flight report</h1><p class="sub">%s</p>' % html.escape(sub),
            '<div class="tiles">%s</div>' % "".join(tiles)]
    concl = conclusions(a)
    if concl:
        body.append('<h2>What this run shows</h2><div class="card"><ul class="concl">%s</ul></div>'
                    % "".join("<li>%s</li>" % html.escape(c) for c in concl))
    body.append('<h2>Timeline</h2><div class="card"><div class="legend">'
                '<span><i class="sw" style="background:var(--good)"></i>Link up</span>'
                '<span><i class="sw" style="background:var(--crit)"></i>Link lost</span>'
                '<span><i class="sw" style="background:var(--s1)"></i>ESP-NOW heard</span>'
                '<span><i class="sw" style="background:var(--s2)"></i>LoRa heard</span>'
                '<span><i class="sw" style="background:var(--ink2)"></i>Alert / command (PENDING → CONFIRMED)</span>'
                '<span><i class="sw" style="background:var(--warn)"></i>Alert tagged DELAYED</span></div>%s</div>'
                % timeline_svg(a))
    if a["has_radio"] and any(x[1] is not None or x[2] is not None for x in a["rssi"]):
        xs = [x[0] for x in a["rssi"]]
        vals = [v for x in a["rssi"] for v in x[1:] if v is not None]
        lo, hi = min(vals) - 5, max(vals) + 5
        body.append('<h2>Signal strength at the ground board</h2><div class="card"><div class="legend">'
                    '<span><i class="sw" style="background:var(--s1)"></i>ESP-NOW</span>'
                    '<span><i class="sw" style="background:var(--s2)"></i>LoRa</span>'
                    '<span>gaps = not heard · closer to 0 dBm is stronger</span></div>%s</div>' % line_svg(
                        a, xs, [{"name": "ESP-NOW", "color": "var(--s1)", "ys": [x[1] for x in a["rssi"]]},
                                {"name": "LoRa", "color": "var(--s2)", "ys": [x[2] for x in a["rssi"]]}],
                        " dBm", lo, hi, lambda v: "%d dBm" % v))
    if a["radio_bps"]:
        xs = [x[0] for x in a["radio_bps"]]
        ys = [x[1] for x in a["radio_bps"]]
        body.append('<h2>Radio traffic (bit/s, both directions, at the ground board)</h2><div class="card">%s</div>'
                    % line_svg(a, xs, [{"name": "Radio", "color": "var(--s1)", "ys": ys}], " bit/s", 0,
                               max(ys + [100]) * 1.1, lambda v: "%d" % v))
    if a["classes"]:
        body.append('<h2>Alerts by class</h2><div class="card">%s</div>' % class_bars(a))
    if a["outages"]:
        trs = "".join("<tr><td>%s</td><td>%.0f s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
            mmss(o["start"]), o["secs"], "yes" if o["restored"] else "no (still down at end)",
            fmt(o.get("buffered_at_restore")), fmt(o.get("flush_secs"), " s")) for o in a["outages"])
        body.append('<h2>Outages</h2><div class="card scroll"><table><tr><th>Lost at</th><th>Lasted</th>'
                    '<th>Came back</th><th>Alerts queued on the drone</th><th>Time to flush</th></tr>%s</table></div>' % trs)
    if a["commands"]:
        trs = "".join("<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
            mmss(c["pending"]), html.escape(str(c["cmd"])), c["ref"] or "–",
            "yes" if c["during_outage"] else "no", fmt(c["ms"], " ms") if c["ms"] is not None else "never")
            for c in a["commands"])
        body.append('<h2>Operator commands</h2><div class="card scroll"><table><tr><th>Sent at</th><th>Command</th>'
                    '<th>For alert</th><th>During outage</th><th>PENDING → CONFIRMED</th></tr>%s</table></div>' % trs)
    if d:
        rowsd = [("Model", "%s, imgsz %s, confidence gate %s" % (d.get("model"), d.get("imgsz"), d.get("conf_gate"))),
                 ("Footage", d.get("source")), ("YOLO time per frame", fmt(d["infer_ms"], " ms")),
                 ("Frames processed", fmt(d["fps"], " /s", 1)), ("Objects per frame", fmt(d["objects"])),
                 ("Alert round trip (drone side, median)", fmt(d["rtt_ms"], " ms")),
                 ("Retries (drone)", fmt(d["retries"])), ("Still queued at end", fmt(d["buffered_end"]))]
        body.append('<h2>Drone side</h2><div class="card scroll"><table>%s</table></div>' % "".join(
            "<tr><th>%s</th><td>%s</td></tr>" % (html.escape(k), html.escape(str(v))) for k, v in rowsd))
    body.append('<p class="foot">Computed from %s (%d recorded events). GPS positions are simulated. Signal strength is '
                'what the ground board heard. The video goes over Wi-Fi, never over the radio.</p>'
                % (html.escape(Path(path).name), len(rows)))
    page = ('<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,'
            'initial-scale=1"><title>ArcDrone flight report</title><style>%s</style></head><body><main>%s</main>'
            '<div id="tip"></div><script>%s</script></body></html>') % (CSS, "".join(body), JS)
    out = Path(path).with_suffix(".html")
    out.write_text(page)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record", help="record a live run (Ctrl+C to stop and build the report)")
    r.add_argument("--ws", default="ws://localhost:8001/ws", help="ground server WebSocket")
    r.add_argument("--drone", help="drone laptop IP (or URL) for its --preview-port stats, optional")
    r.add_argument("--note", default="", help="what this run is, shown on the report")
    r.add_argument("--out", default="runs", help="folder for recordings")
    b = sub.add_parser("report", help="build the HTML report from a recording")
    b.add_argument("path")
    args = ap.parse_args()
    if args.cmd == "record":
        record(args)
    else:
        print(build_report(args.path))


if __name__ == "__main__":
    main()
