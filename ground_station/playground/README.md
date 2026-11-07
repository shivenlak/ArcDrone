# Ground Station Playground

A fake ground station you can click around in. No radio, no ESP32, no other teammate's code needed.

A fake drone lives inside the server and sends made-up alerts. The page shows them, and you can send commands back. It speaks the real WebSocket schema from `CLAUDE.md`, so it is a safe place to learn how the real `server.py` will behave.

**This is a dev tool.** The demo must use the real ESP32 radios.

## Run it

From the repo root:

```
bash ground_station/playground/run.sh
```

The script creates `.venv` if missing, installs `fastapi` and `uvicorn` on the first run, starts the server, and opens the page in your browser. If the browser does not open, go to http://localhost:8000. Stop with Ctrl+C.

Other port: `bash ground_station/playground/run.sh 8001`

Manual run (venv already set up):

```
.venv/bin/python -m ground_station.playground.fake_server --port 8000 --alert-every 2
```

## What you can do on the page

| Action | What happens |
|---|---|
| Wait | A fake alert (car, van, truck, pedestrian) arrives every 2 s. |
| Acknowledge or HOLD on an alert card | The command shows PENDING, then CONFIRMED about 0.3 s later. |
| HOLD / RESUME / RTB buttons | Same, and the drone mode changes once CONFIRMED. |
| Cut link | Light turns red, "LINK LOST". The fake drone buffers alerts. New commands stay PENDING. |
| Restore link | Buffered alerts arrive oldest first, with a DELAYED badge when older than 3 s. Pending commands turn CONFIRMED. |

Good demo of the reliability story: cut the link, press HOLD, restore the link.

## How it works

```
BROWSER (playground.html)  <--- WebSocket /ws --->  PYTHON SERVER (fake_server.py)
                                                    fake drone inside
```

- `fake_server.py` is a FastAPI app. It serves `playground.html` at `/` and a WebSocket at `/ws`.
- A WebSocket is a connection that stays open, so the server can push events to the page without being asked.
- `FakeStation` in `fake_server.py` holds the state: link up or down, drone mode, the buffer of held-back alerts, and commands waiting for the link.
- Three loops run in the background: one makes alerts, one sends a drone heartbeat and stats every 1 s, and the command handler answers button clicks.

### Messages

Server to browser (same as the contract in `CLAUDE.md`): `alert`, `link`, `drone`, `cmd_status`, `stats`.

Browser to server:

```json
{"action": "command", "cmd": "HOLD", "ref_seq": 4711}
```

`cmd` is one of `OPERATOR_ACK`, `HOLD`, `RESUME`, `RTB`.

The playground adds one message that is **not** in the contract, only for the Cut link button:

```json
{"action": "fake_link", "up": false}
```

## Files

| File | Purpose |
|---|---|
| `fake_server.py` | FastAPI server and fake drone |
| `playground.html` | The page, plain HTML and vanilla JS |
| `run.sh` | One-command launcher |

## Path to the real server

The real `ground_station/server.py` keeps the same WebSocket layer and the same JSON. Only the fake drone is replaced by `protocol.link.Link`, which talks to the ESP32 over USB serial:

```
drone -> radio -> ESP32 -> USB serial -> Link -> server.py -> WebSocket -> page
```

- `Link` callbacks `on_packet` and `on_event` replace the alert loop and `set_link`.
- `link.send(p.COMMAND, ...)` replaces the fake 0.3 s confirm. CONFIRMED comes from the `delivered` event with the matching `seq`.
- `link.summary()` replaces the fake stats.
