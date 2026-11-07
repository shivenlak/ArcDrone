#!/usr/bin/env bash
# One command to run the fake ground station playground.
#   bash ground_station/playground/run.sh            (default port 8000)
#   bash ground_station/playground/run.sh 8001       (other port)
set -e
cd "$(dirname "$0")/../.."          # repo root
PORT="${1:-8000}"
URL="http://localhost:$PORT"

if [ ! -x .venv/bin/python ]; then
  echo "Creating .venv ..."
  python3 -m venv .venv
fi
if ! .venv/bin/python -c "import fastapi, uvicorn" 2>/dev/null; then
  echo "Installing fastapi + uvicorn (first run only, can be slow) ..."
  .venv/bin/pip install -q fastapi "uvicorn[standard]"
fi

# Open the browser once the server answers (works on WSL, Linux, Mac).
(
  for _ in $(seq 1 60); do
    curl -s -o /dev/null "$URL/" && break
    sleep 1
  done
  if command -v wslview >/dev/null; then wslview "$URL"
  elif command -v explorer.exe >/dev/null; then explorer.exe "$URL" || true
  elif command -v xdg-open >/dev/null; then xdg-open "$URL"
  elif command -v open >/dev/null; then open "$URL"
  else echo "Open $URL in your browser."; fi
) &

echo "Playground at $URL  (Ctrl+C to stop)"
exec .venv/bin/python -m ground_station.playground.fake_server --port "$PORT"
