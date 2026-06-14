#!/usr/bin/env python3
"""
Test script for POIM WebSocket ingest + consume.

Usage:
  # Test ingest only (push one message to /ws/ingest_poim)
  python test_poim_ws.py --host localhost --port 8010 --mode ingest

  # Test consume only (subscribe to /ws/poim and print what arrives)
  python test_poim_ws.py --host localhost --port 8010 --mode consume

  # Full round-trip: ingest then verify it appears on /ws/poim
  python test_poim_ws.py --host localhost --port 8010 --mode roundtrip

  # Keep sending in a loop (default interval: 1s, Ctrl+C to stop)
  python test_poim_ws.py --host localhost --port 8010 --mode ingest --loop
  python test_poim_ws.py --host localhost --port 8010 --mode ingest --loop --interval 2.5

  # Loop with a fixed number of iterations
  python test_poim_ws.py --host localhost --port 8010 --mode ingest --loop --count 10

  # Use wss:// (TLS) instead of ws://
  python test_poim_ws.py --host dd14-193-50-192-71.ngrok-free.app --port 443 --tls --mode roundtrip
"""

import argparse
import asyncio
import json
import sys
import time

try:
    from websockets.asyncio.client import connect as ws_connect
except ImportError:
    from websockets import connect as ws_connect  # type: ignore

# ── Payload ────────────────────────────────────────────────────────────────────

_POIM_POI_TEMPLATE = {
    "providerId": 889,
    "id": 43,
    "location": {
        "lat": 48.0274744147061,
        "lon": 7.371491053787699,
        "name": "Aire de Fronholz",
    },
    "type": 1,
    "content": {
        "currentFacilityStatus": 15,
        "currentOccupancy": {
            "rate": 0,
            "trend": 7,
            "totalSpaces": 25,
            "freeSpaces": 25,
            "confidence": 0,
        },
    },
}


def _make_payload() -> dict:
    """Build a fresh payload with timestamp = now (ms)."""
    return {
        "method": "poitrigger",
        "params": {
            "poi": {
                **_POIM_POI_TEMPLATE,
                "timestamp": int(time.time() * 1000),
            }
        },
        "id": 12,+
    }

# ── Helpers ────────────────────────────────────────────────────────────────────

def _url(host: str, port: int, path: str, tls: bool) -> str:
    scheme = "wss" if tls else "ws"
    # Standard ports: omit port number to avoid nginx/ngrok rejections
    if (tls and port == 443) or (not tls and port == 80):
        return f"{scheme}://{host}{path}"
    return f"{scheme}://{host}:{port}{path}"


def _ok(msg: str) -> None:
    print(f"  ✅  {msg}")


def _fail(msg: str) -> None:
    print(f"  ❌  {msg}")


def _info(msg: str) -> None:
    print(f"  ℹ️   {msg}")


# ── Test modes ─────────────────────────────────────────────────────────────────

async def test_ingest(host: str, port: int, tls: bool) -> bool:
    url = _url(host, port, "/ws/ingest_poim", tls)
    print(f"\n[INGEST] Connecting to {url} ...")
    try:
        async with ws_connect(url) as ws:
            _ok("Connected to /ws/ingest_poim")
            msg = json.dumps(_make_payload())
            await ws.send(msg)
            _ok(f"Sent payload ({len(msg)} bytes)")
            print(f"         Payload: {msg[:120]}{'...' if len(msg) > 120 else ''}")
            # Give server a moment to process
            await asyncio.sleep(0.3)
        _ok("Connection closed cleanly")
        return True
    except Exception as e:
        _fail(f"Ingest failed: {type(e).__name__}: {e}")
        return False


async def test_ingest_loop(
    host: str,
    port: int,
    tls: bool,
    interval: float,
    count: int,          # 0 = infinite
) -> bool:
    """
    Open a single persistent WebSocket connection to /ws/ingest_poim and keep
    sending the payload every `interval` seconds.

    Press Ctrl+C to stop, or pass --count N to send exactly N messages.
    The timestamp field is updated on every iteration so each message is unique.
    """
    url = _url(host, port, "/ws/ingest_poim", tls)
    limit = f"{count} messages" if count > 0 else "∞  (Ctrl+C to stop)"
    print(f"\n[INGEST-LOOP] Connecting to {url}")
    print(f"              Interval : {interval}s  |  Limit : {limit}\n")

    iteration = 0
    try:
        async with ws_connect(url) as ws:
            _ok("Connected — starting loop …")
            while True:
                iteration += 1
                msg = json.dumps(_make_payload())
                await ws.send(msg)
                ts = time.strftime("%H:%M:%S")
                print(f"  [{ts}]  #{iteration:>5}  sent {len(msg)} bytes")

                if count > 0 and iteration >= count:
                    _ok(f"Reached requested count ({count}). Done.")
                    break

                await asyncio.sleep(interval)

    except KeyboardInterrupt:
        print(f"\n  ⏹️   Interrupted after {iteration} message(s).")
        return True
    except Exception as e:
        _fail(f"Loop ingest failed: {type(e).__name__}: {e}")
        return False

    _ok(f"Loop finished — {iteration} message(s) sent.")
    return True


async def test_consume(host: str, port: int, tls: bool, timeout: float = 5.0) -> bool:
    url = _url(host, port, "/ws/poim", tls)
    print(f"\n[CONSUME] Connecting to {url} ...")
    try:
        async with ws_connect(url) as ws:
            _ok("Connected to /ws/poim")
            print(f"          Waiting up to {timeout}s for a message ...")
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=timeout)
                data = json.loads(raw)
                pois = data.get("pois", [])
                _ok(f"Received snapshot — {len(pois)} POI(s) in store")
                for poi in pois:
                    poi_id = poi.get("id", "?")
                    name = (poi.get("location") or {}).get("name") or poi.get("name", "?")
                    occ = (poi.get("content") or {}).get("currentOccupancy", {})
                    free = occ.get("freeSpaces", "?")
                    total = occ.get("totalSpaces", "?")
                    print(f"          → POI {poi_id}: '{name}'  free={free}/{total}")
                return True
            except asyncio.TimeoutError:
                _fail(f"No message received within {timeout}s")
                return False
    except Exception as e:
        _fail(f"Consume failed: {type(e).__name__}: {e}")
        return False


async def test_roundtrip(host: str, port: int, tls: bool, timeout: float = 5.0) -> bool:
    print(f"\n[ROUNDTRIP] Full ingest → consume verification")

    # Step 1: subscribe first so we don't miss the broadcast
    consume_url = _url(host, port, "/ws/poim", tls)
    ingest_url  = _url(host, port, "/ws/ingest_poim", tls)
    target_id   = str(_POIM_POI_TEMPLATE["id"])

    print(f"  Step 1 — Subscribe to {consume_url}")
    try:
        async with ws_connect(consume_url) as consumer:
            _ok("Subscribed to /ws/poim")

            # Drain the initial state snapshot
            initial_raw = await asyncio.wait_for(consumer.recv(), timeout=3.0)
            initial = json.loads(initial_raw)
            _info(f"Initial snapshot has {len(initial.get('pois', []))} POI(s)")

            # Step 2: ingest
            print(f"  Step 2 — Ingest to {ingest_url}")
            async with ws_connect(ingest_url) as ingester:
                await ingester.send(json.dumps(_make_payload()))
                _ok("Payload sent to /ws/ingest_poim")
                await asyncio.sleep(0.1)

            # Step 3: wait for broadcast on consumer
            print(f"  Step 3 — Waiting up to {timeout}s for broadcast on /ws/poim ...")
            deadline = time.monotonic() + timeout
            found = False
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                try:
                    raw = await asyncio.wait_for(consumer.recv(), timeout=remaining)
                    data = json.loads(raw)
                    pois = data.get("pois", [])
                    for poi in pois:
                        if str(poi.get("id")) == target_id:
                            found = True
                            name = (poi.get("location") or {}).get("name", "?")
                            occ  = (poi.get("content") or {}).get("currentOccupancy", {})
                            free  = occ.get("freeSpaces", "?")
                            total = occ.get("totalSpaces", "?")
                            _ok(
                                f"POI id={target_id} found in broadcast: "
                                f"'{name}' free={free}/{total}"
                            )
                            break
                    if found:
                        break
                except asyncio.TimeoutError:
                    break

            if not found:
                _fail(f"POI id={target_id} never appeared in /ws/poim broadcast")
                return False

            return True

    except Exception as e:
        _fail(f"Roundtrip failed: {type(e).__name__}: {e}")
        return False


# ── Entry point ────────────────────────────────────────────────────────────────

async def main(args: argparse.Namespace) -> int:
    print("=" * 60)
    print("  POIM WebSocket test")
    print(f"  Server : {'wss' if args.tls else 'ws'}://{args.host}:{args.port}")
    mode_label = args.mode + (" [LOOP]" if args.loop else "")
    print(f"  Mode   : {mode_label}")
    print("=" * 60)

    ok = False

    if args.loop:
        if args.mode not in ("ingest",):
            print(f"⚠️  --loop is only supported with --mode ingest (got '{args.mode}')")
            return 1
        ok = await test_ingest_loop(
            args.host, args.port, args.tls,
            interval=args.interval,
            count=args.count,
        )
    elif args.mode == "ingest":
        ok = await test_ingest(args.host, args.port, args.tls)
    elif args.mode == "consume":
        ok = await test_consume(args.host, args.port, args.tls, args.timeout)
    elif args.mode == "roundtrip":
        ok = await test_roundtrip(args.host, args.port, args.tls, args.timeout)
    else:
        print(f"Unknown mode: {args.mode}")
        return 1

    print()
    if ok:
        print("✅  All checks passed")
    else:
        print("❌  One or more checks failed")
    return 0 if ok else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="POIM WebSocket test")
    parser.add_argument("--host", default="localhost", help="Server hostname or IP")
    parser.add_argument("--port", type=int, default=8010, help="Server port")
    parser.add_argument("--tls", action="store_true", help="Use wss:// instead of ws://")
    parser.add_argument(
        "--mode",
        choices=["ingest", "consume", "roundtrip"],
        default="roundtrip",
        help="Test mode (default: roundtrip)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for a message (default: 5)",
    )

    # ── Loop options ──────────────────────────────────────────────────────────
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Keep sending in a loop until Ctrl+C (only with --mode ingest)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        metavar="SECONDS",
        help="Seconds between sends when --loop is active (default: 1.0)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        metavar="N",
        help="Stop after N sends when --loop is active; 0 = infinite (default: 0)",
    )

    args = parser.parse_args()
    sys.exit(asyncio.run(main(args)))