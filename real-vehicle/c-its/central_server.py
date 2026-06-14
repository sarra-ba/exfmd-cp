#!/usr/bin/env python3
"""
External CAM central server.

Usage:
  python /tmp/workspace/Aniisss/c-its/central_server.py --host 0.0.0.0 --port 8010
"""

import argparse
import asyncio
import json
import threading
import time
from typing import Any, Dict, List, Optional, Set

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
import uvicorn


class PoimStore:
    def __init__(self) -> None:
        self._records: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

    @staticmethod
    def _now() -> float:
        return time.time()

    def ingest_snapshot(self, payload: Dict[str, Any]) -> bool:
        if not isinstance(payload, dict):
            return False
        
        poi = payload.get('params', {}).get('poi')
        if not isinstance(poi, dict):
            return False

        key = poi.get('id')
        if not key:
            return False
        
        changed = False
        with self._lock:
            record = dict(poi)
            record['updated_at'] = self._now()
            self._records[str(key)] = record
            changed = True
        return changed

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            pois = [dict(item) for item in self._records.values()]
        return {'timestamp': self._now(), 'pois': pois}


class CamStore:
    def __init__(self) -> None:
        self._records: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

    @staticmethod
    def _now() -> float:
        return time.time()

    @staticmethod
    def _record_key(record: Dict[str, Any], index: int) -> str:
        for key in ('station_id', 'vehicle_id', 'source_topic'):
            value = record.get(key)
            if value is not None:
                text = str(value).strip()
                if text:
                    return f'{key}:{text}'
        return f'unknown:{index}'

    def ingest_snapshot(self, payload: Dict[str, Any]) -> bool:
        cams = payload.get('cams')
        if not isinstance(cams, list):
            return False
        changed = False
        with self._lock:
            for index, item in enumerate(cams):
                if not isinstance(item, dict):
                    continue
                record = dict(item)
                record['updated_at'] = float(record.get('updated_at', self._now()))
                key = self._record_key(record, index)
                self._records[key] = record
                changed = True
        return changed

    def prune_stale(self, ttl_seconds: float) -> bool:
        now = self._now()
        changed = False
        with self._lock:
            stale_keys = [
                key for key, item in self._records.items()
                if (now - float(item.get('updated_at', 0.0))) > ttl_seconds
            ]
            for key in stale_keys:
                del self._records[key]
                changed = True
        return changed

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            cams = [dict(item) for item in self._records.values()]
        return {'timestamp': self._now(), 'cams': cams}


class WebSocketHub:
    def __init__(self) -> None:
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._clients: Set[WebSocket] = set()
        self._lock = threading.Lock()

    def set_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        with self._lock:
            self._clients.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        with self._lock:
            self._clients.discard(websocket)

    async def broadcast(self, payload: Dict[str, Any]) -> None:
        with self._lock:
            clients = list(self._clients)
        dead_clients: List[WebSocket] = []
        for ws in clients:
            try:
                await ws.send_json(payload)
            except Exception:
                dead_clients.append(ws)
        if dead_clients:
            with self._lock:
                for ws in dead_clients:
                    self._clients.discard(ws)

    def broadcast_threadsafe(self, payload: Dict[str, Any]) -> None:
        if self._loop is None or self._loop.is_closed():
            return
        asyncio.run_coroutine_threadsafe(self.broadcast(payload), self._loop)


def create_app(ttl_seconds: float, cleanup_period_seconds: float) -> FastAPI:
    app = FastAPI(title='CAM Central Server', version='1.0')
    cam_store = CamStore()
    poim_store = PoimStore()
    cam_ws_hub = WebSocketHub()
    poim_ws_hub = WebSocketHub()
    cleanup_task: Optional[asyncio.Task] = None

    async def _broadcast_cam_state_if_changed(changed: bool) -> None:
        if changed:
            await cam_ws_hub.broadcast(cam_store.snapshot())

    async def _cleanup_loop() -> None:
        while True:
            await asyncio.sleep(cleanup_period_seconds)
            if cam_store.prune_stale(ttl_seconds):
                await cam_ws_hub.broadcast(cam_store.snapshot())

    @app.on_event('startup')
    async def _on_startup() -> None:
        nonlocal cleanup_task
        loop = asyncio.get_running_loop()
        cam_ws_hub.set_loop(loop)
        poim_ws_hub.set_loop(loop)
        cleanup_task = asyncio.create_task(_cleanup_loop())

    @app.on_event('shutdown')
    async def _on_shutdown() -> None:
        if cleanup_task is not None:
            cleanup_task.cancel()
            try:
                await cleanup_task
            except asyncio.CancelledError:
                pass

    @app.get('/cam/state')
    async def cam_state() -> Dict[str, Any]:
        return cam_store.snapshot()

    @app.post('/cam/ingest')
    async def cam_ingest(payload: Dict[str, Any]) -> Dict[str, Any]:
        changed = cam_store.ingest_snapshot(payload)
        if not changed:
            raise HTTPException(status_code=400, detail='Payload must contain cams list')
        await _broadcast_cam_state_if_changed(changed)
        return {'status': 'ok'}

    @app.websocket('/ws/cam')
    async def ws_cam(websocket: WebSocket) -> None:
        await cam_ws_hub.connect(websocket)
        await websocket.send_json(cam_store.snapshot())
        try:
            while True:
                _ = await websocket.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            await cam_ws_hub.disconnect(websocket)

    @app.websocket('/ws/ingest')
    async def ws_ingest(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    payload = json.loads(data)
                except json.JSONDecodeError:
                    continue
                changed = cam_store.ingest_snapshot(payload)
                if changed:
                    await cam_ws_hub.broadcast(cam_store.snapshot())
        except WebSocketDisconnect:
            pass

    @app.websocket('/ws/ingest_poim')
    async def ws_ingest_poim(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    payload = json.loads(data)
                except json.JSONDecodeError:
                    continue
                changed = poim_store.ingest_snapshot(payload)
                if changed:
                    await poim_ws_hub.broadcast(poim_store.snapshot())
        except WebSocketDisconnect:
            pass

    @app.websocket('/ws/poim')
    async def ws_poim(websocket: WebSocket) -> None:
        await poim_ws_hub.connect(websocket)
        await websocket.send_json(poim_store.snapshot())
        try:
            while True:
                _ = await websocket.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            await poim_ws_hub.disconnect(websocket)

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description='CAM central server')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8010)
    parser.add_argument('--ttl-seconds', type=float, default=7.0)
    parser.add_argument('--cleanup-period-seconds', type=float, default=1.0)
    parser.add_argument('--log-level', default='warning')
    parser.add_argument('--proxy-headers', action='store_true', help='Trust reverse proxy forwarded headers')
    parser.add_argument('--forwarded-allow-ips', default='127.0.0.1', help='Allowed proxy IPs or "*"')
    args = parser.parse_args()

    app = create_app(args.ttl_seconds, args.cleanup_period_seconds)
    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level=args.log_level,
        proxy_headers=args.proxy_headers,
        forwarded_allow_ips=args.forwarded_allow_ips,
    )


if __name__ == '__main__':
    main()
