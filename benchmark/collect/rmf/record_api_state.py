#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

app = FastAPI()
OUT = {'path': None}


@app.websocket('/')
async def sink(ws: WebSocket):
    await ws.accept()
    peer = f'{ws.client.host}:{ws.client.port}' if ws.client else '?'
    _write({'_event': 'connected', '_peer': peer})
    try:
        while True:
            raw = await ws.receive_text()
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                payload = {'_unparsed': raw}
            _write({'_peer': peer, 'msg': payload})
    except WebSocketDisconnect:
        _write({'_event': 'disconnected', '_peer': peer})



def _write(obj):
    obj['_recv_wall'] = time.time()

    try:
        with open(OUT['path'], 'a', encoding='utf-8') as f:
            f.write(json.dumps(obj, default=str) + '\n')
            f.flush()
    except OSError:
        import logging
        logging.exception(
            'Failed to write RMF API state to %s',
            OUT['path']
        )
        raise


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='Where to write the JSONL output')
    ap.add_argument('--host', default='0.0.0.0',
                    help='Address to listen on for the fleet adapter and dispatcher')
    ap.add_argument('--port', type=int, default=8000,
                    help='Port named in RMF server_uri')
    args = ap.parse_args()
    output_path = Path(args.out).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('a', encoding='utf-8'):
        pass
    OUT['path'] = str(output_path)

    uvicorn.run(app, host=args.host, port=args.port, ws='wsproto', log_level='warning')
