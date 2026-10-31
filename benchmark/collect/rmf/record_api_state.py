#!/usr/bin/env python3

import argparse
import json
import time

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
    with open(OUT['path'], 'a') as f:
        f.write(json.dumps(obj, default=str) + '\n')
        f.flush()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='Where to write the JSONL output')
    ap.add_argument('--host', default='0.0.0.0',
                    help='Address to listen on for the fleet adapter and dispatcher')
    ap.add_argument('--port', type=int, default=8000,
                    help='Port named in RMF server_uri')
    args = ap.parse_args()
    OUT['path'] = args.out
    open(args.out, 'a').close()

    import uvicorn
    uvicorn.run(app, host=args.host, port=args.port, ws='wsproto', log_level='warning')
