"""HTTP worker functions; never call ROS from the inference thread."""
import base64
import io
import json
import time
import numpy as np
from PIL import Image
import requests
from .core import parse_actions


def image_rgb(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if msg.encoding not in channels:
        raise ValueError(f'Unsupported image encoding {msg.encoding}')
    c = channels[msg.encoding]
    if msg.width < 1 or msg.height < 1 or msg.step < msg.width*c:
        raise ValueError('Invalid image dimensions or step')
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if len(raw) != msg.height*msg.step:
        raise ValueError('Image byte count does not match height*step')
    pixels = raw.reshape(msg.height, msg.step)[:, :msg.width*c].reshape(msg.height, msg.width, c)
    if msg.encoding == 'mono8':
        return np.repeat(pixels, 3, axis=2).copy()
    if msg.encoding in ('bgr8', 'bgra8'):
        return pixels[:, :, :3][:, :, ::-1].copy()
    return pixels[:, :, :3].copy()


def jpeg_b64(msg, size):
    image = Image.fromarray(image_rgb(msg)).resize((size, size), getattr(Image, 'Resampling', Image).BILINEAR)
    out = io.BytesIO()
    image.save(out, format='JPEG', quality=90)
    return base64.b64encode(out.getvalue()).decode('ascii')


def infer(url, overview, wrist, state, instruction, size, timeout, max_chunk):
    payload = {'image_b64': jpeg_b64(overview, size),
               'wrist_image_b64': jpeg_b64(wrist, size), 'state': state,
               'instruction': instruction, 'return_chunk': True}
    deadline = time.monotonic()+timeout
    with requests.post(url, json=payload, timeout=(min(timeout, 2.0), timeout),
                       allow_redirects=False, stream=True) as response:
        if response.status_code != 200:
            raise ValueError(f'Inference HTTP status {response.status_code}')
        body = bytearray()
        for block in response.iter_content(8192):
            if time.monotonic() > deadline:
                raise TimeoutError('Inference response exceeded total deadline')
            body.extend(block)
            if len(body) > 1024*1024:
                raise ValueError('Inference response exceeds 1 MiB')
        return parse_actions(json.loads(body), max_chunk)
