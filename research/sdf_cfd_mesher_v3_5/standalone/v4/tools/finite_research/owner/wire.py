"""Small binary framing for the isolated native process transport."""
import struct

MAX_CHUNK = 65536


def exact(stream, count):
    parts = []
    while count:
        data = stream.read(count)
        if not data: raise EOFError('NativeStreamClosed')
        parts.append(data); count -= len(data)
    return b''.join(parts)


def receive(stream):
    kind, count = struct.unpack('!cI', exact(stream, 5))
    if count > MAX_CHUNK: raise ValueError('NativeFrameTooLarge')
    return kind, exact(stream, count)


def send(channel, kind, data=b''):
    if len(data) > MAX_CHUNK: raise ValueError('NativeFrameTooLarge')
    channel.sendall(struct.pack('!cI', kind, len(data))+data)
