"""Byte-identical indented JSON with the existing fsync and atomic rename."""
import ctypes
import json
import os
from pathlib import Path
from optimized_mixed_certificate import Native, Bytes
from path_preserving_deduplication import atomic_replace


def encoded(value):
    # The standard C encoder retains the original sorting, number and escape
    # semantics. The native pass adds only the original indent=2 whitespace.
    compact=json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf8')
    native=Native();native.dll.indent_json.argtypes=[ctypes.c_void_p,ctypes.c_size_t]
    native.dll.indent_json.restype=Bytes
    result=native.dll.indent_json(ctypes.c_char_p(compact),len(compact))
    try:return ctypes.string_at(result.data,result.length)
    finally:native.dll.bytes_free(result)


def durable_json(path,value):
    path=Path(path);pending=path.with_name(path.name+'.pending')
    data=encoded(value)
    with pending.open('wb') as stream:
        stream.write(data);stream.flush();os.fsync(stream.fileno())
    atomic_replace(pending,path)
