"""SHA-256 streaming compatibility for the installed Python 3.10 transport."""
import hashlib


def file_digest(stream,digest):
    result=hashlib.new(digest) if isinstance(digest,str) else digest()
    while True:
        block=stream.read(1024*1024)
        if not block:return result
        result.update(block)


def install():
    if not hasattr(hashlib,'file_digest'):hashlib.file_digest=file_digest
