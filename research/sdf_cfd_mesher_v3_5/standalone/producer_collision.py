"""Use the existing guarded cold batch for complete producer populations.

The original streamed certifier remains the oracle and unsupported dispatch.
Every proof, legacy digest, independent coverage and retained readback stays.
"""
from contextlib import contextmanager, nullcontext
import threading


@contextmanager
def complete_batch():
    import native_mixed_face_certificate as reference
    from reuse_mixed_certificate import _cold_native_inputs
    from optimized_mixed_certificate import certify as batch
    from sdf_cfd_mesher_v4.common.authority import frozen
    from unittest.mock import patch
    original=reference.certify
    if original is not reference._STANDARD_CERTIFY:
        yield
        return
    thread=threading.get_ident()

    def dispatch(points,faces,active,*,workers=4):
        if reference.certify is not dispatch:
            raise ValueError('ChangedProducerCollisionBinding')
        if (threading.get_ident()!=thread or type(workers) is not int or workers!=4 or
                type(faces) is not list or type(active) is not list):
            return original(points,faces,active,workers=workers)
        complete=(len(active)==len(faces) and
            all(type(value) is int and value==index for index,value in enumerate(active)))
        if not complete:
            return original(points,faces,active,workers=workers)
        polygons=tuple(faces)
        if not _cold_native_inputs(points,polygons):
            return original(points,faces,active,workers=workers)
        # Same constructor/refusal gate and private operands as the qualified
        # cold helper. No old proof, relation or acceptance value is imported.
        reference.Filter()
        metadata=(points.shape,points.dtype.str,points.strides)
        private=frozen(points,'<f8')
        result=batch(private,polygons,range(len(polygons)),workers=4,calculate_digest=True)
        if (reference.certify is not dispatch or
                (points.shape,points.dtype.str,points.strides)!=metadata or
                points.tobytes()!=private.tobytes() or tuple(faces)!=polygons or
                not _cold_native_inputs(points,tuple(faces)) or
                type(active) is not list or len(active)!=len(polygons) or
                any(type(value) is not int or value!=index for index,value in enumerate(active))):
            raise ValueError('ChangedProducerCollisionInputs')
        result.pop('optimization')
        return result

    with patch.object(reference,'certify',dispatch):
        try:
            yield
        finally:
            if reference.certify is not dispatch:
                raise ValueError('ChangedProducerCollisionBinding')


def for_mode(mode):
    if mode=='production':return complete_batch()
    if mode=='audit':return nullcontext()
    raise ValueError('UnknownVerificationMode')
