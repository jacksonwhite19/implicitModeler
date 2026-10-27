"""Joined-background certificate; retain every proof, omit dead digest only.

The default generic/capsule certifier still calculates its canonical digest.
No record, predicate, exact fallback or independent-coverage obligation changes.
"""
from functools import partial
from optimized_mixed_certificate import certify as existing_certify, PackedRows
from background_screen import Native
from background_summary import summary

class Rows(PackedRows):
    def __init__(self,rows=(),*,native_factory=None,_preparation=None):
        super().__init__(rows,native_factory=native_factory)
        if _preparation is not None:
            from background_preparation import require
            require(_preparation)
        self._preparation=_preparation

    def append_classified(self,*args,**kwargs):
        return super().append_classified(*args,**kwargs,_preparation=self._preparation)

    def summary(self):return summary(self)

def certify(points,faces,active,*,workers=8,directory=None,calculate_digest=False,_preparation=None):
    factory=Native if directory is None else partial(Native,directory=directory)
    return existing_certify(points,faces,active,workers=workers,native_factory=factory,
                            row_factory=partial(Rows,_preparation=_preparation),calculate_digest=calculate_digest)
