"""Opt-in production pair session using the exact representation adapter."""
import json
from . import repaired_kernel as kernel
from .runtime import CertificateSession as FrozenSession,canonical_bytes

class CertificateSession(FrozenSession):
    def pair(self,a,b):
        authorities=tuple(sorted(((a.points,a.ids,a.declared_dimension),(b.points,b.ids,b.declared_dimension)),key=str))
        key=(kernel.VERSION,authorities)
        if self.capacity and key in self.cache:
            self.hits+=1;value=self.cache.pop(key);self.cache[key]=value
            return json.loads(value)
        self.misses+=1;result=kernel.certificate.certify(a,b)
        if self.capacity:
            self.cache[key]=canonical_bytes(result)
            if len(self.cache)>self.capacity:self.cache.popitem(last=False)
        return result
