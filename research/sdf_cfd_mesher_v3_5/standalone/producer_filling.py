"""Producer-only exact degree engine; the independent reader keeps its oracle."""
def filling(state):
    from background_topology_repair import boundary
    from cavity_volume_audit import audit
    from prism_degree import serial
    _,outside,_=boundary(state['cells'])
    return audit(state['points'],state['cells'],outside,[],_degree=serial)


def for_mode(mode):
    if mode=='production':return filling
    if mode=='audit':return None
    raise ValueError('UnknownVerificationMode')
