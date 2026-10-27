"""Bounded, independent core-side face closures for mixed-shell planning.

Each triangular cap gets one affine tetrahedral neighbour. Its centroid is the
reflection of the shell centroid in the face centroid. This supplies a local
feasibility stencil, not a joined core mesh or a final core-quality receipt.
Final core centres/volumes must be validated again after actual core meshing.
The coupled determinant is already exact in its inputs: only face incidence
changes. No provisional neighbour coordinate enters that determinant.
"""
import sys
sys.dont_write_bytecode=True
import time
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
from pathlib import Path


def canonical_cell(points,cell):
    """Normalize cyclic face storage before constructing diagnostic geometry.

    Coordinates and orientation are unchanged. This makes floating-point
    reduction order independent of checkpoint face sorting/local numbering.
    The qualified metric implementation itself is unchanged.
    """
    def coordinates(face):return tuple(tuple(map(float,points[v])) for v in face)
    faces=[]
    for original in cell['faces']:
        face=tuple(original)
        faces.append(min((face[i:]+face[:i] for i in range(len(face))),key=coordinates))
    return dict(cell,faces=sorted(faces,key=coordinates))


def check(points,cells,outer,details=False):
    tick=time.perf_counter()
    mixed=[c for c in cells if c['kind']!='prism']
    if not mixed:return dict(passed=True,failures=[],closures=0,scope='AllPrismUnchanged')
    cells=[canonical_cell(points,c) for c in sorted(cells,key=lambda c:c['id'])]
    mixed=[c for c in cells if c['kind']!='prism']
    outer_keys={tuple(sorted(f)) for f in outer}
    from collections import defaultdict
    owners=defaultdict(list)
    for c in cells:
        for f in c['faces']:owners[tuple(sorted(f))].append((c['id'],tuple(f)))
    p=list(points);ghosts=[];stencils=[];next_id=max(c['id'] for c in cells)+1
    for c in sorted(mixed,key=lambda c:c['id']):
        geo=g.t.geometry(points,c)
        for j,f in enumerate(c['faces']):
            if tuple(sorted(f)) not in outer_keys:continue
            if len(f)!=3:raise ValueError('NonTriangularClosureNeedsQualifiedTemplate')
            centre=geo['face_centres'][j]
            # For a tet, C=(3*faceCentre+apex)/4. Set C=2*faceCentre-ownerC.
            apex=5*centre-4*geo['centre'];k=len(p);p.append(apex)
            a,b,d=map(int,f);faces=[(a,d,b),(a,b,k),(b,d,k),(d,a,k)]
            ghost=dict(id=next_id,column=c['column'],layer=0,kind='tetrahedron',
                       vertices=[a,b,d,k],faces=faces)
            next_id+=1;ghosts.append(ghost)
            for face in faces:owners[tuple(sorted(face))].append((ghost['id'],face))
            stencils.append(dict(shell_cell=c['id'],source=c['column'],face=list(f),
                coordinates=[g.np.asarray(p[v]).tolist() for v in (a,b,d,k)],
                topology=[[0,2,1],[0,1,3],[1,2,3],[2,0,3]]))
    p=g.np.asarray(p)
    # The existing complete-neighbour implementation supplies every metric.
    quality=g.t.quality(p,cells+ghosts,owners,1)
    ids={c['id'] for c in mixed};ghost_ids={c['id'] for c in ghosts}
    failures=[z for z in quality['failures'] if z.get('id') in ids or any(i in ghost_ids for i in z.get('owners',[]))]
    exact=[g.t.exact_cell(p,c) for c in ghosts]
    for c,z in zip(ghosts,exact):
        if z['jacobian_sign']<=0 or z['volume_sign']<=0:
            failures.append(dict(kind='closure_exact',column=c['column'],detail=z))
    result=dict(passed=not failures,failures=failures,closures=len(ghosts),
        cells=[z for z in quality['cells'] if z['id'] in ids],
        faces=[z for z in quality['faces'] if any(i in ghost_ids for i in z['owners'])],
        closure_identity=identity(sorted([dict(shell_cell=s['shell_cell'],
            face=sorted(s['coordinates'][:3]),apex=s['coordinates'][3]) for s in stencils],
            key=lambda s:(s['shell_cell'],s['face']))),
        metric_version=file_hash(Path(g.t.__file__)),closure_version=file_hash(Path(__file__)),
        scope='Independent affine face-neighbour feasibility closures; not a conformal common core. Final core face metrics require new validation.',
        determinant_scope='Final coupled face incidence and unchanged shell areas; independent of provisional core coordinates')
    # This object enters transaction proof identity. Timing belongs in a
    # separate diagnostic record, never in a scientific validation receipt.
    if details:result.update(stencils=stencils,closure_exact=exact)
    return result


def certify_stencils(points,cells,check):
    """Use unchanged exact face predicates for each independent owner/closure pair."""
    byid={c['id']:c for c in cells};required=0;hashes=[]
    for stencil in check.get('stencils',[]):
        owner=byid[stencil['shell_cell']]
        vertices=sorted(set(owner['vertices']));lookup={v:i for i,v in enumerate(vertices)}
        p=[points[v] for v in vertices]+[stencil['coordinates'][3]];apex=len(vertices)
        face=[lookup[v] for v in stencil['face']];a,b,c=face
        faces=[tuple(lookup[v] for v in f) for f in owner['faces']]
        faces.extend([(a,b,apex),(b,c,apex),(c,a,apex)])
        certificate=g.s.cert.certify(g.np.asarray(p),faces,range(len(faces)-3,len(faces)))
        if not certificate['passed']:raise ValueError('InvalidIndependentClosureGeometry')
        required+=certificate['required_pairs'];hashes.append(certificate['proof_hash'])
    return dict(passed=True,required_relations=required,proof_identity=identity(hashes),
                scope='Independent diagnostic closure stars; excluded from committed shell/core certificate')


def qualify_closures(folder,layer,points,cells,checked):
    """Bind generated local closure geometry before any shell publication."""
    if 'complete_neighbor' not in checked:return
    import subprocess,json
    tick=time.perf_counter()
    coupled=checked['complete_neighbor'];path=folder/f'COUPLED_{layer}.json'
    g.save(path,coupled)
    def wsl(p):
        p=p.resolve();return '/mnt/'+p.drive[0].lower()+str(p)[2:].replace('\\','/')
    subprocess.run(['wsl','--','python3','-B',wsl(Path(__file__).with_name('coupled_closure_gmsh.py')),wsl(path)],
                   check=True,capture_output=True)
    generated=json.loads(path.with_name(path.stem+'_GMSH.json').read_text())
    if not generated['passed'] or generated['closure_identity']!=coupled['closure_identity']:
        raise ValueError('UnboundProvisionalClosure')
    coupled['exact_closure']=certify_stencils(points,cells,coupled)
    g.save(path,coupled)
    g.save(folder/f'COUPLED_{layer}_PERFORMANCE.json',dict(seconds=time.perf_counter()-tick,
        generated_seconds=generated['seconds'],closures=coupled['closures']))
