"""Append only a child's new pair segments; inherit immutable parent indexes."""
from pathlib import Path
import immutable_state_access as a
from incremental_pair_storage import build_incidence
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity

def append_incidence(parent_index,child,directory):
    parent=parent_index.body
    if child.kind!='recursive' or child.parent.content_root!=parent['parent_content']:raise ValueError('WrongIncidenceAncestry')
    all_segments=list(child.proof_segments());prefix=len(parent['segments'])
    if [identity(d) for d,_,_ in all_segments[:prefix]]!=[s['identity'] for s in parent['segments']]:raise ValueError('ChangedInheritedSegment')
    new=all_segments[prefix:];new_count=sum(d['count'] for d,_,_ in new)
    if parent['parent_count']+new_count!=len(child['certificate']):raise ValueError('UnqualifiedPairRemovalOrReplacement')
    # This descriptor is only a private index-build input, never a committed state.
    class SegmentBuildInput:
        content_root=identity(dict(new_segments=[d for d,_,_ in new]))
        def proof_segments(self):return iter(new)
        def __getitem__(self,key):
            if key!='certificate':raise KeyError(key)
            return range(new_count)
    directory=Path(directory);root=build_incidence(SegmentBuildInput(),directory/'new_segments');added=a.read(directory/'new_segments/MANIFEST.json')
    merged=dict(added,parent_content=child.content_root,parent_count=len(child['certificate']),segments=parent['segments']+added['segments'],files={**parent['files'],**added['files']},inherited_index_root=parent_index.root,append_implementation=file_hash(__file__))
    durable_json(directory/'MANIFEST.json',merged)
    return dict(root=identity(merged),inherited_segments=prefix,new_segments=len(new),prior_pair_rows=0,new_pair_rows=new_count,private_new_segment_root=root)
