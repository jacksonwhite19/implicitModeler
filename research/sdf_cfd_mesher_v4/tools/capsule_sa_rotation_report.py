"""Replay compact decision tables from sealed sampled representations."""
import json
import hashlib
import subprocess
import numpy as np
import capsule_sa_rotation as r
import capsule_sa_rotation_checks as checks
c=r.c;h=r.h

def supplement():
    grid=np.load(r.ROOT/'SAMPLING.npz');groups=grid['groups'];weights=grid['weights']
    result=dict(regional_sampling={},wake_modal_evolution={},wall_modal_evolution={},wall_filtered_comparisons={},quotient={},source_field_extrema={})
    analysis=h.read(r.ROOT/'ANALYSIS.json')
    for name in c.NAMES:
        result['regional_sampling'][name]={};result['wake_modal_evolution'][name]={};result['quotient'][name]={}
        result['wall_modal_evolution'][name]={};result['wall_filtered_comparisons'][name]={}
        result['source_field_extrema'][name]=h.read(c.ROOT/name/'MEASUREMENTS.json')['checkpoints']
        for step in r.STEPS:
            d=np.load(r.ROOT/(name+'_'+str(step)+'.npz'));angle=-float(d['force_angle'])
            result['regional_sampling'][name][str(step)]={};result['wake_modal_evolution'][name][str(step)]={}
            for group in dict.fromkeys(groups):
                mask=groups==group;w=weights[mask]
                result['regional_sampling'][name][str(step)][group]={f:r.difference(d[f+'_aligned'][mask],r.shift(d[f][mask],angle),w) for f in r.FIELDS}
                result['wake_modal_evolution'][name][str(step)][group]={f:r.modes(d[f][mask],w) for f in r.FIELDS}
            wall=np.load(r.ROOT/(name+'_wall_'+str(step)+'.npz'))
            result['wall_modal_evolution'][name][str(step)]={f:r.modes(wall[f],np.ones(64)) for f in ('p','shear')}
        for i,j in [(600,700),(700,800),(800,900),(900,1000),(600,1000),(800,1000)]:
            pair=f'{i}-{j}';result['quotient'][name][pair]={region:{f:v['optimal'] for f,v in fields.items()} for region,fields in analysis['runs'][name]['pairs'][pair].items()}
        for i,j in [(800,900),(900,1000),(800,1000)]:
            a=np.load(r.ROOT/(name+'_wall_'+str(i)+'.npz'));b=np.load(r.ROOT/(name+'_wall_'+str(j)+'.npz'))
            aa=np.load(r.ROOT/(name+'_'+str(i)+'.npz'));bb=np.load(r.ROOT/(name+'_'+str(j)+'.npz'))
            angle=float(bb['force_angle']-aa['force_angle'])
            result['wall_filtered_comparisons'][name][f'{i}-{j}']={f:{str(m):r.pair(checks.lowpass(a[f],m),checks.lowpass(b[f],m),np.ones(64),angle) for m in (1,2,4,8,16)} for f in ('p','shear')}
    h.durable_json(r.ROOT/'SUPPLEMENT.json',h.seal(result))

def report():
    a=h.read(r.ROOT/'ANALYSIS.json');sens=h.read(r.ROOT/'SENSITIVITY.json');wall=h.read(r.ROOT/'WALL_ANALYSIS.json');sup=h.read(r.ROOT/'SUPPLEMENT.json');ad=h.read(r.ROOT/'ADMISSION.json')
    lines=[]
    def p(x=''):lines.append(str(x))
    def table(headers,rows):
        p()
        p('| '+' | '.join(headers)+' |');p('| '+' | '.join(['---']*len(headers))+' |')
        for row in rows:p('| '+' | '.join(f'{x:.9g}' if isinstance(x,(float,np.floating)) else str(x) for x in row)+' |')
        p()
    p('# Retained steady-SA rotational-mode diagnosis')
    p('Classification: V4CapsuleSteadySARotationAlignmentReducesButDoesNotEliminateLateEvolution')
    p('Outcome B. Large-scale wake change is predominantly rotational. Complete steady equivalence under rotation is not established. This is an analysis-only, non-qualifying result. No stopping contract or production rule is issued.')
    p('Starting HEAD: '+r.HEAD)
    p('The body and inflow have continuous axial symmetry. The finite square outer box and discrete tetra/prism triangulation do not. Arbitrary axial rotation is therefore an inner-flow diagnostic, not an exact symmetry of the entire discrete boundary-value problem.')
    p('## Authority and immutability')
    p(json.dumps(ad,indent=2))
    p('All prior result evidence and implementation hashes, retained rank checkpoints and canonical internal arrays were verified before extraction. End verification repeats the same checks. No source field, control, mesh or decomposition was edited. Native wall post-processing ran only on ordinary scratch copies; every pre-existing scratch file remained byte-identical.')
    p('## Sampling and operators')
    p('Six disks at s/D=0.5,1,2,4,6,8; 32 annular midpoint radii (j+0.5)*0.0025 m; 256 azimuths, theta=2*pi*j/256. Total wake samples=49152. Body-surrounding volume: 25 z planes from -0.12 to 0.12 m, 12 radii from 0.045 to 0.155 m, 256 azimuths; 76800 points. Total=125952. Disk weights are exact annular areas; body weights are cylindrical midpoint volume weights. Body and wake norms are separate. This is not a whole-domain norm and does not cover all prism interiors.')
    p('Wall: 64 midpoint z positions over [-0.08,0.08], 256 azimuths; exact capsule radius sqrt(0.04^2-max(abs(z)-0.04,0)^2). Equal z/azimuth weights give equal exact capsule surface areas. Wall values are sampled from actual wall-face values, not triangle-ID comparisons.')
    p('Primary replay: eight nearest cell-centre values, positive inverse-square distance weights. Cell centres are the existing vertex-mean canonical centres (exact for tetrahedra; approximate for warped prisms). Sensitivity: 24-neighbour weighted affine least-squares reconstruction, reproducing linear functions. It can extrapolate and is not a positivity-preserving field. Original fields remain positive. Wall replay uses eight face-centre neighbours. No claimed interpolation error bound; method spread is a diagnostic.')
    p('Active rotation R(a) about +Z. Canonical angle a=-atan2(Fy,Fx). Query x_original=R(-a)*x_canonical; vector result=R(a)*U_original; scalars unchanged. No translation and no pressure offset fit. In cylindrical components active field rotation is q(theta-a). Fourier shifting operates on sampled rings, never raw cell indices. Direct rotated-query checks and a 128-versus-256-angle replay quantify representation sensitivity.')
    p('L2 is weighted RMS, sqrt(sum(w*|difference|^2)/sum(w)); Linf is maximum vector magnitude or scalar absolute difference. U normalization=33.528 m/s; kinematic p normalization=0.5*33.528^2 m^2/s^2; SA scalar normalization=measured reference-region RMS. Dimensional norms are retained. No combined multi-field score or arbitrary relative weights.')
    p('## Orientation and rotation rates')
    for name,run in a['runs'].items():
        p(name)
        table(['iteration','force principal deg','canonical rotation deg','moment principal deg','moment-force deg'],[(step,((run['states'][str(step)]['force_angle_deg']+180)%360-180),-((run['states'][str(step)]['force_angle_deg']+180)%360-180),((run['states'][str(step)]['moment_angle_deg']+180)%360-180),run['states'][str(step)]['force_moment_offset_deg']) for step in r.STEPS])
        table(['estimator/window','slope deg/iteration','mean increment','increment std','quadratic acceleration'],[(f'{f}/{win}',v['slope'],v['mean_first_difference'],v['rate_std'],v['quadratic_acceleration']) for f,windows in run['rates'].items() for win,v in windows.items()])
        p('Rates are per SIMPLE outer iteration, never physical angular velocity. Wake rates below are checkpoint secants only; no intervening wake fields were invented.')
        table(['station','800 angle','900 angle','1000 angle','800-900 deg/it','900-1000 deg/it','1000 force offset'],[(g,run['states']['800']['regions'][g]['wake']['centroid_angle_deg'],run['states']['900']['regions'][g]['wake']['centroid_angle_deg'],run['states']['1000']['regions'][g]['wake']['centroid_angle_deg'],(run['states']['900']['regions'][g]['wake']['centroid_angle_deg']-run['states']['800']['regions'][g]['wake']['centroid_angle_deg'])/100,(run['states']['1000']['regions'][g]['wake']['centroid_angle_deg']-run['states']['900']['regions'][g]['wake']['centroid_angle_deg'])/100,run['states']['1000']['regions'][g]['wake']['force_offset_deg']) for g in run['states']['1000']['regions'] if g.startswith('wake')])
    p('The first deficit moment and deficit centroid have exactly the same phase by definition and are not independent confirmations. Moment orientation is approximately +89.4 degrees from force. Wake deficit centroid is approximately opposite force, with a station-dependent phase lag. Coherent Fourier phase convention: q1=mean(q*exp(-i*theta)); a positive cosine lobe has orientation -arg(q1).')
    p('## Field differences and optimal rotations')
    for name,pairs in sens['comparisons'].items():
        p(name+'; 24-neighbour linear reconstruction')
        table(['pair/region/field','raw L2','aligned L2','raw Linf','aligned Linf','L2 ratio','optimal angle deg','optimal L2','normalized aligned L2'],[(key+'/'+region+'/'+f,v['raw']['L2'],v['force_aligned']['L2'],v['raw']['Linf'],v['force_aligned']['Linf'],v['ratio'],v['optimal']['angle_deg'],v['optimal']['L2'],v['normalized_aligned_L2']) for key,regions in pairs.items() for region,fields in regions.items() for f,v in fields.items()])
    p('The optimizer brackets negative-to-positive objective-derivative crossings across the full 256-angle period, then refines each bracket with Brent root finding (angular tolerance 1e-13 rad, bookkeeping only). The constant Fourier term is excluded from the objective. The even-grid Nyquist mode is omitted from rotations because its sine partner is not representable; no resolved m=0..127 mode is discarded. The 128-angle replay checks angular resolution. Each field is minimized separately, not in a tuned aggregate.')
    p('## Raw finite-change projection')
    p('g=[R(epsilon)q-R(-epsilon)q]/(2*epsilon), epsilon=2*pi/256; half-step replay retained. In Cartesian form the active vector generator is J*U-(-y*dU/dx+x*dU/dy); the active scalar generator is the negative of the coordinate angular derivative. This sign convention is verified analytically. alpha=<delta,g>/<g,g>; explained fraction=||alpha*g||^2/||delta||^2. A 30-36 degree checkpoint rotation is not infinitesimal, so this projection cannot equal the finite-rotation fit even for a pure rotating pattern.')
    for name in c.NAMES:
        p(name+' 900-1000')
        table(['region/field','alpha rad','total change','rotational norm','remainder norm','fraction','half-step generator difference'],[(region+'/'+f,v['projection']['alpha_rad'],v['projection']['total'],v['projection']['projected'],v['projection']['nonrotational'],v['projection']['explained_squared_fraction'],v['projection']['generator_half_step_relative_difference']) for region,fields in sens['comparisons'][name]['900-1000'].items() for f,v in fields.items()])
        table(['wake field','m<=1','m<=2','m<=4','m<=8','m<=16'],[(f,*[bands[str(m)]['explained_squared_fraction'] for m in (1,2,4,8,16)]) for f,bands in sens['mode_projection'][name]['900-1000']['wake'].items()])
    p('Low-pass projections diagnose scale dependence; they do not replace the full-field result. Small-scale interpolation/discrete-mesh roughness strongly affects derivatives. No neutral eigenvalue is calculated.')
    p('## Modes and wake invariants')
    for name,run in a['runs'].items():
        p(name)
        table(['iteration/station','deficit integral m3/s','width m','centroid magnitude m','centroid angle deg'],[(str(step)+'/'+g,v['wake']['deficit_integral'],v['wake']['rms_width'],v['wake']['centroid_magnitude'],v['wake']['centroid_angle_deg']) for step in r.STEPS for g,v in run['states'][str(step)]['regions'].items() if 'wake' in v])
        table(['iteration/station/field','m0 RMS','m1 RMS','m2 RMS','m1 coherent phase deg','dominant nonaxisymmetric m'],[(str(step)+'/'+g+'/'+f,m['0']['rms_coefficient'],m['1']['rms_coefficient'],m['2']['rms_coefficient'],m['1']['coherent_phase_deg'],m['dominant_nonaxisymmetric_mode']) for step in (800,900,1000) for g,v in run['states'][str(step)]['regions'].items() if g in ('wake_0.5','wake_2','wake_8') for f,m in v['modes'].items()])
    p('m amplitudes are RMS complex Fourier coefficients over rings (and cylindrical vector components). A real cosine amplitude is twice its complex coefficient. Vector phases are listed radial, azimuthal, axial; a small coherent coefficient can have an unstable phase despite finite RMS energy. Coherent phases are not all interchangeable orientation estimators.')
    p('## Wall fields')
    p('Native Foundation wallShearStress uses minus the model devSigma boundary traction. Pressure is retained kinematic p, Cp=p/(0.5*Uinf^2); stress multiplied by rho is Pa. No solver time step was executed. Reverse shear is sign opposite the midbody median along +Z projected onto the capsule tangent; it is not a qualified separation line.')
    for name in c.NAMES:
        p(name)
        table(['pair/field','raw L2','aligned L2','ratio','optimal angle deg'],[(key+'/'+f,v['raw']['L2'],v['force_aligned']['L2'],v['ratio'],v['optimal']['angle_deg']) for key,fields in wall[name]['pairs'].items() for f,v in fields.items()])
        table(['iteration','rear p m1 amplitude','rear p lobe angle deg','rear reverse-shear area fraction'],[(step,wall[name]['states'][str(step)]['rear']['pressure_modes']['1']['rms_coefficient'],-wall[name]['states'][str(step)]['rear']['pressure_modes']['1']['coherent_phase_deg'][0],wall[name]['states'][str(step)]['rear']['reverse_shear_area_fraction']) for step in r.STEPS])
        table(['900-1000 field/band','aligned/raw L2'],[(f+'/'+m,v['ratio']) for f,bands in sup['wall_filtered_comparisons'][name]['900-1000'].items() for m,v in bands.items()])
    p('Wall pressure rotates coherently. Full wall shear does not collapse cleanly. Its high-frequency face-scale variation and fixed triangulation contributions preclude calling the whole wall state a rigidly rotating copy.')
    p('## Cross-seed iteration 1000')
    table(['region/field','raw L2','canonical L2','raw Linf','canonical Linf','ratio'],[(region+'/'+f,v['raw']['L2'],v['aligned']['L2'],v['raw']['Linf'],v['aligned']['Linf'],v['aligned']['L2']/v['raw']['L2']) for region,fields in sens['cross_seed'].items() for f,v in fields.items()])
    table(['wall field','raw L2','canonical L2','ratio'],[(f,v['raw']['L2'],v['aligned']['L2'],v['aligned']['L2']/v['raw']['L2']) for f,v in wall['cross_seed'].items()])
    p('The seeds become much closer in the wake after force alignment, but do not become numerically identical. Cross-seed aligned differences are comparable to, and sometimes exceed, within-seed aligned differences. This supports a common large-scale asymmetric family, not mesh-independent equality.')
    p('## Quotient-space distances')
    for name,run in a['runs'].items():
        p(name+'; IDW replay, dimensional best-aligned L2')
        table(['pair/region','U','p','nuTilda','nut'],[(pair+'/'+region,*[fields[f]['optimal']['L2'] for f in r.FIELDS]) for pair,regions in run['pairs'].items() if pair!='500-600' for region,fields in regions.items()])
    p('Best-aligned distances do not decrease monotonically toward zero. The last 100-step wake U distances increase slightly in both interpolation methods. Rotational alignment removes the dominant long-angle change but does not establish quotient-space convergence.')
    p('## Sampling uncertainty and unresolved evidence')
    p(json.dumps(sens['reconstruction'],indent=2))
    table(['region','max U angle difference','max p angle difference','max nuTilda angle difference','max nut angle difference'],[(region,*[max(abs(v[region][f]['optimal']['angle_deg']-v[region][f]['half_angular_grid_optimal']['angle_deg']) for pairs in sens['comparisons'].values() for v in pairs.values()) for f in r.FIELDS]) for region in ('body','wake')])
    p('The preceding table is 128-versus-256 azimuths, degrees. Small near-body SA phase estimates are less well resolved than wake orientations. This limits a rigid-rotation interpretation of every field.')
    p('## Retained loads and conservation')
    loadrows=[]
    for name in c.NAMES:
        rows=h.read(c.ROOT/name/'HISTORY.json')['rows'];last=rows[-1]
        loadrows.append((name,last['force'],last['moment'],last['pressure_force'][2],last['viscous_force'][2],float(np.linalg.norm(last['force'][:2])),float(np.linalg.norm(last['moment'][:2])),max(x['flux']['normalized'] for x in rows),last['flux']['normalized'],max(abs(x['flux']['wall_sumMag']) for x in rows)))
    table(['seed','terminal F N','terminal M Nm','pressure drag N','viscous drag N','lateral force N','lateral moment Nm','max imbalance','final imbalance','max wall sumMag'],loadrows)
    p('Conservation uses original conservative phi: S=sum(phi), M=sum(abs(phi)), inflow=(M-S)/2, outflow=(M+S)/2, imbalance=abs(S)/max(inflow,outflow). No U dot Sf reconstruction was substituted.')
    p('Direct-query versus Fourier-shift differences, nearest-cell versus IDW differences, and linear versus IDW results are retained in EXTRACTION, SUPPLEMENT and SENSITIVITY. These are method-sensitivity diagnostics, not rigorous interpolation error bars. The 24-neighbour linear result reduces much of the IDW floor; the residual cannot all be assigned to physical flow evolution. Pressure, small-scale wall shear, SA mode amplitude/phase differences and downstream phase lag remain non-rotational or discretization-dependent contributions.')
    p('DiscreteResidualProjectionNotEvaluated. Retained logs contain normalized linear-system residual norms, not cellwise nonlinear steady momentum/SA residual vectors. Reconstructing the exact discrete residual requires the precise assembled operator and source state. No residual was fabricated and no solver was advanced. Conservative phi remains the directly retained continuity evidence.')
    p('## Decision')
    p('A near-neutral azimuthal interpretation is supported for the large-scale wake/load structure. A mathematically neutral mode is not proved: no Jacobian eigenvalue or exact steady-residual projection is available, and the square domain/discrete mesh break continuous symmetry. An otherwise approximately stable asymmetric large-scale pattern is supported. A complete steady state modulo rotation is not established. Non-rotational differences remain measurable, with material interpolation/discretization uncertainty.')
    p('new CFD iterations=0; new SA iterations=0; new SST iterations=0; SST-LM=0; URANS=0; Transient=0; SAS=0; DES=0; LES=0; mesh changes=0; prism changes=0; L11 changes=0; Run A=NO; Run A2=NO; production authority replaced=NO; new convergence contract=NO.')
    p('Exactly one recommended next task: a read-only discrete steady-operator/Jacobian diagnostic on the retained GW1 SA states, with rotation-consistent interpolation checks, to separate a near-neutral rotational mode from mesh/domain pinning and unresolved non-rotational residuals. Do not advance CFD as part of that diagnostic.')
    (r.ROOT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')

if __name__=='__main__':supplement();report()
