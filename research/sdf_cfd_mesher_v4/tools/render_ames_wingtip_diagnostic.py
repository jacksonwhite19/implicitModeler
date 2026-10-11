"""Render compact diagnostic views of the Ames wingtip parameterization.

This output is not a production surface and is not used as qualification input.
"""
from pathlib import Path
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from ames_wingtip_geometry_authority import diagnostic_surface, bounds
OUT=HERE.parent/'results/ames_wingtip_geometry_authority'
OUT.mkdir(parents=True,exist_ok=True)
x,f=diagnostic_surface()
fig=plt.figure(figsize=(12,7)); ax=fig.add_subplot(111,projection='3d')
ax.plot_trisurf(x[:,0],x[:,2],x[:,1],triangles=f,color='#4c8eda',alpha=.72,linewidth=.08,edgecolor='#17324d')
ax.set_xlabel('x streamwise (m)'); ax.set_ylabel('z span (m)'); ax.set_zlabel('y vertical (m)')
ax.set_title('NASA Ames NACA 0012 rounded-tip parameterization\nDiagnostic only; not production adaptive surface')
ax.view_init(elev=24,azim=-66); fig.tight_layout(); fig.savefig(OUT/'ames_wingtip_geometry_overview.png',dpi=160); plt.close(fig)
fig=plt.figure(figsize=(10,5)); ax=fig.add_subplot(111)
for th in np.linspace(0,np.pi,7):
 p=np.array([__import__('ames_wingtip_geometry_authority',fromlist=['tip_point']).tip_point(float(q),float(th)) for q in np.linspace(0,1,300)])
 ax.plot(p[:,0],p[:,1],label=f'theta={th:.2f}')
ax.set_xlabel('x streamwise (m)'); ax.set_ylabel('y vertical (m)'); ax.set_title('Section and rotated-tip traces'); ax.grid(True,alpha=.25); fig.tight_layout(); fig.savefig(OUT/'ames_wingtip_section_traces.png',dpi=160); plt.close(fig)
(OUT/'GEOMETRY_BOUNDS.json').write_text(__import__('json').dumps(bounds(),indent=2)+'\n')
print(bounds()); print(OUT)
