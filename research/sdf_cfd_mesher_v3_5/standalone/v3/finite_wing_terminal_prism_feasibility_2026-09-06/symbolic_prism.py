"""CasADi proposer formulas. Independent frozen numpy evaluation is authority."""
import casadi as ca
import numpy as np
def norm(v):return ca.sqrt(ca.dot(v,v)+1e-60)
def avg(q):return sum(q)/len(q)
def area_center(q):
 if len(q)==3:return .5*ca.cross(q[1]-q[0],q[2]-q[0]),avg(q)
 pc=avg(q);aa=[ca.cross(q[(i+1)%len(q)]-q[i],pc-q[i]) for i in range(len(q))];suma=sum(aa);nhat=suma/norm(suma);an=[ca.dot(a,nhat) for a in aa]
 fc=sum(an[i]*(q[i]+q[(i+1)%len(q)]+pc) for i in range(len(q)))/(3*sum(an))
 return .5*suma,fc
def make_functions():
 p=ca.SX.sym('p',3,6);pts=[p[:,i] for i in range(6)];patterns=[[0,2,1],[3,4,5],[0,1,4,3],[1,2,5,4],[2,0,3,5]]
 areas=[];centres=[];newell=[]
 for pat in patterns:
  q=[pts[i] for i in pat];ar,c=area_center(q);areas.append(ar);centres.append(c);newell.append(.5*sum(ca.cross(q[i],q[(i+1)%len(q)]) for i in range(len(q))))
 coupled=newell[1:];a=sum(norm(s) for s in coupled)/4;matrix=sum(ca.mtimes(s,s.T) for s in coupled)/a**2;det=ca.fabs(ca.det(matrix))
 ve=avg(pts);areas=[ca.if_else(ca.dot(ar,fc-ve)<0,-ar,ar) for ar,fc in zip(areas,centres)];cest=avg(centres);py=[ca.dot(ar,fc-cest) for ar,fc in zip(areas,centres)];centre=sum(py[i]*(.75*centres[i]+.25*cest) for i in range(5))/sum(py)
 a0=pts[1]-pts[0];a1=pts[4]-pts[3]-a0;b0=pts[2]-pts[0];b1=pts[5]-pts[3]-b0;q0=ca.cross(a0,b0);q1=ca.cross(a1,b0)+ca.cross(a0,b1);q2=ca.cross(a1,b1)
 vertical=[pts[i+3]-pts[i] for i in range(3)];j=[]
 for v in vertical:
  c0=ca.dot(v,q0);c1=ca.dot(v,q1);c2=ca.dot(v,q2);denom=ca.if_else(ca.fabs(2*c2)>1e-40,2*c2,1);t=-c1/denom
  j += [c0,c0+c1+c2,ca.if_else(ca.logic_and(c2>0,ca.logic_and(t>0,t<1)),c0+c1*t+c2*t*t,c0)]
 jmin=ca.mmin(ca.vertcat(*j));vol=ca.dot(sum(vertical)/6,q0+.5*q1+q2/3);orientation=ca.dot(q0,ca.cross(pts[4]-pts[3],pts[5]-pts[3]))
 prism=ca.Function('prism',[p],[det,jmin,vol,orientation,centre])
 q=ca.SX.sym('quad',3,4);own=ca.SX.sym('own',3);nei=ca.SX.sym('nei',3);ar,fc=area_center([q[:,i] for i in range(4)]);da=ca.fabs(ca.dot(ar,fc-own));db=ca.fabs(ca.dot(ar,nei-fc));w=ca.fmin(da,db)/(da+db)
 cp=fc-own;delta=nei-own;sv=cp-ca.dot(ar,cp)/(ca.dot(ar,delta)+1e-150)*delta;svmag=norm(sv);svhat=sv/(svmag+1e-150);fd=ca.fmax(.2*norm(delta),ca.mmax(ca.horzcat(*[ca.fabs(ca.dot(q[:,i]-fc,svhat)) for i in range(4)])))
 skew=svmag/(fd+1e-150);side=ca.Function('side',[q,own,nei],[w,skew])
 return prism,side
