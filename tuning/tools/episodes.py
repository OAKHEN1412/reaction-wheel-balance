import sys
rows=[]
for ln in open(sys.argv[1],encoding='utf-8',errors='replace'):
    p=ln.strip().split(',')
    if len(p)>=7 and p[0].isdigit():
        try: rows.append((int(p[0]),p[1],*map(float,p[2:7])))
        except: pass
eps=[];cur=None
for r in rows:
    if r[1]=='BALANCING':
        cur=(cur or [])+[r]
    elif cur:
        eps.append(cur); cur=None
if cur: eps.append(cur)
b=[r for r in rows if r[1]=='BALANCING']
print("glitch>700rpm:",sum(1 for r in b if abs(r[4])>700), "of", len(b))
for i,e in enumerate(eps):
    th=[r[2] for r in e]; du=[r[6] for r in e]
    # zero crossings of theta -> oscillation period
    zc=[e[k][0] for k in range(1,len(e)) if th[k-1]*th[k]<0]
    per=(2*(zc[-1]-zc[0])/(len(zc)-1)/1000) if len(zc)>2 else 0
    print(f"ep{i:2d} dur={(e[-1][0]-e[0][0])/1000:5.2f}s th=[{min(th):6.1f},{max(th):5.1f}] end={th[-1]:6.1f} period~{per:.2f}s duty<0.1:{sum(1 for d in du if d<0.1)/len(du):.0%} sat:{sum(1 for d in du if d>=0.99)/len(du):.0%}")
