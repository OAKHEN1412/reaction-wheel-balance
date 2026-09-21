import sys, statistics as s
rows=[]
for ln in open(sys.argv[1],encoding='utf-8',errors='replace'):
    p=ln.strip().split(',')
    if len(p)>=7 and p[0].isdigit():
        try: rows.append((int(p[0]),p[1],*map(float,p[2:7])))
        except: pass
eps=[];cur=None
for r in rows:
    if r[1]=='BALANCING': cur=(cur or [])+[r]
    elif cur: eps.append(cur); cur=None
if cur: eps.append(cur)
for i,e in enumerate(eps):
    dur=(e[-1][0]-e[0][0])/1000
    if dur<3: print(f"ep{i} {dur:.1f}s (short)"); continue
    seg=e[100:] if len(e)>200 else e   # skip first 2 s
    th=[r[2] for r in seg]; m=s.mean(th)
    zc=[seg[k][0] for k in range(1,len(seg)) if (th[k-1]-m)*(th[k]-m)<0]
    per=(2*(zc[-1]-zc[0])/(len(zc)-1)/1000) if len(zc)>2 else 0
    w=[r[4] for r in seg if abs(r[4])<700]; d=[r[6] for r in seg]
    print(f"ep{i} dur={dur:5.1f}s ended={'FALL' if e is not eps[-1] or rows[-1][1]!='BALANCING' else 'still'} th mean={m:5.2f} SD={s.pstdev(th):.2f} range=[{min(th):.1f},{max(th):.1f}] period={per:.2f}s wheel mean={s.mean(w):.0f} [{min(w):.0f},{max(w):.0f}] duty max={max(d):.2f}")
