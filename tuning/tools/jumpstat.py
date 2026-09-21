import sys
lines=open(sys.argv[1],encoding='utf-8',errors='replace').read().splitlines()[int(sys.argv[2]):]
rows=[]
for ln in lines:
    p=ln.strip().split(',')
    if len(p)>=7 and p[0].isdigit():
        try: rows.append((int(p[0]),p[1],*map(float,p[2:7])))
        except: pass
msgs=[l for l in lines if l.startswith('jump:')]
j=[r for r in rows if r[1]=='JUMP_UP']
if not j: print("no jump rows", msgs); sys.exit()
t0=j[0][0]; th0=j[0][2]; side=1 if th0>0 else -1
kick=[r for r in j if r[6]>=0.99 and r[5]*side>0]
cap=next((r for r in rows if r[0]>t0 and r[1]=='BALANCING'),None)
after=[r for r in rows if cap and r[0]>=cap[0] and r[1]=='BALANCING']
fell=next((r for r in rows if cap and r[0]>cap[0] and r[1]=='FALLEN'),None)
print("msgs:",msgs[-2:] if msgs else None)
print(f"rest={th0:.1f}  spinup_ms={(kick[0][0]-t0) if kick else '-'}  peak_rate={max(abs(r[3]) for r in j):.0f}dps")
if cap:
    ov=max((-side*r[2] for r in after), default=0)
    first0=next((r[0] for r in after if r[2]*side<=0),None)
    settle=next((after[k][0] for k in range(len(after)) if all(abs(x[2])<1.5 for x in after[k:k+50]) and len(after[k:k+50])==50),None)
    print(f"captured at {cap[2]:.1f}deg rate={cap[3]:.0f}  t_capture={cap[0]-t0}ms  overshoot={ov:.1f}deg  t_upright={(first0-t0) if first0 else '-'}ms  settle(<1.5deg 1s)={(settle-t0) if settle else '-'}ms  ->", "FELL" if fell else f"still balancing ({(after[-1][0]-cap[0])/1000:.1f}s)")
else:
    print("not captured; min |theta| reached:", round(min(abs(r[2]) for r in rows if r[0]>=t0),1))
