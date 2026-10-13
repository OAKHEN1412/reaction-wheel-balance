import sys, re, statistics as st, glob
pat = re.compile(r'^(\d+),([A-Z_]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)$')
def load(fn):
    rows=[]
    for line in open(fn, encoding='utf-8', errors='replace'):
        m=pat.match(line.strip())
        if m:
            t,s,th,thd,w,c,d=m.groups(); rows.append((int(t),s,float(th),float(thd),float(w),float(c),float(d)))
    return rows
def episodes(rows):
    eps=[];cur=[]
    for r in rows:
        if r[1]=='BALANCING':
            if cur and r[0]-cur[-1][0]>500: eps.append(cur);cur=[]
            cur.append(r)
        else:
            if cur: eps.append((cur)); cur=[]
    if cur: eps.append(cur)
    return eps
def main():
  for fn in sorted(glob.glob('logs/*.log')):
      rows=load(fn)
      if not rows: continue
      states=set(r[1] for r in rows)
      eps=episodes(rows)
      long=[e for e in eps if (e[-1][0]-e[0][0])>3000]
      print(f"== {fn} rows={len(rows)} states={sorted(states)} eps={len(eps)} long>3s={len(long)}")
      for e in long:
          dur=(e[-1][0]-e[0][0])/1000; th=[r[2] for r in e]
          # excursions > 2 deg
          exc=[]; inexc=False
          for r in e:
              if abs(r[2])>2.0 and not inexc: inexc=True; start=r
              if inexc and abs(r[2])<0.8: inexc=False; exc.append((start[0],r[0]))
          print(f"   ep t0={e[0][0]} dur={dur:.1f}s mean={st.mean(th):.2f} sd={st.pstdev(th):.2f} min={min(th):.1f} max={max(th):.1f} excursions>2deg={len(exc)} maxduty={max(r[6] for r in e):.2f}")

if __name__=='__main__': main()