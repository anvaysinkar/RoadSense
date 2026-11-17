import json, roadsense_analyze as ra, rs_features as rf, eval_3vehicles as ev3
P=dict(ra.DEFAULTS); P.update(json.load(open('best_3veh.json')))
for v in ['hatchback','sedan','suv']:
    seg=max(ra.parse_log(f'../datasets/raw_drives/{v}.txt'), key=lambda s: len(s['raw']))
    for kl in (6,2):
        Q=dict(P,K_LOCAL=kl)
        a,_=ra.replay(seg['raw'],seg['kcal'],Q,ra.make_gps_lookup(seg))
        b=rf.replay_ml(seg['raw'],seg['kcal'],Q,ra.make_gps_lookup(seg))
        keys=['t','label','heaveDurMs','nHits','hitSpanMs','firstSign','jerkPk_mg']
        same=len(a)==len(b) and all(all(x[k]==y[k] for k in keys) for x,y in zip(a,b))
        t0=ev3.calib_end(seg,Q); n=sum(1 for e in b if e['t']>=t0)
        print(v,kl,len(a),len(b),'identical' if same else 'MISMATCH','after calib',n, 'win len', min(e['n_win'] for e in b), max(e['n_win'] for e in b))
