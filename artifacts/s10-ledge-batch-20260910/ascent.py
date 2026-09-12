"""Match only approach/ascent/first level support; preserve the prior full-cycle report."""
import argparse
import json
import numpy as np
import batch_match as b
import render_results

BASE=b.OUT
OUT=BASE/'ascent'
# Fixed using recorded IMU/joint phases, before searching any shortened simulation.
ENDS={'150146_C1':14.0,'150146_U2':30.5,'150146_U3':72.,'150146_U4':110.,
      '150705_C1':12.35,'150705_C2':29.6,'150705_C3':41.95,'150705_C4':66.5,
      '150921_C1':13.5,'151220_U1':13.3}


def choose(results):
    return min([r for r in results if r['completed']] or [r for r in results if r.get('up_completed')] or results,
               key=lambda r:r['score'])


def refine_fidelity(c,ref,geom,results):
    chosen=choose(results)
    if not chosen['completed'] or chosen['reference_orientation_rmse_deg']<=20:return results
    gains=[120.,160.] if chosen['kp_leg']==80 else [160.,200.]
    offsets=np.round(chosen['offset_m']+np.array([-.03,0,.03]),5)
    delays=np.round(np.clip(chosen['delay_s'],-.2,.2)+np.array([-.1,0,.1]),5)
    def key(r):return tuple(round(r.get(k,v),5) for k,v in [('kp_leg',80),('kd_leg',2),('offset_m',0),('delay_s',0)])
    tested={key(r) for r in results}
    for kp in gains:
        cc={**c,'controller':dict(kp_leg=kp,kd_leg=kp/40)}
        for offset in offsets:
            for delay in delays:
                trial=dict(kp_leg=kp,kd_leg=kp/40,offset_m=float(offset),delay_s=float(delay))
                if key(trial) not in tested:
                    results.append(b.run(cc,ref,geom,float(offset),float(delay)));tested.add(key(trial))
    return results


def refine_wheels(c,ref,geom,results):
    chosen=choose(results)
    if not chosen['completed'] or chosen['reference_orientation_rmse_deg']<=15:return results
    for wheel_kd in [.9,1.2,1.8,2.4]:
        cc={**c,'controller':dict(kp_leg=chosen['kp_leg'],kd_leg=chosen['kd_leg'],kd_wheel=wheel_kd)}
        results.append(b.run(cc,ref,geom,chosen['offset_m'],chosen['delay_s']))
    return results


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--only');args=parser.parse_args()
    b.selfcheck();OUT.mkdir(exist_ok=True);b.OUT=OUT
    clips=[];summaries=[]
    for old in b.CLIPS:
        if old['mode']=='down':continue
        c={k:v for k,v in old.items() if k not in ('reverse_start','reverse_peak','reference_outcome')}
        c.update(end=ENDS[c['id']],mode='up',parent_mode=old['mode'],parent_end=old['end'],
                 crop_reason='Recorded first level phase before subsequent body adjustment/reverse motion' if old['mode']=='cycle' else 'Entire original ascent-only interval retained')
        clips.append(c)
    b.m.review.dump(OUT/'clips.json',clips)
    for c in clips:
        if args.only and args.only not in (c['id'],c['recording']):continue
        p=BASE/c['id'];folder=OUT/c['id'];folder.mkdir(exist_ok=True)
        original=dict(np.load(p/'reference.npz'));keep=original['time_s']<=c['end']+1e-7
        ref={k:(v[keep] if k!='initial_gyro' else v) for k,v in original.items()}
        assert abs(ref['time_s'][-1]-c['end'])<1e-6
        geom=json.loads((p/'geometry.json').read_text());config=json.loads((p/'configuration.json').read_text())
        config={k:v for k,v in config.items() if k not in ('reverse_start','reverse_peak')}
        np.savez_compressed(folder/'reference.npz',**ref)
        b.m.review.dump(folder/'geometry.json',geom);b.m.review.dump(folder/'configuration.json',{**config,**c})
        print('ASCENT',c['id'],c['start'],c['end'],flush=True)
        baseline=b.run(c,ref,geom,0.,0.,'baseline');results=[baseline]
        for offset in np.linspace(-.18,.18,13):
            if abs(offset)>1e-8:results.append(b.run(c,ref,geom,float(offset),0.))
        success=[r for r in results if r['completed']]
        if not success:
            old_trials=json.loads((p/'search.json').read_text())
            known=[r for r in old_trials if r.get('up_completed') and abs(r['delay_s'])>1e-8]
            for r in known:results.append(b.run(c,ref,geom,r['offset_m'],r['delay_s']))
            success=[r for r in results if r['completed']]
        if not success:
            tested={(round(r['offset_m'],5),round(r['delay_s'],5)) for r in results}
            for offset in np.linspace(-.18,.18,13):
                for delay in [-.3,-.2,-.1,.1,.2,.3]:
                    if (round(offset,5),round(delay,5)) not in tested:
                        results.append(b.run(c,ref,geom,float(offset),delay))
            success=[r for r in results if r['completed']]
        if not success:
            # Bounded tracking-gain diagnostic; height, masses, reference and torque limits stay fixed.
            for kp,kd in [(120.,3.),(160.,4.)]:
                cc={**c,'controller':dict(kp_leg=kp,kd_leg=kd)}
                for offset in [-.09,-.06,0.,.06,.12]:
                    for delay in [-.2,0.,.2]:results.append(b.run(cc,ref,geom,offset,delay))
            success=[r for r in results if r['completed']]
        results=refine_fidelity(c,ref,geom,results)
        results=refine_wheels(c,ref,geom,results)
        success=[r for r in results if r['completed']]
        partial=[r for r in results if r.get('up_completed')]
        chosen=choose(results)
        c['controller']={k:chosen[k] for k in ['kp_leg','kd_leg','kd_wheel']}
        selected=b.run(c,ref,geom,chosen['offset_m'],chosen['delay_s'],'selected')
        summary=dict(clip=c,geometry={k:v for k,v in geom.items() if k!='fits'},baseline=baseline,selected=selected,
            trial_count=len(results),successful_trials=len(success),up_completed_trials=len(partial),human_review='candidate_not_confirmed',training_ready=False,
            previous_selected=json.loads((p/'selected.json').read_text()),
            reason='Ascent-only simulation fit; not a human-qualified demonstration or whole-cycle success')
        b.m.review.dump(folder/'search.json',results);b.m.review.dump(folder/'summary.json',summary)
        b.m.review.dump(folder/'configuration.json',{**config,**c})
        render_results.export(c);summaries.append(summary)
        print('RESULT',c['id'],selected['completed'],'offset',selected['offset_m'],'delay',selected['delay_s'],
              'first_support',selected['up_completed_source_s'],flush=True)
    b.m.review.dump(OUT/'last_run.json',summaries)


if __name__=='__main__':main()
