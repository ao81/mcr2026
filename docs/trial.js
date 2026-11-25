// 区間タイムによる実車比較。編集中と保存後で位置が変わるため、項目は名前と添字で参照する。
'use strict';
const trialStorage = 'mcr-section-trial-v1';
function trialFields(names=['config.h','tables.h']) {
  const result = new Map();
  for (const name of names) {
    const f = headerFile(name);
    if (!f) throw Error(name + ' を含むリポジトリを開いてください');
    for (const g of f.groups) for (const item of g.items) {
      itemToks(item).forEach((t,i) => {
        const id = item.kind === 'define' ? item.name : item.name + '[' + i + ']';
        result.set(id,{f,t,value:display(f.edits.get(t.key) ?? t.orig)});
      });
    }
  }
  return result;
}
function trialSignature(fields, original=false) {
  let h = 2166136261;
  for (const [k,v] of fields) for (const c of k+'='+(original?display(v.t.orig):v.value)+';') h = Math.imul(h ^ c.charCodeAt(0),16777619);
  return h >>> 0;
}
function trialEdit(values) {
  const fields = trialFields();
  for (const [name,value] of Object.entries(values)) {
    const {f,t} = fields.get(name);
    const out = format(t.orig, value);
    if (out === t.orig) f.edits.delete(t.key); else f.edits.set(t.key,out);
  }
}
function trialId() { return 1 + crypto.getRandomValues(new Uint32Array(1))[0] % 2147483646; }
function readTrial() { return JSON.parse(localStorage.getItem(trialStorage) || 'null'); }
function persistTrial(s) {
  localStorage.setItem(trialStorage,JSON.stringify(s));
  S.activeTrial = s;
  renderWorkspace();
  $('trialStatus').textContent = s.message;
}

// センサ表のマクロの種類に従い、実際に通った段階とセンサ値で呼ばれる出力だけを試す。
function actionName(p) {
  if (p>=20 && p<=25) return ['TRACE',p-20];
  if (p>=30 && p<=35) return ['CURVE_R',p-30];
  if (p>=40 && p<=45) return ['CURVE_L',p-40];
  if ([101,102,103,201,501].includes(p)) return ['LINE_STEER',0];
  if (p>=110 && p<=161) {
    const suffix = {0:'ENTER',1:'SLOW',10:'RETURN',11:'OUTSIDE'}[(p>=150?p-150:p-110)];
    if (suffix) return ['CRANK_'+(p>=150?'R_':'L_')+suffix,0];
  }
  if (p>=300 && p<=750) {
    const k = p>=600?p-600:p-300;
    for (const [base,name] of [[0,'APPROACH'],[100,'ALIGN'],[120,'FINISH'],[140,'RECOVER']]) {
      if (k>=base && k<=base+10) return ['LANE_'+(p>=600?'L_':'R_')+name,k-base];
    }
  }
  return null;
}
function trialQueue(target, seen, params) {
  const f = trialFields(), get = k => Number(f.get(k)?.value);
  const q = [];
  function offer(name,step,lo,hi,center=get(name)) {
    if (!Number.isFinite(center)) return;
    for (const dir of [1,-1]) {
      const val = Number((center+dir*step).toFixed(5));
      if (val>=lo && val<=hi && val!==get(name)) q.push({[name]:val});
    }
  }
  const encountered=(lo,hi)=>seen.some(([p])=>p>=lo && p<=hi);
  const crank=target===1 || (target===6 && encountered(100,161));
  const lane=target===2 || (target===6 && encountered(200,750));
  const legacyLane=lane && (!get('USE_NEW_LANE_STEER') || encountered(320,350) || encountered(620,650));
  const trace = [4,5,6].includes(target);
  const specs = (enabled, lines) => {
    if (enabled) for (const [key,step,lo,hi] of lines) offer(key,step,lo,hi);
  };
  for (const [use,prefix] of [[crank,'CRANK'],[lane,'LANE']]) {
    if (!use) continue;
    if (get('USE_NEW_'+prefix+'_BRAKE')) {
      offer(prefix+'_BRAKE_MARGIN_M',-0.005,0,0.3);
      offer(prefix+'_BRAKE_POWER',-3,-100,-1);
      const key=prefix+'_ENTRY_MPS', auto=params[prefix.toLowerCase()+'_entry_mps'];
      offer(key,0.04,0.2,get('V_MAX_MPS'),get(key)||auto);
    } else {
      offer(prefix+'_K_MIN',1,1,100);
      offer(prefix+'_RAMP_STEP_MS',1,1,150);
    }
  }
  specs(crank,[['CROSS_STRAIGHT_MS',-3,1,300],['CRANK_EXIT_LINE_IGNORE_MS',-5,0,500]]);
  specs(crank && get('USE_NEW_CRANK_QUICK'),[['CRANK_QUICK_WINDOW_M',-0.002,0.002,0.10]]);
  specs(crank && get('USE_NEW_CRANK_TURN'),[['CRANK_ARC_EXIT_RATIO',-0.02,0.4,1]]);
  specs(crank && seen.some(x=>[110,111,150,151].includes(x[0])),[['CRANK_ANGLE',1,1,get('SERVO_MAX_ANGLE')],['CRANK_ENTER_MS',-3,1,300]]);
  specs(lane,[['LANE_START_WAIT_MS',-3,1,300]]);
  specs(lane && get('USE_NEW_LANE_STEER'),[['LANE_SEARCH_M',-0.01,0.02,0.6]]);
  specs(lane && get('USE_NEW_LANE_DISTANCE'),[['LANE_DIST_ERR',0.005,0.005,0.20]]);
  specs(legacyLane,[['LANE_STEER_ANGLE',1,1,get('SERVO_MAX_ANGLE')],['LANE_STRAIGHT_ANGLE',1,0,12]]);
  specs(trace,[['CURVE_DRIVE_PERCENT',2,1,100]]);
  specs(true,[['DIP_SPEED_BASE',1,1,7],['PLAN_GRIP_RATIO_LAT',0.02,0.35,1],['PLAN_GRIP_RATIO_LONG',0.02,0.35,1]]);
  specs(crank||lane,[['COURSE_MARGIN_M',-0.002,0.002,0.06],['PLAN_OVERSPEED',0.01,1,1.15],['SPEED_HOLD_BAND_MPS',0.01,0.01,0.2],['SPEED_HOLD_S',-0.002,0.005,0.10]]);
  specs(trace && (get('USE_NEW_TRACE_SPEED')||get('USE_NEW_CURVE_SPEED')),[['TRACE_MIN_POWER',-3,-100,0],['TRACE_CURV_FILTER_S',-0.002,0.002,0.20],['TRACE_SAFE_RADIUS_M',0.05,0,2]]);
  specs(trace && get('USE_NEW_CURVE_PREVIEW'),[['PREVIEW_CURV_FILTER_S',-0.005,0.005,0.3],['PREVIEW_RATIO',0.05,0,1]]);
  specs(trace && (get('USE_NEW_LINE_SHAPE')||get('USE_NEW_CURVE_PREVIEW')),[['LINE_OBS_BAND',0.5,1,40],['LINE_OBS_DEADZONE_M',0.001,0,0.01],['LINE_CHECK_MAX_ANGLE',0.02,0.1,0.8],['LINE_CHECK_MIN_STEER',1,1,30]]);
  specs(trace && get('USE_NEW_LINE_SHAPE'),[['LINE_STEER_GAIN',0.5,1,40],['STEER_LEAD_S',0.002,0,0.08]]);

  // 機能を切り替える場合はスケッチの static_assert と同じ依存関係を満たす。
  const requirements = {CRANK_QUICK:['CRANK_BRAKE','CRANK_TURN'],LANE_BRAKE:['LANE_STEER'],LANE_DISTANCE:['LANE_STEER'],
    LANE_DOWN_ONLY:['LANE_DISTANCE'],LINE_SHAPE:['STEER'],CURVE_PREVIEW:['TRACE_SPEED']};
  if (!get('CRANK_ENTRY_MPS')) requirements.CRANK_BRAKE=['CRANK_TURN'];
  specs(get('USE_NEW_VMAX_LEARN'),[['VMAX_LEARN_MIN_GAIN',0.02,0,1],['VMAX_LEARN_RANGE',0.01,0.02,0.5],
    ['VMAX_STEADY_MS',-10,100,2000],['VMAX_STEADY_TOL',0.5,0.5,10],
    ['VMAX_STEADY_MIN_RATIO',0.05,0.1,0.9],['VMAX_STEADY_MAX_RATIO',0.05,1.1,3]]);
  const switches = ['TRACTION','VMAX_LEARN'];
  if (trace) switches.push('TRACE_SPEED','CURVE_SPEED','STEER','LINE_SHAPE','CURVE_PREVIEW');
  if (crank) switches.push('CRANK_QUICK','CRANK_BRAKE','CRANK_TURN');
  if (lane) switches.push('LANE_STEER','LANE_BRAKE','LANE_DISTANCE','LANE_WIDTH_INTERVAL','LANE_DOWN_ONLY');
  if (crank||lane) switches.push('BRAKE_RANGE');
  for (const sw of switches) {
    const patch = {};
    function toggle(k,on) {
      const name='USE_NEW_'+k;
      if ((patch[name] ?? get(name)) === on) return;
      patch[name]=on;
      if (on) for (const dep of requirements[k]||[]) toggle(dep,1);
      else for (const [dep,parents] of Object.entries(requirements)) if (parents.includes(k)) toggle(dep,0);
    }
    toggle(sw,1-get('USE_NEW_'+sw));
    q.push(patch);
  }

  const arrays=headerFile('tables.h').groups.flatMap(g=>g.items);
  const covered=new Set();
  for (const [p,sensor] of seen) {
    const action=actionName(p);
    if (!action) continue;
    let [name,stage]=action;
    if (name.endsWith('_APPROACH') && get('USE_NEW_LANE_STEER')) { name='LINE_STEER'; stage=0; }
    const group=arrays.find(i=>i.name===name+'_ACT'), sensors=arrays.find(i=>i.name===name+'_SENSOR');
    if (!group || !sensors) continue;
    const n=itemToks(sensors).findIndex((_,i)=>get(name+'_SENSOR['+i+']')===sensor);
    if (n<0) continue;
    const all=itemToks(group), ts=group.rows[stage]?.toks;
    if (!ts?.length) continue;
    const source=headerFile('tables.h').text;
    const multi=group.rows.length>1 && /^(TRACE|CURVE_|LANE_)/.test(name);
    const available=multi?ts:all;
    const first=available[0].start, last=available.at(-1).end;
    const begin=source.lastIndexOf('(',first);
    const start=source.slice(0,begin).search(/[A-Z]+$/);
    if (start<0) continue;
    const calls=[...source.slice(start,last+1).matchAll(/\b(AM|M|A)\(\s*([-+\d,\s]+)/g)];
    const call=calls[n];
    if (!call) continue;
    const at=start+call.index, count=call[2].split(',').length;
    const idx=all.findIndex(t=>t.start>=at);
    for (let j=0;j<count;j++) {
      const steer=call[1]!=='M' && !j;
      if (idx+j>=all.length) continue;
      if (name==='TRACE' && (steer?get('USE_NEW_STEER'):get('USE_NEW_TRACE_SPEED'))) continue;
      if (name.startsWith('CURVE_') && !steer && get('USE_NEW_CURVE_SPEED')) continue;
      const key=name+'_ACT['+(idx+j)+']';
      if (covered.has(key)) continue;
      covered.add(key);
      const limit=steer?get('SERVO_MAX_ANGLE'):100;
      offer(key,steer?1:2,-limit,limit);
    }
  }
  for (const [key,step,lo,hi,enabled] of [
    ['TRACE_STAGE_UP_MS',-5,1,4000,trace],['CURVE_STAGE_DOWN_MS',-2,0,1000,trace],
    ['LANE_STAGE_LIMIT',1,1,120,lane],['LANE_STEER_OUTER',2,-100,100,legacyLane],['LANE_STEER_INNER',2,-100,100,legacyLane],
    ['LANE_STEER_MS',-3,1,600,legacyLane],['LANE_STRAIGHT_SPEED',2,-100,100,legacyLane],
    ...['CRANK','LANE'].flatMap(prefix=>['FULL_MS','HOLD_END_MS','RAMP'].map(k=>[prefix+'_'+k,k==='RAMP'?2:-3,k==='RAMP'?-100:0,k==='RAMP'?100:2000,(prefix==='CRANK'?crank:lane)&&!get('USE_NEW_'+prefix+'_BRAKE')]))]) {
    if (!enabled) continue;
    for (let i=0;f.has(key+'['+i+']');i++) {
      const id=key+'['+i+']';
      let min=lo,max=hi;
      if (['TRACE_STAGE_UP_MS','CURVE_STAGE_DOWN_MS','LANE_STAGE_LIMIT'].includes(key)) {
        if (i>0) min=Math.max(min,get(key+'['+(i-1)+']')+1);
        if (f.has(key+'['+(i+1)+']')) max=Math.min(max,get(key+'['+(i+1)+']')-1);
      }
      if (key.endsWith('FULL_MS')) max=Math.min(max,get(id.replace('FULL_MS','HOLD_END_MS')));
      if (key.endsWith('HOLD_END_MS')) min=Math.max(min,get(id.replace('HOLD_END_MS','FULL_MS')));
      offer(id,step,min,max);
    }
  }
  return q.filter(patch=>Object.keys(patch).length);
}

function trialStats(times) {
  const median = a => [...a].sort((x,y)=>x-y)[Math.floor(a.length/2)];
  const ms=median(times);
  return {ms,spread:median(times.map(t=>Math.abs(t-ms)))};
}
const trialHeaders=['config.h','tables.h','dbg_config.h'];
function trialStamp(s) {
  const dbg=headerFile('dbg_config.h');
  stageValues(dbg,{DBG_TRIAL_ID:s.id});
  const fields=trialFields(trialHeaders);
  s.signature=trialSignature(fields);
  s.savedSignature=trialSignature(fields,true);
  s.staged={};
  for (const [key,{f,t,value}] of fields) if (f.edits.has(t.key)) s.staged[key]=value;
}
function nextTrial(s, feedback) {
  // 毎回、現在までの採用設定から候補を作る。採用で切り替わった制御の候補にも対応する。
  const patch=trialQueue(s.target,s.seen,s.params).find(p=>{
    const key=Object.keys(p).sort().map(k=>k+':'+p[k]).join(',');
    return !s.tried.includes(key);
  });
  if (!patch) {
    s.pending=null;
    s.message=feedback+'。全候補の比較が終了しました。「比較を終了・採用値に戻す」で確定してください。';
    trialStamp(s);
    return;
  }
  const f=trialFields();
  s.undo=Object.fromEntries(Object.keys(patch).map(k=>[k,f.get(k).value]));
  s.pending=patch;
  s.tried.push(Object.keys(patch).sort().map(k=>k+':'+patch[k]).join(','));
  trialEdit(patch);
  s.id=trialId();
  s.runs=[];
  trialStamp(s);
  s.message=feedback+'。次の候補 '+Object.entries(patch).map(([k,v])=>k+' '+s.undo[k]+' → '+v).join('、')+
    '。保存し、debug/debug.ino を書き込んで同じ条件で3走してください。';
}
function beginTrial() {
  const existing=readTrial();
  if (existing) { checkTrial(existing); $('trialStatus').textContent=existing.message; return; }
  trialFields();
  field(headerFile('dbg_config.h'),'DBG_TRIAL_ID');
  prepareRun();
  const s={target:Number($('eventType').value), number:Number($('eventNumber').value), id:trialId(),
    dip:null,route:null,best:null,runs:[],seen:[],params:{},tried:[],pending:null,
    message:'基準設定を保存し、debug/debug.ino を書き込んでください。同じ条件で3走し、比較用CSVを追加します。'};
  trialStamp(s);
  persistTrial(s);
}
function checkTrial(s) {
  const actual=trialSignature(trialFields(trialHeaders));
  if (actual===s.signature) return;
  if (actual===s.savedSignature) {
    const dbg=headerFile('dbg_config.h');
    trialEdit(Object.fromEntries(Object.entries(s.staged).filter(([k])=>!k.startsWith('DBG_'))));
    stageValues(dbg,Object.fromEntries(Object.entries(s.staged).filter(([k])=>k.startsWith('DBG_'))));
    if (trialSignature(trialFields(trialHeaders))===s.signature) return;
  }
  throw Error('比較中の設定と違います。最後に保存した比較用設定を開いてください');
}
function addTrial(csv) {
  const s=readTrial();
  if (!s) throw Error('先に「タイム比較を開始」を押してください');
  if (s.best && !s.pending) throw Error(s.message);
  checkTrial(s);
  const {meta,rows,params}=parseDebugCsv(csv);
  if (meta.trial!==s.id || meta.target!==s.target || meta.target_no!==s.number ||
      !Number.isSafeInteger(meta.run) || meta.run<=0 || !Number.isInteger(meta.dip) ||
      !Number.isInteger(meta.route)) throw Error('今回の設定を保存・書き込みして取得した比較用CSVを選んでください');
  if (s.runs.some(r=>r.no===meta.run)) throw Error('同じ走行番号のCSVは1回だけ追加できます');
  if (s.dip!==null && meta.dip!==s.dip) throw Error('DIPスイッチが基準走行と異なります');
  if (![1,2,4].includes(meta.reason)) throw Error('区間終了または途中停止の詳細CSVを選んでください');
  const complete=meta.reason===1;
  if (complete && (!Number.isInteger(meta.end_ms) || !Number.isInteger(meta.trigger_ms) ||
      meta.trigger_ms<0 || meta.end_ms<=meta.trigger_ms || meta.events!==s.number ||
      Math.abs(rows.at(-1).ms-meta.end_ms)>=meta.interval_ms)) throw Error('指定区間の終了時刻を確認できません');
  const sameRoute=s.route===null || meta.route===s.route;
  if (!complete || !sameRoute) {
    if (!s.best) {
      s.runs=[]; s.route=s.dip=null; s.seen=[]; s.params={};
      s.message='基準走行が途中停止または別経路でした。位置と条件を確認し、3走を取り直してください。';
    } else {
      trialEdit(s.undo);
      nextTrial(s,complete?'通過した標識の順序・種類が異なるため候補を取り消しました':'途中停止のため候補を取り消しました');
    }
  } else {
    s.route=meta.route; s.dip=meta.dip;
    const ms=meta.end_ms-meta.trigger_ms;
    s.runs.push({no:meta.run,ms});
    if (!s.best) {
      const seen=new Map(s.seen.map(x=>[x.join(','),x]));
      for (const row of rows) if (row.ms>=meta.trigger_ms && row.ms<meta.end_ms) {
        const sensor=parseInt(String(row.sensor),2);
        if (Number.isInteger(sensor)) seen.set(row.pattern+','+sensor,[row.pattern,sensor]);
      }
      s.seen=[...seen.values()]; s.params=params;
    }
    if (s.runs.length<3) s.message=(s.best?'候補':'基準')+' '+s.runs.length+'/3走。今回 '+ms+' ms。残りのCSVを追加してください。';
    else {
      const stat=trialStats(s.runs.map(r=>r.ms));
      let feedback;
      if (!s.best) { s.best=stat; feedback='基準中央値 '+stat.ms+' ms'; }
      else if (s.best.ms-stat.ms>Math.max(2,s.best.spread,stat.spread)) {
        feedback='採用 '+s.best.ms+' → '+stat.ms+' ms'; s.best=stat;
      } else {
        trialEdit(s.undo); feedback='候補中央値 '+stat.ms+' ms。改善を確認できず、採用値 '+s.best.ms+' ms に戻しました';
      }
      nextTrial(s,feedback);
    }
  }
  persistTrial(s);
}
function clearTrial() {
  localStorage.removeItem(trialStorage);
  S.activeTrial=null;
  renderWorkspace();
}
function endTrial() {
  const s=readTrial();
  if (!s) throw Error('比較は開始されていません');
  try { checkTrial(s); }
  catch(e) {
    if (!confirm('開いている設定が比較中の値と異なります。ファイルは変更せず、比較記録だけを終了しますか？')) throw e;
    clearTrial();
    $('trialStatus').textContent='比較記録を終了しました。車体へ書き込む値を確認してください。';
    return;
  }
  if (s.pending) trialEdit(s.undo);
  stageValues(headerFile('dbg_config.h'),{DBG_TRIAL_ID:0});
  clearTrial();
  $('trialStatus').textContent=(s.best?'採用した設定の中央値 '+s.best.ms+' ms。':'基準測定を終了しました。')+'保存・同期し、全体を走るときは basic/basic.ino を書き込んでください。';
}
function trialError(e) {
  let guidance='';
  try { guidance=readTrial()?.message || ''; } catch (_) {}
  const why=e.message || String(e);
  $('trialStatus').textContent=why+(guidance && guidance!==why ? '。'+guidance : '');
}
$('trialStart').onclick=()=>{try { beginTrial(); } catch(e) { trialError(e); }};
$('trialEnd').onclick=()=>{try { endTrial(); } catch(e) { trialError(e); }};
$('trialLog').onchange=async e=>{
  const input=e.target;
  S.readingTrials=true;
  renderWorkspace();
  try { for (const file of input.files) addTrial(await file.text()); }
  catch(err) { trialError(err); }
  finally { input.value=''; S.readingTrials=false; renderWorkspace(); }
};
try {
  S.activeTrial=readTrial();
  if (S.activeTrial) {
    $('trialStatus').textContent=S.activeTrial.message;
    switchWorkspace(true);
  }
} catch(e) { trialError(e); }
