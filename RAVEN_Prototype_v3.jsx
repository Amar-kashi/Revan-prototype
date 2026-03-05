import { useState, useEffect, useRef, useCallback } from "react";
import * as THREE from "three";

/* ═══════════════════════════════════════════════════════════════
   MAP
═══════════════════════════════════════════════════════════════ */
const ROWS = 30, COLS = 42, CS = 0.55;
const FLOOR = 0, WALL = 1, RUBBLE = 2, SURVIVOR = 4, HAZARD = 5;

function buildMap() {
  const m = Array.from({ length: ROWS }, () => Array(COLS).fill(WALL));
  const carve = (r1, c1, r2, c2) => {
    for (let r = Math.max(0, r1); r <= Math.min(ROWS - 1, r2); r++)
      for (let c = Math.max(0, c1); c <= Math.min(COLS - 1, c2); c++)
        m[r][c] = FLOOR;
  };
  carve(1, 1, 9, 11); carve(1, 15, 9, 25); carve(1, 29, 9, 40);
  carve(13, 1, 22, 11); carve(13, 15, 22, 25); carve(13, 29, 22, 40);
  carve(25, 3, 29, 39);
  carve(4, 11, 6, 15); carve(4, 25, 6, 29);
  carve(16, 11, 18, 15); carve(16, 25, 18, 29);
  carve(9, 3, 13, 6); carve(9, 18, 13, 21); carve(9, 32, 13, 35);
  carve(22, 5, 25, 8); carve(22, 19, 25, 22); carve(22, 33, 25, 36);
  [[3,4],[7,8],[2,17],[6,22],[3,32],[8,37],[15,3],[20,7],[14,18],[21,23],[15,31],[19,38]]
    .forEach(([r,c]) => { if (m[r][c] === FLOOR) m[r][c] = RUBBLE; });
  [[5,5],[4,20],[6,34],[17,4],[16,20],[18,35]]
    .forEach(([r,c]) => { if (m[r][c] === FLOOR) m[r][c] = SURVIVOR; });
  [[2,8],[8,19],[5,36],[14,9],[20,22],[27,15]]
    .forEach(([r,c]) => { if (m[r][c] === FLOOR) m[r][c] = HAZARD; });
  return m;
}

/* ═══════════════════════════════════════════════════════════════
   A* PATHFINDING
═══════════════════════════════════════════════════════════════ */
function astar(map, sr, sc, er, ec) {
  const pass = (r,c) => r>=0&&r<ROWS&&c>=0&&c<COLS&&map[r][c]!==WALL&&map[r][c]!==RUBBLE;
  if (!pass(sr,sc)||!pass(er,ec)) return [];
  if (sr===er&&sc===ec) return [[sr,sc]];
  const cost=(r,c)=>map[r][c]===HAZARD?8:1;
  const h=(r,c)=>Math.abs(r-er)+Math.abs(c-ec);
  const key=(r,c)=>r*COLS+c;
  const open=new Map(),closed=new Set(),came=new Map(),g=new Map();
  const sk=key(sr,sc);
  g.set(sk,0); open.set(sk,{r:sr,c:sc,f:h(sr,sc)});
  const D8=[[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]];
  let iter=0;
  while(open.size>0&&iter++<10000){
    let bk=null,bf=Infinity;
    for(const[k,v]of open){if(v.f<bf){bf=v.f;bk=k;}}
    const{r,c}=open.get(bk);
    if(r===er&&c===ec){
      const path=[];let cur=bk;
      while(cur!==undefined){path.unshift([Math.floor(cur/COLS),cur%COLS]);cur=came.get(cur);}
      return path;
    }
    open.delete(bk);closed.add(bk);
    for(const[dr,dc]of D8){
      const nr=r+dr,nc=c+dc;
      if(!pass(nr,nc))continue;
      const nk=key(nr,nc);
      if(closed.has(nk))continue;
      if(dr!==0&&dc!==0&&(!pass(r+dr,c)||!pass(r,c+dc)))continue;
      const mv=(dr!==0&&dc!==0)?1.414:1;
      const ng=(g.get(bk)||0)+mv*cost(nr,nc);
      if(!g.has(nk)||ng<g.get(nk)){
        g.set(nk,ng);came.set(nk,bk);
        open.set(nk,{r:nr,c:nc,f:ng+h(nr,nc)});
      }
    }
  }
  return [];
}

function chainPaths(map, wpts) {
  if(wpts.length<2)return null;
  let full=[];
  for(let i=0;i<wpts.length-1;i++){
    const seg=astar(map,wpts[i].r,wpts[i].c,wpts[i+1].r,wpts[i+1].c);
    if(!seg.length)return null;
    full=[...full,...(i===0?seg:seg.slice(1))];
  }
  return full;
}

/* ═══════════════════════════════════════════════════════════════
   DRONE CONFIG + PRE-COMPUTED PATROL PATHS
═══════════════════════════════════════════════════════════════ */
const DRONE_CFG = [
  { id:1, zone:"A", col:0x17D4FF, hex:"#17D4FF", bat:87, status:"ACTIVE",  speed:0.008,
    wpts:[{r:2,c:2},{r:2,c:10},{r:8,c:10},{r:8,c:2}] },
  { id:2, zone:"B", col:0xA78BFA, hex:"#A78BFA", bat:73, status:"MAPPING", speed:0.009,
    wpts:[{r:2,c:16},{r:2,c:24},{r:8,c:24},{r:8,c:16}] },
  { id:3, zone:"C", col:0x34D399, hex:"#34D399", bat:91, status:"ACTIVE",  speed:0.007,
    wpts:[{r:2,c:30},{r:2,c:39},{r:8,c:39},{r:8,c:30}] },
  { id:4, zone:"D", col:0xFBBF24, hex:"#FBBF24", bat:65, status:"ACTIVE",  speed:0.010,
    wpts:[{r:14,c:2},{r:14,c:10},{r:21,c:10},{r:21,c:2}] },
  { id:5, zone:"E", col:0xFF6B6B, hex:"#FF6B6B", bat:42, status:"LOW BAT", speed:0.006,
    wpts:[{r:14,c:16},{r:14,c:24},{r:21,c:24},{r:21,c:16}] },
  { id:6, zone:"F", col:0x4ADE80, hex:"#4ADE80", bat:88, status:"ACTIVE",  speed:0.0085,
    wpts:[{r:14,c:30},{r:14,c:39},{r:21,c:39},{r:21,c:30}] },
];
const MAP_DATA = buildMap();
const PATROL_PATHS = DRONE_CFG.map(d => {
  const loop = [...d.wpts, d.wpts[0]];
  const p = chainPaths(MAP_DATA, loop);
  return p && p.length > 1 ? p : [[d.wpts[0].r, d.wpts[0].c],[d.wpts[0].r, d.wpts[0].c]];
});
const SURVIVOR_POS = [[5,5],[4,20],[6,34],[17,4],[16,20],[18,35]];
const HAZARD_POS   = [[2,8],[8,19],[5,36],[14,9],[20,22],[27,15]];
const WP_COLORS    = ["#17D4FF","#FF8800","#FF44FF","#44FFFF","#FF4444","#44FF44","#FFFF44","#FF8844"];
const BASE_POS     = { r:28, c:20 };

/* ═══════════════════════════════════════════════════════════════
   COMPONENT
═══════════════════════════════════════════════════════════════ */
export default function RAVENPrototype() {

  /* ── Refs ── */
  const mountRef      = useRef(null);
  const minimapRef    = useRef(null);
  const feedRefs      = useRef([null,null,null,null,null,null]);
  const threeRef      = useRef({});
  const camRef        = useRef({ phi:0.48, theta:-0.25, r:17, tx:0, tz:0 });
  const mouseRef      = useRef({ down:false, x:0, y:0, btn:0, moved:false });
  const pathGrpRef    = useRef(null);
  const markGrpRef    = useRef(null);
  const dronesMesh    = useRef([]);
  const dronesAnim    = useRef(DRONE_CFG.map((d,i)=>({ t:i*0.18%1, path:PATROL_PATHS[i], speed:d.speed })));
  const missionMesh   = useRef(null);
  const missionAnim   = useRef({ t:0, path:[], on:false, speed:0.0055 });
  const alertId       = useRef(0);
  const detectedSet   = useRef(new Set());
  const timeStr       = useRef("00:00");
  const logId         = useRef(20);
  const wpRef         = useRef([]);
  const frameCount    = useRef(0);
  const mapBgCanvas   = useRef(null);
  const droneLockRef  = useRef(null);
  const standbyMeshes = useRef([]);
  const dispatchedSet = useRef(new Set());
  const scanLogRef    = useRef({});
  const zoneScores    = useRef({});
  const lastReroute   = useRef({});
  const evaluateRerouteRef = useRef(null); // avoids stale-closure in render loop

  /* ── State ── */
  const [elapsed,          setElapsed]          = useState(0);
  const [waypoints,        setWaypoints]        = useState([]);
  const [pathInfo,         setPathInfo]         = useState(null);
  const [noPath,           setNoPath]           = useState(false);
  const [alerts,           setAlerts]           = useState([]);
  const [log,              setLog]              = useState([
    { id:0, time:"00:00", msg:"RAVEN system online · All sensors active", type:"sys" },
    { id:1, time:"00:00", msg:"6 drones deployed · Zone patrol initiated", type:"sys" },
    { id:2, time:"00:00", msg:"A* pathfinding engine ready",              type:"sys" },
  ]);
  const [droneUI,          setDroneUI]          = useState(
    DRONE_CFG.map(d=>({ id:d.id, zone:d.zone, hex:d.hex, bat:d.bat, status:d.status, r:d.wpts[0].r, c:d.wpts[0].c }))
  );
  const [selectedDrone,    setSelectedDrone]    = useState(null);
  const [droneCtrl,        setDroneCtrl]        = useState({ x:0, z:0 });

  // ── Update 1 · Incidents ──
  const [incidents,        setIncidents]        = useState([]);

  // ── Update 2 · Dispatch ──
  const [dispatches,       setDispatches]       = useState([]);

  // ── Update 3 · Analysis / Heatmap ──
  const [showAnalysis,     setShowAnalysis]     = useState(false);
  const [heatmapOn,        setHeatmapOn]        = useState(false);
  const [heatmapMode,      setHeatmapMode]      = useState("risk");

  // ── Update 4 · Battery / RTB / Standby ──
  const [batteries,        setBatteries]        = useState(
    DRONE_CFG.reduce((acc,d)=>({ ...acc, [d.id]:d.bat }),{})
  );
  const [standbyDrones]                         = useState([
    { id:7, zone:"SB-1", hex:"#888888", status:"STANDBY" },
    { id:8, zone:"SB-2", hex:"#666666", status:"STANDBY" },
  ]);
  const [returningDrones,  setReturningDrones]  = useState([]);
  const [swapAnimations,   setSwapAnimations]   = useState([]);
  const [showBatteryPanel, setShowBatteryPanel] = useState(false);

  // ── Update 5 · AI Routing ──
  const [routingLog,       setRoutingLog]       = useState([]);
  const [showRoutingPanel, setShowRoutingPanel] = useState(false);

  /* ── Clock ── */
  useEffect(() => {
    let s = 0;
    const t = setInterval(() => {
      s++;
      timeStr.current = `${String(Math.floor(s/60)).padStart(2,"0")}:${String(s%60).padStart(2,"0")}`;
      setElapsed(s);
    }, 1000);
    return () => clearInterval(t);
  }, []);

  /* ── Helpers ── */
  const addAlert = useCallback((msg, type="info") => {
    const id = alertId.current++;
    setAlerts(p => [{ id, msg, type, time:timeStr.current }, ...p.slice(0,6)]);
    setTimeout(() => setAlerts(p => p.filter(a => a.id !== id)), 5500);
  }, []);

  const addLog = useCallback((msg, type="info") => {
    setLog(p => [{ id:logId.current++, time:timeStr.current, msg, type }, ...p.slice(0,79)]);
  }, []);

  /* ── Update 1 · ETA Calculator ── */
  const calcNearestETA = useCallback((tr, tc) => {
    let best = { eta:Infinity, droneId:null };
    dronesMesh.current.forEach((mesh, i) => {
      if (!mesh) return;
      const dr = Math.round(mesh.position.z / CS);
      const dc = Math.round(mesh.position.x / CS);
      const path = astar(MAP_DATA, dr, dc, tr, tc);
      if (!path.length) return;
      const etaSec = Math.ceil(path.length * CS / (DRONE_CFG[i].speed * 60));
      if (etaSec < best.eta) best = { eta:etaSec, droneId:DRONE_CFG[i].id };
    });
    return best;
  }, []);

  /* ── Update 2 · Dispatch ── */
  const dispatchDrone = useCallback((droneIdx, tr, tc, incidentKey) => {
    const mesh = dronesMesh.current[droneIdx];
    if (!mesh) return;
    const dr = Math.round(mesh.position.z / CS);
    const dc = Math.round(mesh.position.x / CS);
    const path = astar(MAP_DATA, dr, dc, tr, tc);
    if (!path.length) return;
    dronesAnim.current[droneIdx].path      = path;
    dronesAnim.current[droneIdx].t         = 0;
    dronesAnim.current[droneIdx].paused    = false;
    dronesAnim.current[droneIdx].dispatched = true;
    dispatchedSet.current.add(incidentKey);
    const cfg = DRONE_CFG[droneIdx];
    const msg = `D${cfg.id}[${cfg.zone}] DISPATCHED → R${tr} C${tc}`;
    addAlert(msg, "dispatch");
    addLog(msg, "dispatch");
    setDispatches(prev => [...prev, { droneIdx, targetKey:incidentKey, tr, tc }]);
    setIncidents(prev => prev.map(inc =>
      inc.id === incidentKey ? { ...inc, dispatched:true, dispatchedDrone:cfg.id } : inc
    ));
  }, [addAlert, addLog]);

  /* ── Camera ── */
  const updateCam = useCallback(() => {
    const cam = threeRef.current.camera; if (!cam) return;
    const { phi, theta, r, tx, tz } = camRef.current;
    const cx = (COLS-1)*CS/2+tx, cz = (ROWS-1)*CS/2+tz;
    cam.position.set(cx+r*Math.sin(phi)*Math.sin(theta), r*Math.cos(phi), cz+r*Math.sin(phi)*Math.cos(theta));
    cam.lookAt(cx, 0, cz);
  }, []);

  /* ── Update 3 · Heatmap Draw ── */
  const drawHeatmap = useCallback((ctx, W, H, mode) => {
    const cw = W/COLS, ch = H/ROWS;
    const now = Date.now();
    for (let r = 0; r < ROWS; r++) {
      for (let c = 0; c < COLS; c++) {
        if (MAP_DATA[r][c] === WALL) continue;
        let alpha = 0, color = "0,255,100";
        if (mode === "risk") {
          let hazardScore = 0;
          HAZARD_POS.forEach(([hr,hc]) => {
            const d = Math.abs(hr-r)+Math.abs(hc-c);
            if (d < 6) hazardScore += (6-d)/6;
          });
          const lastScan  = scanLogRef.current[`${r},${c}`];
          const staleness = lastScan ? Math.min(1,(now-lastScan)/30000) : 1;
          const risk      = Math.min(1, hazardScore*0.6 + staleness*0.4);
          alpha = risk * 0.55;
          if      (risk < 0.4) color = `0,255,${Math.round(255*(1-risk*2.5))}`;
          else if (risk < 0.7) color = `${Math.round(255*((risk-0.4)/0.3))},255,0`;
          else                  color = `255,${Math.round(255*(1-(risk-0.7)/0.3))},0`;
        } else if (mode === "coverage") {
          const lastScan = scanLogRef.current[`${r},${c}`];
          if (!lastScan) { alpha = 0.5; color = "80,80,200"; }
          else { const age=(now-lastScan)/20000; alpha=Math.min(0.45,age*0.45); color="0,150,255"; }
        } else if (mode === "traffic") {
          let count = 0;
          dronesMesh.current.forEach(mesh => {
            if (!mesh) return;
            const dr=Math.abs(Math.round(mesh.position.z/CS)-r);
            const dc=Math.abs(Math.round(mesh.position.x/CS)-c);
            if (dr+dc < 4) count++;
          });
          alpha = count * 0.18;
          color = `255,${Math.round(180-count*40)},0`;
        }
        if (alpha > 0.02) {
          ctx.fillStyle = `rgba(${color},${alpha.toFixed(2)})`;
          ctx.fillRect(c*cw, r*ch, cw, ch);
        }
      }
    }
    // Legend
    const labels = { risk:["LOW","MED","HIGH"], coverage:["SCANNED","","UNSEEN"], traffic:["SPARSE","","DENSE"] };
    const gradColors = {
      risk:["#00FF64","#FFFF00","#FF3300"],
      coverage:["#0096FF","#0050AA","#505080"],
      traffic:["#FFAA00","#FF6600","#FF2200"]
    };
    const lg = ctx.createLinearGradient(4, H-12, 84, H-12);
    gradColors[mode].forEach((col,i) => lg.addColorStop(i/2, col));
    ctx.fillStyle="rgba(0,0,0,0.55)"; ctx.fillRect(2,H-18,90,16);
    ctx.fillStyle=lg; ctx.fillRect(4,H-14,80,6);
    ctx.fillStyle="#AAAAAA"; ctx.font="5px monospace";
    labels[mode].forEach((l,i) => ctx.fillText(l, 4+i*28, H-4));
  }, []);

  /* ── Draw Minimap ── */
  const drawMinimap = useCallback(() => {
    const canvas = minimapRef.current; if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width, H = canvas.height;
    const cw = W/COLS, ch = H/ROWS;

    if (mapBgCanvas.current) ctx.drawImage(mapBgCanvas.current, 0, 0, W, H);
    else {
      ctx.fillStyle="#04080F"; ctx.fillRect(0,0,W,H);
      for (let r=0;r<ROWS;r++) for (let c=0;c<COLS;c++) {
        const cell=MAP_DATA[r][c];
        if      (cell===FLOOR)    ctx.fillStyle=r>=25?"#556677":"#B0C4D8";
        else if (cell===RUBBLE)   ctx.fillStyle="#6A5020";
        else if (cell===SURVIVOR) ctx.fillStyle="#00FF44";
        else if (cell===HAZARD)   ctx.fillStyle="#FF3333";
        else continue;
        ctx.fillRect(c*cw, r*ch, cw, ch);
      }
    }

    // Mission path
    const ma = missionAnim.current;
    if (ma.path.length > 1) {
      ctx.strokeStyle="#00FF88"; ctx.lineWidth=1.5; ctx.globalAlpha=0.85;
      ctx.beginPath();
      ma.path.forEach(([r,c],i) => {
        const px=c*cw+cw/2, py=r*ch+ch/2;
        i===0 ? ctx.moveTo(px,py) : ctx.lineTo(px,py);
      });
      ctx.stroke(); ctx.globalAlpha=1;
      wpRef.current.forEach((wp,i) => {
        const px=wp.c*cw+cw/2, py=wp.r*ch+ch/2;
        ctx.fillStyle=WP_COLORS[i]||"#FFFFFF";
        ctx.beginPath(); ctx.arc(px,py,3.5,0,Math.PI*2); ctx.fill();
        ctx.fillStyle="#FFFFFF"; ctx.font="7px monospace";
        ctx.fillText(i+1, px-2, py-5);
      });
    }

    // Update 3 · Heatmap overlay
    if (heatmapOn) drawHeatmap(ctx, W, H, heatmapMode);

    // Drone dots
    dronesMesh.current.forEach((mesh, i) => {
      if (!mesh) return;
      const { x, z } = mesh.position;
      const dc=x/CS, dr=z/CS;
      const px=dc*cw+cw/2, py=dr*ch+ch/2;
      const hex=DRONE_CFG[i].hex;
      ctx.shadowColor=hex; ctx.shadowBlur=5;
      ctx.fillStyle=hex;
      ctx.beginPath(); ctx.arc(px,py,2.5,0,Math.PI*2); ctx.fill();
      ctx.shadowBlur=0;
      ctx.fillStyle="#FFFFFF"; ctx.font="6px monospace";
      ctx.fillText(`D${i+1}`, px+3, py-3);
    });

    // Mission drone dot
    if (missionMesh.current && missionMesh.current.visible) {
      const { x, z } = missionMesh.current.position;
      const px=(x/CS)*cw+cw/2, py=(z/CS)*ch+ch/2;
      ctx.fillStyle="#FFFFFF"; ctx.shadowColor="#FFFFFF"; ctx.shadowBlur=6;
      ctx.beginPath(); ctx.arc(px,py,3.5,0,Math.PI*2); ctx.fill();
      ctx.shadowBlur=0;
    }
  }, [drawHeatmap, heatmapOn, heatmapMode]);

  /* ── Draw Camera Feeds (follows drone live position) ── */
  const drawFeeds = useCallback(() => {
    dronesMesh.current.forEach((mesh, i) => {
      const canvas = feedRefs.current[i]; if (!canvas || !mesh) return;
      const ctx = canvas.getContext("2d");
      const W = canvas.width, H = canvas.height;
      ctx.fillStyle="#030608"; ctx.fillRect(0,0,W,H);

      // Viewport centered on live drone position
      const cfg    = DRONE_CFG[i];
      const droneR = mesh.position.z / CS;
      const droneC = mesh.position.x / CS;
      const HALF_R = 5, HALF_C = 7;
      const r1 = Math.floor(droneR - HALF_R);
      const r2 = Math.floor(droneR + HALF_R);
      const c1 = Math.floor(droneC - HALF_C);
      const c2 = Math.floor(droneC + HALF_C);
      const zr = r2-r1+1, zc = c2-c1+1;
      const cw = W/zc, ch = H/zr;

      // Draw cells
      for (let r=r1; r<=r2; r++) {
        for (let c=c1; c<=c2; c++) {
          if (r<0||r>=ROWS||c<0||c>=COLS) continue;
          const cell=MAP_DATA[r][c];
          const px=(c-c1)*cw, py=(r-r1)*ch;
          if      (cell===WALL)     ctx.fillStyle="#0A0F15";
          else if (cell===FLOOR)    ctx.fillStyle="#0D1C1F";
          else if (cell===RUBBLE)   ctx.fillStyle="#1A1005";
          else if (cell===SURVIVOR) {
            const t=Date.now()*0.003;
            const grd=ctx.createRadialGradient(px+cw/2,py+ch/2,0,px+cw/2,py+ch/2,cw*1.2);
            grd.addColorStop(0,`rgba(0,255,80,${0.5+0.4*Math.sin(t)})`);
            grd.addColorStop(1,"rgba(0,255,80,0)");
            ctx.fillStyle=grd;
          } else if (cell===HAZARD) {
            const t=Date.now()*0.004;
            const grd=ctx.createRadialGradient(px+cw/2,py+ch/2,0,px+cw/2,py+ch/2,cw*1.2);
            grd.addColorStop(0,`rgba(255,50,30,${0.5+0.4*Math.sin(t+1)})`);
            grd.addColorStop(1,"rgba(255,50,30,0)");
            ctx.fillStyle=grd;
          } else ctx.fillStyle="#030608";
          ctx.fillRect(px, py, cw, ch);
        }
      }

      // Grid lines
      ctx.strokeStyle="rgba(0,150,160,0.08)"; ctx.lineWidth=0.5;
      for (let r=r1;r<=r2;r++){ ctx.beginPath();ctx.moveTo(0,(r-r1)*ch);ctx.lineTo(W,(r-r1)*ch);ctx.stroke(); }
      for (let c=c1;c<=c2;c++){ ctx.beginPath();ctx.moveTo((c-c1)*cw,0);ctx.lineTo((c-c1)*cw,H);ctx.stroke(); }

      // Drone dot
      const px=(droneC-c1)*cw+cw/2, py=(droneR-r1)*ch+ch/2;
      ctx.shadowColor=cfg.hex; ctx.shadowBlur=8;
      ctx.fillStyle=cfg.hex;
      ctx.beginPath(); ctx.arc(px,py,3,0,Math.PI*2); ctx.fill();
      ctx.shadowBlur=0;

      // Expanding scan ring
      const t2=Date.now()*0.002;
      const scanR=cw*1.8*((t2*0.4+i*0.6)%1);
      ctx.strokeStyle=cfg.hex;
      ctx.globalAlpha=0.35*(1-(scanR/(cw*1.8)));
      ctx.lineWidth=1.2;
      ctx.beginPath(); ctx.arc(px,py,scanR,0,Math.PI*2); ctx.stroke();
      ctx.globalAlpha=1;

      // Crosshair
      ctx.strokeStyle=cfg.hex; ctx.lineWidth=0.8; ctx.globalAlpha=0.6;
      const ch2=6;
      ctx.beginPath(); ctx.moveTo(px-ch2,py); ctx.lineTo(px+ch2,py); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(px,py-ch2); ctx.lineTo(px,py+ch2); ctx.stroke();
      ctx.globalAlpha=1;

      // Corner brackets (tactical HUD)
      const bS=5;
      ctx.strokeStyle=cfg.hex; ctx.lineWidth=1.5; ctx.globalAlpha=0.8;
      [[px-bS,py-bS,1,1],[px+bS,py-bS,-1,1],[px-bS,py+bS,1,-1],[px+bS,py+bS,-1,-1]].forEach(([bx,by,sx,sy])=>{
        ctx.beginPath(); ctx.moveTo(bx,by); ctx.lineTo(bx+sx*3,by); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(bx,by); ctx.lineTo(bx,by+sy*3); ctx.stroke();
      });
      ctx.globalAlpha=1;

      // Label top-left
      ctx.fillStyle=cfg.hex; ctx.font="bold 8px monospace";
      ctx.fillText(`D${cfg.id}·${cfg.zone}`, 3, 9);

      // Live coords bottom-right
      ctx.fillStyle="rgba(255,255,255,0.5)"; ctx.font="6px monospace";
      ctx.fillText(`R${Math.round(droneR)} C${Math.round(droneC)}`, W-38, H-3);

      // REC blink top-right
      if (Math.floor(Date.now()/700)%2===0) {
        ctx.fillStyle="#FF3333";
        ctx.beginPath(); ctx.arc(W-6,6,3,0,Math.PI*2); ctx.fill();
      }
    });
  }, []);

  /* ── Offscreen minimap bg ── */
  const buildMinimapBg = useCallback(() => {
    const oc=document.createElement("canvas");
    oc.width=252; oc.height=180;
    const ctx=oc.getContext("2d");
    const cw=oc.width/COLS, ch=oc.height/ROWS;
    ctx.fillStyle="#020508"; ctx.fillRect(0,0,oc.width,oc.height);
    for (let r=0;r<ROWS;r++) for (let c=0;c<COLS;c++) {
      const cell=MAP_DATA[r][c];
      if (cell===WALL) continue;
      if      (cell===FLOOR)    ctx.fillStyle=r>=25?"#445566":"#8BA5BE";
      else if (cell===RUBBLE)   ctx.fillStyle="#6A5020";
      else if (cell===SURVIVOR) ctx.fillStyle="#00FF44";
      else if (cell===HAZARD)   ctx.fillStyle="#FF3333";
      ctx.fillRect(c*cw, r*ch, cw-0.3, ch-0.3);
    }
    mapBgCanvas.current=oc;
  }, []);

  /* ── Draw Path 3D ── */
  const drawPath3D = useCallback((path) => {
    const pg=pathGrpRef.current; if(!pg)return;
    while(pg.children.length){const c=pg.children[0];c.geometry?.dispose();pg.remove(c);}
    if(!path||path.length<2)return;
    const pts=path.map(([r,c])=>new THREE.Vector3(c*CS,0.22,r*CS));
    const line=new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({color:0x00FF88,linewidth:2})
    );
    pg.add(line);
    const dg=new THREE.SphereGeometry(0.05,8,8);
    const dm=new THREE.MeshBasicMaterial({color:0x00FF88});
    path.forEach(([r,c],idx)=>{
      if(idx%4===0){const d=new THREE.Mesh(dg,dm);d.position.set(c*CS,0.22,r*CS);pg.add(d);}
    });
    const mi=Math.floor(path.length/2);
    const[mr,mc]=path[mi];
    const arr=new THREE.Mesh(new THREE.ConeGeometry(0.09,0.2,6),new THREE.MeshBasicMaterial({color:0x00FF88}));
    arr.position.set(mc*CS,0.35,mr*CS); pg.add(arr);
  }, []);

  /* ── Draw Waypoint Markers 3D ── */
  const drawMarkers3D = useCallback((wpts) => {
    const mg=markGrpRef.current; if(!mg)return;
    while(mg.children.length){const c=mg.children[0];c.geometry?.dispose();mg.remove(c);}
    const cg=new THREE.CylinderGeometry(0.16,0.16,0.06,18);
    const COLS3=[0x17D4FF,0xFF8800,0xFF44FF,0x44FFFF,0xFF4444,0x44FF44,0xFFFF44,0xFF8844];
    wpts.forEach((wp,i)=>{
      const m=new THREE.Mesh(cg,new THREE.MeshBasicMaterial({color:COLS3[i%COLS3.length]}));
      m.position.set(wp.c*CS,0.22,wp.r*CS); mg.add(m);
      const ring=new THREE.Mesh(
        new THREE.RingGeometry(0.2,0.28,20),
        new THREE.MeshBasicMaterial({color:COLS3[i%COLS3.length],side:THREE.DoubleSide,transparent:true,opacity:0.6})
      );
      ring.rotation.x=-Math.PI/2; ring.position.set(wp.c*CS,0.12,wp.r*CS); mg.add(ring);
    });
  }, []);

  /* ── Update 5 · Zone Scoring Engine ── */
  const scoreZone = useCallback((zoneIdx, currentIncidents) => {
    const cfg  = DRONE_CFG[zoneIdx];
    const wpts = cfg.wpts;
    const now  = Date.now();

    // Factor 1: unscanned density
    let unscanned=0, total=0;
    const r1=Math.min(...wpts.map(w=>w.r)), r2=Math.max(...wpts.map(w=>w.r));
    const c1=Math.min(...wpts.map(w=>w.c)), c2=Math.max(...wpts.map(w=>w.c));
    for (let r=r1;r<=r2;r++) for (let c=c1;c<=c2;c++) {
      if (MAP_DATA[r]?.[c]===WALL) continue;
      total++;
      const ls=scanLogRef.current[`${r},${c}`];
      if (!ls || now-ls > 15000) unscanned++;
    }
    const coverageScore = total>0 ? unscanned/total : 0;

    // Factor 2: active incidents nearby
    let incidentScore=0;
    currentIncidents.forEach(inc => {
      if (inc.dispatched) return;
      const inZone = inc.r>=r1-2&&inc.r<=r2+2 && inc.c>=c1-2&&inc.c<=c2+2;
      if (inZone) { const age=(now-inc.detectedAt)/1000; incidentScore+=Math.min(1,age/30); }
    });
    incidentScore=Math.min(1,incidentScore);

    // Factor 3: hazard proximity
    let hazardScore=0;
    HAZARD_POS.forEach(([hr,hc])=>{
      if (hr>=r1-3&&hr<=r2+3&&hc>=c1-3&&hc<=c2+3) hazardScore+=0.35;
    });
    hazardScore=Math.min(1,hazardScore);

    // Factor 4: staleness
    const droneLastScan = lastReroute.current[zoneIdx]
      ? Math.min(1,(now-lastReroute.current[zoneIdx])/20000) : 1;

    const score = coverageScore*0.30 + incidentScore*0.40 + hazardScore*0.15 + droneLastScan*0.15;
    return { score:parseFloat(score.toFixed(3)), breakdown:{ coverage:coverageScore, incident:incidentScore, hazard:hazardScore, staleness:droneLastScan } };
  }, []);

  /* ── Update 5 · Reroute Decision ── */
  const evaluateReroute = useCallback((droneIdx, currentIncidents) => {
    const now=Date.now();
    if (lastReroute.current[droneIdx] && now-lastReroute.current[droneIdx] < 6000) return;

    const scores=DRONE_CFG.map((_,i)=>({ zoneIdx:i, ...scoreZone(i, currentIncidents) }));

    // Update live UI scores
    const scoreMap={};
    DRONE_CFG.forEach((cfg,i)=>{ scoreMap[cfg.zone]=scores[i].score; });
    zoneScores.current=scoreMap;

    const ranked=[...scores].sort((a,b)=>b.score-a.score);
    const best=ranked[0];
    const current=scores[droneIdx];
    if (best.zoneIdx===droneIdx || best.score-current.score < 0.2) return;

    const targetCfg=DRONE_CFG[best.zoneIdx];
    const droneMesh=dronesMesh.current[droneIdx];
    if (!droneMesh) return;

    const dr=Math.round(droneMesh.position.z/CS);
    const dc=Math.round(droneMesh.position.x/CS);
    const crossPath=astar(MAP_DATA,dr,dc,targetCfg.wpts[0].r,targetCfg.wpts[0].c);
    if (!crossPath||crossPath.length<2) return;

    const zoneLoop=[...targetCfg.wpts,targetCfg.wpts[0]];
    const zonePath=chainPaths(MAP_DATA,zoneLoop);
    if (!zonePath) return;

    dronesAnim.current[droneIdx].path=[...crossPath,...zonePath.slice(1)];
    dronesAnim.current[droneIdx].t=0;
    lastReroute.current[droneIdx]=now;

    const sourceCfg=DRONE_CFG[droneIdx];
    const reason=best.breakdown.incident>0.3?"INCIDENT PRIORITY"
               : best.breakdown.coverage>0.6?"COVERAGE GAP"
               : "HAZARD PROXIMITY";

    const entry={ id:now, time:timeStr.current, droneId:sourceCfg.id,
      from:sourceCfg.zone, to:targetCfg.zone, score:best.score.toFixed(2),
      reason, breakdown:best.breakdown };

    setRoutingLog(prev=>[entry,...prev.slice(0,19)]);
    addLog(`AI · D${sourceCfg.id} rerouted ${sourceCfg.zone}→${targetCfg.zone} [${reason}]`,"sys");
    addAlert(`D${sourceCfg.id} reassigned to Zone ${targetCfg.zone} · ${reason}`,"dispatch");
  }, [scoreZone, addLog, addAlert]);

  // Keep ref current so render loop can call it without being a dependency
  evaluateRerouteRef.current = evaluateReroute;

  /* ══════════════════════════════════════════════════════════════
     SCENE INIT
  ══════════════════════════════════════════════════════════════ */
  useEffect(() => {
    if (!mountRef.current) return;
    const el=mountRef.current;
    const W=el.clientWidth, H=el.clientHeight;

    const renderer=new THREE.WebGLRenderer({antialias:true});
    renderer.setSize(W,H);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));
    renderer.shadowMap.enabled=true;
    el.appendChild(renderer.domElement);

    const scene=new THREE.Scene();
    scene.background=new THREE.Color(0x04080F);
    scene.fog=new THREE.FogExp2(0x04080F,0.02);

    const camera=new THREE.PerspectiveCamera(48,W/H,0.1,80);
    threeRef.current={renderer,scene,camera};
    updateCam();

    scene.add(new THREE.AmbientLight(0x88AACC,0.5));
    const sun=new THREE.DirectionalLight(0xffffff,0.9);
    sun.position.set(14,22,10); sun.castShadow=true; scene.add(sun);
    const fill=new THREE.DirectionalLight(0x3388BB,0.3);
    fill.position.set(-10,15,-10); scene.add(fill);

    const gnd=new THREE.Mesh(
      new THREE.PlaneGeometry(COLS*CS+4,ROWS*CS+4),
      new THREE.MeshLambertMaterial({color:0x020508})
    );
    gnd.rotation.x=-Math.PI/2;
    gnd.position.set((COLS-1)*CS/2,-0.02,(ROWS-1)*CS/2);
    gnd.receiveShadow=true; scene.add(gnd);

    const mats={
      wall: new THREE.MeshLambertMaterial({color:0x1A3055}),
      floor:new THREE.MeshLambertMaterial({color:0xB0C4D8}),
      road: new THREE.MeshLambertMaterial({color:0x6A7A8A}),
      rub:  new THREE.MeshLambertMaterial({color:0x6A5020}),
      surv: new THREE.MeshLambertMaterial({color:0x00CC44,emissive:0x003310}),
      haz:  new THREE.MeshLambertMaterial({color:0xCC2222,emissive:0x330000}),
    };
    const geos={
      wall: new THREE.BoxGeometry(CS*0.97,0.62,CS*0.97),
      fl:   new THREE.BoxGeometry(CS*0.96,0.04,CS*0.96),
      surv: new THREE.CylinderGeometry(0.08,0.08,0.38,8),
      haz:  new THREE.CylinderGeometry(0.08,0.11,0.32,6),
      rub:  new THREE.BoxGeometry(0.18,0.12,0.18),
      ring: new THREE.RingGeometry(0.13,0.18,16),
    };
    const ringSurvMat=new THREE.MeshBasicMaterial({color:0x00FF44,side:THREE.DoubleSide,transparent:true,opacity:0.6});
    const ringHazMat =new THREE.MeshBasicMaterial({color:0xFF4444,side:THREE.DoubleSide,transparent:true,opacity:0.6});

    for (let r=0;r<ROWS;r++) for (let c=0;c<COLS;c++) {
      const cell=MAP_DATA[r][c];
      const x=c*CS, z=r*CS;
      if (cell===WALL) {
        const m=new THREE.Mesh(geos.wall,mats.wall); m.position.set(x,0.31,z); m.castShadow=true; scene.add(m);
      } else {
        const fl=new THREE.Mesh(geos.fl,r>=25?mats.road:mats.floor); fl.position.set(x,0,z); fl.receiveShadow=true; scene.add(fl);
        if (cell===SURVIVOR) {
          const s=new THREE.Mesh(geos.surv,mats.surv); s.position.set(x,0.21,z); scene.add(s);
          const ring=new THREE.Mesh(geos.ring,ringSurvMat.clone()); ring.rotation.x=-Math.PI/2; ring.position.set(x,0.06,z); ring.userData.pulse=true; ring.userData.pulsePhase=r*c*0.3; scene.add(ring);
        } else if (cell===HAZARD) {
          const h=new THREE.Mesh(geos.haz,mats.haz); h.position.set(x,0.18,z); scene.add(h);
          const ring=new THREE.Mesh(geos.ring,ringHazMat.clone()); ring.rotation.x=-Math.PI/2; ring.position.set(x,0.06,z); ring.userData.pulse=true; ring.userData.pulsePhase=r*c*0.5+1; scene.add(ring);
        } else if (cell===RUBBLE) {
          const rb=new THREE.Mesh(geos.rub,mats.rub); rb.position.set(x,0.08,z); rb.rotation.y=(r*7+c*13)*0.4; scene.add(rb);
        }
      }
    }

    const pg=new THREE.Group(), mg=new THREE.Group();
    scene.add(pg); scene.add(mg);
    pathGrpRef.current=pg; markGrpRef.current=mg;

    // Patrol drones
    const droneSphGeo=new THREE.SphereGeometry(0.14,14,14);
    DRONE_CFG.forEach((cfg,i)=>{
      const mat=new THREE.MeshPhongMaterial({color:cfg.col,emissive:cfg.col,emissiveIntensity:0.35,shininess:140});
      const mesh=new THREE.Mesh(droneSphGeo,mat);
      const light=new THREE.PointLight(cfg.col,1.5,2.5);
      mesh.add(light);
      mesh.position.set(cfg.wpts[0].c*CS,0.38,cfg.wpts[0].r*CS);
      scene.add(mesh);
      dronesMesh.current[i]=mesh;
    });

    // Update 4 · Standby drones at base
    const sbColors=[0x888888,0x666666];
    sbColors.forEach((col,i)=>{
      const mat=new THREE.MeshPhongMaterial({color:col,emissive:col,emissiveIntensity:0.15,shininess:80});
      const sbMesh=new THREE.Mesh(droneSphGeo,mat);
      sbMesh.position.set((BASE_POS.c+i*1.5)*CS,0.28,BASE_POS.r*CS);
      sbMesh.userData.isStandby=true;
      sbMesh.userData.standbyIdx=i;
      scene.add(sbMesh);
      standbyMeshes.current[i]=sbMesh;
    });

    // Base pad
    const basePad=new THREE.Mesh(
      new THREE.CylinderGeometry(0.6,0.6,0.03,24),
      new THREE.MeshBasicMaterial({color:0x17D4FF,transparent:true,opacity:0.25})
    );
    basePad.position.set(BASE_POS.c*CS,0.01,BASE_POS.r*CS);
    scene.add(basePad);

    const baseRing=new THREE.Mesh(
      new THREE.RingGeometry(0.65,0.85,28),
      new THREE.MeshBasicMaterial({color:0x17D4FF,side:THREE.DoubleSide,transparent:true,opacity:0.5})
    );
    baseRing.rotation.x=-Math.PI/2;
    baseRing.position.set(BASE_POS.c*CS,0.02,BASE_POS.r*CS);
    baseRing.userData.pulse=true; baseRing.userData.pulsePhase=0;
    scene.add(baseRing);

    // Mission drone
    const missionGeo=new THREE.SphereGeometry(0.17,16,16);
    const missionMat=new THREE.MeshPhongMaterial({color:0xFFFFFF,emissive:0x17D4FF,emissiveIntensity:0.5,shininess:200});
    const mDrone=new THREE.Mesh(missionGeo,missionMat);
    const mLight=new THREE.PointLight(0x17D4FF,2.5,4);
    mDrone.add(mLight); mDrone.visible=false; scene.add(mDrone);
    missionMesh.current=mDrone;

    buildMinimapBg();

    /* ── Render Loop ── */
    const clock=new THREE.Clock();
    let fid;

    // We hold a local snapshot of incidents for the AI reroute (avoids React stale closure)
    let incidentsSnapshot = [];
    const updateIncidentsSnapshot = (v) => { incidentsSnapshot = v; };

    const loop = () => {
      fid=requestAnimationFrame(loop);
      const t=clock.getElapsedTime();
      frameCount.current++;

      // Pulse rings
      scene.traverse(obj=>{
        if(obj.userData.pulse){
          const ph=obj.userData.pulsePhase||0;
          const s=0.8+0.28*Math.sin(t*2.6+ph);
          obj.scale.set(s,s,s);
          obj.material.opacity=0.25+0.45*Math.sin(t*2.4+ph);
        }
      });

      // ── Patrol drones ──
      const posUpdates=[];
      dronesAnim.current.forEach((anim,i)=>{
        const path=anim.path;
        if(!path||path.length<2||anim.paused)return;
        anim.t=(anim.t+anim.speed)%(path.length-1);
        const idx=Math.floor(anim.t);
        const frac=anim.t-idx;
        const[r1,c1]=path[idx];
        const[r2,c2]=path[Math.min(idx+1,path.length-1)];
        const x=c1*CS+frac*(c2-c1)*CS;
        const z=r1*CS+frac*(r2-r1)*CS;
        const mesh=dronesMesh.current[i];
        if(mesh){
          mesh.position.set(x,0.38+Math.sin(t*3.2+i*1.3)*0.022,z);
          mesh.rotation.y=t*1.1+i*0.8;
          // Scan log for coverage tracking
          const sr=Math.round(z/CS), sc=Math.round(x/CS);
          for(let dr2=-2;dr2<=2;dr2++)
            for(let dc2=-2;dc2<=2;dc2++)
              scanLogRef.current[`${sr+dr2},${sc+dc2}`]=Date.now();
        }

        // Update 4 · Battery drain every ~180 frames
        if(frameCount.current%180===i*30){
          setBatteries(prev=>{
            const cur=prev[DRONE_CFG[i].id];
            if(cur<=0||anim.rtb)return prev;
            const next=Math.max(0,cur-1);
            if(next<=20&&!anim.rtb){
              anim.rtb=true;
              const rtbPath=astar(MAP_DATA,Math.round(z/CS),Math.round(x/CS),BASE_POS.r,BASE_POS.c);
              if(rtbPath.length>1){
                anim.path=rtbPath; anim.t=0;
                const cfg=DRONE_CFG[i];
                addLog(`D${cfg.id}[${cfg.zone}] RTB · Battery critical`,"warn");
                addAlert(`D${cfg.id} returning to base`,"warn");
                setReturningDrones(p=>[...p,cfg.id]);
              }
            }
            return{...prev,[DRONE_CFG[i].id]:next};
          });
        }

        // Update 4 · RTB dock check
        if(anim.rtb){
          const distToBase=Math.abs(Math.round(z/CS)-BASE_POS.r)+Math.abs(Math.round(x/CS)-BASE_POS.c);
          if(distToBase<=1&&!anim.docked){
            anim.docked=true; anim.paused=true;
            const mesh2=dronesMesh.current[i];
            if(mesh2)mesh2.visible=false;
            const sbMesh=standbyMeshes.current[0];
            if(sbMesh){
              sbMesh.userData.launching=true;
              sbMesh.userData.launchStart=Date.now();
              sbMesh.userData.launchTarget={path:PATROL_PATHS[i],droneIdx:i,startPos:{x:sbMesh.position.x,y:sbMesh.position.y,z:sbMesh.position.z}};
            }
            setSwapAnimations(p=>[...p,{droneId:DRONE_CFG[i].id,startedAt:Date.now()}]);
            addLog(`D${DRONE_CFG[i].id} DOCKED · Standby unit deploying`,"sys");
            addAlert(`SB-1 launching → Zone ${DRONE_CFG[i].zone}`,"dispatch");
            setTimeout(()=>{
              setBatteries(p=>({...p,[DRONE_CFG[i].id]:100}));
              anim.rtb=false; anim.docked=false; anim.paused=false;
              anim.path=PATROL_PATHS[i]; anim.t=0;
              if(mesh2){mesh2.visible=true;mesh2.position.set(BASE_POS.c*CS,0.38,BASE_POS.r*CS);}
              if(sbMesh){sbMesh.userData.launching=false;sbMesh.position.set(BASE_POS.c*CS,0.28,BASE_POS.r*CS);}
              setReturningDrones(p=>p.filter(id=>id!==DRONE_CFG[i].id));
              setSwapAnimations(p=>p.filter(s=>s.droneId!==DRONE_CFG[i].id));
              addLog(`D${DRONE_CFG[i].id} recharged · Resuming patrol`,"sys");
            },8000);
          }
        }

        // Update 5 · AI reroute evaluation (staggered per drone, every ~240 frames)
        if(!anim.rtb&&!anim.paused&&!anim.dispatched){
          if(frameCount.current%240===i*40){
            if(evaluateRerouteRef.current) evaluateRerouteRef.current(i, incidentsSnapshot);
          }
        }

        posUpdates.push({i,r:Math.round(r1+frac*(r2-r1)),c:Math.round(c1+frac*(c2-c1))});
      });

      // Update 4 · Standby launch animation
      standbyMeshes.current.forEach(sbMesh=>{
        if(!sbMesh?.userData.launching)return;
        const lt=sbMesh.userData.launchTarget; if(!lt)return;
        const elapsed2=(Date.now()-(sbMesh.userData.launchStart||Date.now()))/1000;
        const liftPhase=Math.min(1,elapsed2/1.5);
        const travelPhase=Math.max(0,Math.min(1,(elapsed2-1.5)/3));
        const targetX=lt.path[0][1]*CS, targetZ=lt.path[0][0]*CS;
        sbMesh.position.set(
          lt.startPos.x+(targetX-lt.startPos.x)*travelPhase,
          0.28+liftPhase*0.6-travelPhase*0.2,
          lt.startPos.z+(targetZ-lt.startPos.z)*travelPhase
        );
        sbMesh.rotation.y=t*2.5;
        sbMesh.material.color.setHex(travelPhase>0?0x17D4FF:0xAAAAAA);
        sbMesh.material.emissive.setHex(travelPhase>0?0x0088AA:0x444444);
      });

      // Update 2 · Dispatched drone pulsing cyan light
      dronesAnim.current.forEach((anim,i)=>{
        if(!anim.dispatched)return;
        const mesh=dronesMesh.current[i]; if(!mesh)return;
        mesh.children.forEach(child=>{
          if(child.isPointLight){
            child.intensity=2.5+Math.sin(t*8)*1.5;
            child.color.setHex(0x17D4FF);
          }
        });
      });

      // Mission drone
      const ma=missionAnim.current;
      if(ma.on&&ma.path.length>1&&missionMesh.current){
        ma.t=(ma.t+ma.speed)%(ma.path.length-1);
        const idx=Math.floor(ma.t);
        const frac=ma.t-idx;
        const[r1,c1]=ma.path[idx];
        const[r2,c2]=ma.path[Math.min(idx+1,ma.path.length-1)];
        missionMesh.current.position.set(
          c1*CS+frac*(c2-c1)*CS,
          0.45+Math.sin(t*4)*0.015,
          r1*CS+frac*(r2-r1)*CS
        );
        missionMesh.current.rotation.y=t*2;
      }

      if(frameCount.current%3===0){ drawMinimap(); drawFeeds(); }

      if(frameCount.current%60===0&&posUpdates.length>0){
        setDroneUI(prev=>prev.map(d=>{
          const upd=posUpdates.find(u=>u.i===d.id-1);
          if(!upd)return d;
          return{...d,r:upd.r,c:upd.c};
        }));
      }

      renderer.render(scene,camera);
    };
    loop();

    // Subscribe to incidents changes for reroute engine
    const incidentUnsub = (v) => { updateIncidentsSnapshot(v); };
    // We expose this so the detection effect can update it
    window.__ravenUpdateIncidents = incidentUnsub;

    const onResize=()=>{
      if(!el)return;
      const W2=el.clientWidth,H2=el.clientHeight;
      renderer.setSize(W2,H2);
      camera.aspect=W2/H2; camera.updateProjectionMatrix();
    };
    window.addEventListener("resize",onResize);
    return()=>{ cancelAnimationFrame(fid); window.removeEventListener("resize",onResize); renderer.dispose(); el.innerHTML=""; delete window.__ravenUpdateIncidents; };
  }, [updateCam, drawMinimap, drawFeeds, buildMinimapBg, addLog, addAlert]);

  /* ── Update 1 · Detection Check ── */
  useEffect(() => {
    // Keep render loop snapshot in sync
    if (window.__ravenUpdateIncidents) window.__ravenUpdateIncidents(incidents);
  }, [incidents]);

  useEffect(() => {
    const interval=setInterval(()=>{
      dronesMesh.current.forEach((mesh,di)=>{
        if(!mesh)return;
        const dr=Math.round(mesh.position.z/CS);
        const dc=Math.round(mesh.position.x/CS);
        const zone=DRONE_CFG[di].zone;

        SURVIVOR_POS.forEach((pos,si)=>{
          const key=`S${si}`;
          if(detectedSet.current.has(key))return;
          const dist=Math.abs(pos[0]-dr)+Math.abs(pos[1]-dc);
          if(dist<=2){
            detectedSet.current.add(key);
            const msg=`D${di+1}[${zone}] · Survivor ${si+1} detected at R${pos[0]} C${pos[1]}`;
            addAlert(msg,"survivor"); addLog(msg,"survivor");
            const{eta,droneId}=calcNearestETA(pos[0],pos[1]);
            setIncidents(prev=>[...prev,{
              id:key, label:`Survivor ${si+1}`,
              r:pos[0], c:pos[1], detectedAt:Date.now(),
              eta, droneId, type:"survivor"
            }]);
          }
        });

        HAZARD_POS.forEach((pos,hi)=>{
          const key=`H${hi}`;
          if(detectedSet.current.has(key))return;
          const dist=Math.abs(pos[0]-dr)+Math.abs(pos[1]-dc);
          if(dist<=2){
            detectedSet.current.add(key);
            const msg=`D${di+1}[${zone}] · HAZARD ${hi+1} confirmed at R${pos[0]} C${pos[1]}`;
            addAlert(msg,"hazard"); addLog(msg,"hazard");
          }
        });
      });
    },600);
    return()=>clearInterval(interval);
  },[addAlert,addLog,calcNearestETA]);

  /* ── WASD / Arrow key control ── */
  useEffect(()=>{
    const STEP=CS*0.8;
    const onKey=(e)=>{
      const idx=droneLockRef.current;
      if(idx===null||idx===undefined)return;
      const mesh=dronesMesh.current[idx]; if(!mesh)return;
      dronesAnim.current[idx].paused=true;
      const moves={
        ArrowUp:[0,-STEP],w:[0,-STEP],W:[0,-STEP],
        ArrowDown:[0,STEP],s:[0,STEP],S:[0,STEP],
        ArrowLeft:[-STEP,0],a:[-STEP,0],A:[-STEP,0],
        ArrowRight:[STEP,0],d:[STEP,0],D:[STEP,0],
      };
      const mv=moves[e.key]; if(!mv)return;
      e.preventDefault();
      const nx=Math.max(0,Math.min((COLS-1)*CS,mesh.position.x+mv[0]));
      const nz=Math.max(0,Math.min((ROWS-1)*CS,mesh.position.z+mv[1]));
      const nr=Math.round(nz/CS),nc=Math.round(nx/CS);
      if(MAP_DATA[nr]?.[nc]===WALL||MAP_DATA[nr]?.[nc]===RUBBLE)return;
      mesh.position.x=nx; mesh.position.z=nz;
      setDroneCtrl({x:nx,z:nz});
    };
    window.addEventListener("keydown",onKey);
    return()=>window.removeEventListener("keydown",onKey);
  },[]);

  /* ── Map Click ── */
  const handleClick=useCallback((e)=>{
    if(mouseRef.current.moved)return;
    const{renderer,camera}=threeRef.current; if(!renderer)return;
    const rect=mountRef.current.getBoundingClientRect();
    const ray=new THREE.Raycaster();
    ray.setFromCamera({
      x:((e.clientX-rect.left)/rect.width)*2-1,
      y:-((e.clientY-rect.top)/rect.height)*2+1
    },camera);
    const pt=new THREE.Vector3();
    if(!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0,1,0),0),pt))return;
    const c=Math.round(pt.x/CS),r=Math.round(pt.z/CS);
    if(r<0||r>=ROWS||c<0||c>=COLS)return;
    const cell=MAP_DATA[r][c];
    if(cell===WALL||cell===RUBBLE)return;
    const newWp={r,c};
    const updated=[...wpRef.current,newWp];
    wpRef.current=updated;
    setWaypoints([...updated]);
    drawMarkers3D(updated);
    addLog(`Waypoint ${updated.length} set · R${r} C${c}`,"wp");
    if(updated.length>=2){
      const result=chainPaths(MAP_DATA,updated);
      if(result&&result.length>1){
        drawPath3D(result);
        missionAnim.current={t:0,path:result,on:true,speed:0.0055};
        if(missionMesh.current){missionMesh.current.visible=true;missionMesh.current.position.set(updated[0].c*CS,0.45,updated[0].r*CS);}
        const hazOnPath=result.some(([pr,pc])=>MAP_DATA[pr][pc]===HAZARD);
        setPathInfo({steps:result.length,dist:(result.length*CS*1.1).toFixed(1),eta:Math.ceil(result.length*0.18),safe:!hazOnPath,wpts:updated.length});
        setNoPath(false);
        addLog(`Route computed · ${result.length} steps · ${updated.length} waypoints`,"route");
      } else {
        setNoPath(true);setPathInfo(null);drawPath3D(null);
        missionAnim.current.on=false;
        if(missionMesh.current)missionMesh.current.visible=false;
        addLog("No valid path — destination blocked","warn");
      }
    }
  },[drawMarkers3D,drawPath3D,addLog]);

  const resetAll=useCallback(()=>{
    wpRef.current=[];setWaypoints([]);setPathInfo(null);setNoPath(false);
    drawPath3D(null);drawMarkers3D([]);
    missionAnim.current={t:0,path:[],on:false,speed:0.0055};
    if(missionMesh.current)missionMesh.current.visible=false;
    addLog("Route cleared · Click map to add waypoints","sys");
  },[drawPath3D,drawMarkers3D,addLog]);

  /* ── Mouse ── */
  const onDown=e=>{mouseRef.current={down:true,x:e.clientX,y:e.clientY,btn:e.button,moved:false};};
  const onUp  =e=>{if(!mouseRef.current.moved)handleClick(e);mouseRef.current.down=false;};
  const onMove=e=>{
    if(!mouseRef.current.down)return;
    const dx=e.clientX-mouseRef.current.x, dy=e.clientY-mouseRef.current.y;
    if(Math.abs(dx)+Math.abs(dy)>3)mouseRef.current.moved=true;
    mouseRef.current.x=e.clientX; mouseRef.current.y=e.clientY;
    const st=camRef.current;
    if(mouseRef.current.btn===0){st.theta-=dx*0.007;st.phi=Math.max(0.14,Math.min(1.45,st.phi+dy*0.007));}
    else{st.tx-=dx*0.04*Math.cos(st.theta);st.tz-=dx*0.04*Math.sin(st.theta);st.tx+=dy*0.04*Math.sin(st.theta);st.tz-=dy*0.04*Math.cos(st.theta);}
    updateCam();
  };
  const onWheel=e=>{camRef.current.r=Math.max(4,Math.min(34,camRef.current.r+e.deltaY*0.02));updateCam();};

  /* ── Styles & colors ── */
  const fmtT=s=>`${String(Math.floor(s/60)).padStart(2,"0")}:${String(s%60).padStart(2,"0")}`;
  const P={fontFamily:"'Courier New',monospace",fontSize:9,color:"#3A6A8A"};
  const alertColors={
    survivor:{bg:"#001A08",border:"#00AA44",label:"#00FF44"},
    hazard:  {bg:"#1A0002",border:"#AA1122",label:"#FF3344"},
    info:    {bg:"#001522",border:"#005A8A",label:"#17D4FF"},
    dispatch:{bg:"#0A0F1A",border:"#17D4FF55",label:"#17D4FF"},
    warn:    {bg:"#1A0A00",border:"#AA6600",label:"#FF8800"},
  };
  const logColors={survivor:"#00FF44",hazard:"#FF4444",route:"#00FF88",wp:"#17D4FF",sys:"#3A6A8A",warn:"#FF8800",info:"#3A6A8A",dispatch:"#17D4FF"};

  /* ══════════════════════════════════════════════════════════════
     RENDER
  ══════════════════════════════════════════════════════════════ */
  return (
    <div style={{display:"flex",height:"100vh",background:"#02050A",overflow:"hidden",fontFamily:"'Courier New',monospace"}}>

      {/* ══ LEFT PANEL ══ */}
      <div style={{width:185,background:"#060C16",borderRight:"1px solid #0A1828",display:"flex",flexDirection:"column",overflowY:"auto",flexShrink:0}}>

        {/* Header */}
        <div style={{padding:"10px 12px 8px",borderBottom:"1px solid #0A1828",background:"#030709"}}>
          <div style={{color:"#0096A0",fontSize:16,fontWeight:"bold",letterSpacing:6}}>RAVEN</div>
          <div style={{...P,letterSpacing:3,marginTop:1}}>PROTOTYPE v3</div>
          <div style={{display:"flex",alignItems:"center",gap:7,marginTop:6}}>
            <div style={{width:6,height:6,borderRadius:"50%",background:"#00FF88",boxShadow:"0 0 7px #00FF88"}}/>
            <span style={{...P,color:"#00CC66",letterSpacing:2}}>LIVE</span>
            <span style={{...P,marginLeft:"auto",color:"#17D4FF"}}>{fmtT(elapsed)}</span>
          </div>
        </div>

        {/* Waypoint routing */}
        <div style={{padding:"9px 12px",borderBottom:"1px solid #0A1828"}}>
          <div style={{...P,color:"#0096A0",letterSpacing:3,marginBottom:7}}>MULTI-WAYPOINT ROUTE</div>
          {waypoints.length===0&&(
            <div style={{...P,color:"#1A3A5A",marginBottom:6,fontSize:9.5}}>◈ Click any floor tile to place waypoints. Path auto-computes after 2nd point.</div>
          )}
          {waypoints.map((wp,i)=>(
            <div key={i} style={{display:"flex",alignItems:"center",gap:5,marginBottom:4,padding:"4px 6px",background:"#04090F",border:`1px solid ${WP_COLORS[i]||"#1A3A5A"}`,borderRadius:3}}>
              <div style={{width:6,height:6,borderRadius:"50%",background:WP_COLORS[i]||"#FFFFFF",boxShadow:`0 0 5px ${WP_COLORS[i]||"#FFFFFF"}`}}/>
              <span style={{...P,fontSize:9,color:WP_COLORS[i]||"#FFFFFF"}}>WP{i+1}</span>
              <span style={{...P,marginLeft:"auto",fontSize:8.5}}>R{wp.r}·C{wp.c}</span>
            </div>
          ))}
          {waypoints.length>0&&(
            <button onClick={resetAll} style={{width:"100%",background:"#04090F",border:"1px solid #1A3A5A",color:"#3A6A8A",padding:"4px 0",borderRadius:3,cursor:"pointer",...P,letterSpacing:2,fontSize:9,marginTop:3}}>
              [ CLEAR ROUTE ]
            </button>
          )}
        </div>

        {/* Path info */}
        {pathInfo&&(
          <div style={{padding:"8px 12px",borderBottom:"1px solid #0A1828",background:"#010A04"}}>
            <div style={{...P,color:"#00FF88",letterSpacing:2,marginBottom:6}}>▸ ROUTE ACTIVE</div>
            {[["WAYPOINTS",pathInfo.wpts],["STEPS",pathInfo.steps],["DISTANCE",`${pathInfo.dist}m`],["ETA",`${pathInfo.eta}s`],["STATUS",pathInfo.safe?"SAFE ✓":"HAZARD ⚠"]].map(([k,v])=>(
              <div key={k} style={{display:"flex",justifyContent:"space-between",marginBottom:3}}>
                <span style={P}>{k}</span>
                <span style={{...P,color:k==="STATUS"?(pathInfo.safe?"#00FF88":"#FF8800"):"#00FF88",fontWeight:"bold",fontSize:9.5}}>{v}</span>
              </div>
            ))}
          </div>
        )}
        {noPath&&(
          <div style={{padding:"7px 12px",background:"#0D0204",borderBottom:"1px solid #3A0505"}}>
            <div style={{...P,color:"#FF4444",letterSpacing:1}}>✗ NO PATH FOUND</div>
          </div>
        )}

        {/* Minimap */}
        <div style={{padding:"9px 12px",borderBottom:"1px solid #0A1828"}}>
          <div style={{...P,color:"#0096A0",letterSpacing:3,marginBottom:6}}>2D OVERVIEW</div>
          <canvas ref={minimapRef} width={161} height={115} style={{display:"block",border:"1px solid #0A1828",background:"#020508"}}/>
          <div style={{...P,fontSize:8,marginTop:4,opacity:0.7}}>● Drones &nbsp;— Path &nbsp;◈ Waypoints</div>
        </div>

        {/* Swarm status */}
        <div style={{padding:"9px 12px"}}>
          <div style={{...P,color:"#0096A0",letterSpacing:3,marginBottom:7}}>SWARM  6/6</div>
          {droneUI.map(d=>(
            <div key={d.id}
              onClick={()=>{const idx=d.id-1;setSelectedDrone(idx);droneLockRef.current=idx;}}
              style={{marginBottom:5,padding:"4px 6px",background:selectedDrone===d.id-1?"#050E18":"#030709",border:`1px solid ${selectedDrone===d.id-1?d.hex:"#0A1828"}`,borderRadius:2,cursor:"pointer",transition:"border-color 0.2s"}}>
              <div style={{display:"flex",alignItems:"center",gap:5,marginBottom:2.5}}>
                <div style={{width:5,height:5,borderRadius:"50%",background:d.hex,boxShadow:`0 0 4px ${d.hex}`,flexShrink:0}}/>
                <span style={{...P,fontSize:9,color:d.hex}}>D{d.id}/{d.zone}</span>
                <span style={{...P,fontSize:8,marginLeft:"auto",color:d.status==="LOW BAT"?"#FF8800":"#3A6A8A"}}>{d.status}</span>
              </div>
              <div style={{display:"flex",alignItems:"center",gap:4}}>
                <div style={{flex:1,height:3,background:"#0A1828",borderRadius:2}}>
                  <div style={{width:`${batteries[d.id]??d.bat}%`,height:"100%",background:(batteries[d.id]??d.bat)<50?"#FF8800":"#00CC44",borderRadius:2}}/>
                </div>
                <span style={{...P,fontSize:8,width:22,textAlign:"right"}}>{batteries[d.id]??d.bat}%</span>
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* ══ 3D CANVAS ══ */}
      <div style={{flex:1,position:"relative",minWidth:0}}>
        <div ref={mountRef} style={{width:"100%",height:"100%",cursor:"crosshair"}}
          onMouseDown={onDown} onMouseUp={onUp} onMouseMove={onMove}
          onWheel={onWheel} onContextMenu={e=>e.preventDefault()}/>

        {/* Top HUD */}
        <div style={{position:"absolute",top:10,left:10,right:10,display:"flex",justifyContent:"space-between",pointerEvents:"none",gap:8,flexWrap:"wrap"}}>
          <div style={{background:"rgba(2,5,10,0.92)",border:"1px solid #0A1828",borderRadius:4,padding:"5px 12px"}}>
            <span style={{color:"#0096A0",fontWeight:"bold",fontSize:13,letterSpacing:5}}>RAVEN</span>
            <span style={{...P,fontSize:9.5,marginLeft:10}}>EMERGENCY RESPONSE AI · v3</span>
          </div>
          <div style={{background:"rgba(2,5,10,0.92)",border:"1px solid #0A1828",borderRadius:4,padding:"5px 10px",display:"flex",gap:12}}>
            {[["ROOMS","6"],["GRID",`${COLS}×${ROWS}`],["ALGO","A* 8-DIR"],["DRONES","6"]].map(([k,v])=>(
              <span key={k} style={{...P,fontSize:9}}>{k} <b style={{color:"#17D4FF"}}>{v}</b></span>
            ))}
          </div>

          {/* ── Update 3 · ANALYSIS button ── */}
          <div style={{position:"relative",pointerEvents:"all"}}>
            <button onClick={()=>setShowAnalysis(p=>!p)} style={{background:showAnalysis?"#001A2A":"transparent",border:`1px solid ${showAnalysis?"#17D4FF":"#0A2840"}`,borderRadius:4,cursor:"pointer",padding:"4px 10px",display:"flex",alignItems:"center",gap:6}}>
              <span style={{fontSize:10}}>◈</span>
              <span style={{...P,fontSize:9,color:showAnalysis?"#17D4FF":"#3A6A8A",letterSpacing:2}}>ANALYSIS</span>
            </button>
            {showAnalysis&&(
              <div style={{position:"absolute",top:"calc(100% + 8px)",right:0,width:220,zIndex:200,background:"#060C16",border:"1px solid #17D4FF44",borderRadius:6,boxShadow:"0 8px 32px rgba(0,0,0,0.7)"}}>
                <div style={{padding:"8px 12px",background:"linear-gradient(135deg,#030D1A,#060C16)",borderBottom:"1px solid #0A1828",display:"flex",alignItems:"center",gap:8}}>
                  <span style={{fontSize:12}}>◈</span>
                  <span style={{...P,color:"#17D4FF",letterSpacing:3,fontSize:10}}>ANALYSIS TOOLS</span>
                </div>
                <div style={{padding:"10px 12px",borderBottom:"1px solid #0A1828"}}>
                  <div style={{display:"flex",alignItems:"center",marginBottom:8}}>
                    <span style={{...P,color:"#AACCDD",fontSize:9,letterSpacing:1}}>HEATMAP OVERLAY</span>
                    <div onClick={()=>setHeatmapOn(p=>!p)} style={{marginLeft:"auto",width:34,height:18,borderRadius:9,background:heatmapOn?"#17D4FF":"#0A1828",border:`1px solid ${heatmapOn?"#17D4FF":"#1A3A5A"}`,cursor:"pointer",position:"relative",transition:"background 0.2s",boxShadow:heatmapOn?"0 0 8px #17D4FF66":"none"}}>
                      <div style={{position:"absolute",top:2,left:heatmapOn?16:2,width:12,height:12,borderRadius:"50%",background:heatmapOn?"#fff":"#3A6A8A",transition:"left 0.2s"}}/>
                    </div>
                  </div>
                  {heatmapOn&&(
                    <div style={{display:"flex",flexDirection:"column",gap:4}}>
                      {[{key:"risk",icon:"⚠",label:"RISK MAP",desc:"Hazard proximity + stale zones"},{key:"coverage",icon:"◉",label:"COVERAGE",desc:"Scanned vs unvisited cells"},{key:"traffic",icon:"⬡",label:"DRONE TRAFFIC",desc:"Active drone density"}].map(opt=>(
                        <div key={opt.key} onClick={()=>setHeatmapMode(opt.key)} style={{padding:"6px 8px",borderRadius:4,cursor:"pointer",background:heatmapMode===opt.key?"#001A2A":"transparent",border:`1px solid ${heatmapMode===opt.key?"#17D4FF44":"#0A1828"}`,transition:"all 0.15s"}}>
                          <div style={{display:"flex",alignItems:"center",gap:6,marginBottom:1}}>
                            <span style={{fontSize:10}}>{opt.icon}</span>
                            <span style={{...P,fontSize:8,letterSpacing:1,color:heatmapMode===opt.key?"#17D4FF":"#3A6A8A"}}>{opt.label}</span>
                            {heatmapMode===opt.key&&<span style={{marginLeft:"auto",...P,fontSize:7,color:"#17D4FF"}}>● ACTIVE</span>}
                          </div>
                          <div style={{...P,fontSize:7,color:"#1A3A5A",paddingLeft:16}}>{opt.desc}</div>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
                <div style={{padding:"8px 12px",display:"flex",gap:8}}>
                  {[{label:"COVERAGE",value:`${Math.round((Object.keys(scanLogRef.current).length/(ROWS*COLS))*100)}%`},{label:"RISK ZONES",value:HAZARD_POS.length},{label:"SURVIVORS",value:SURVIVOR_POS.length}].map(({label,value})=>(
                    <div key={label} style={{flex:1,textAlign:"center"}}>
                      <div style={{...P,fontSize:11,color:"#17D4FF",fontWeight:"bold"}}>{value}</div>
                      <div style={{...P,fontSize:6.5,color:"#1A3A5A",letterSpacing:1}}>{label}</div>
                    </div>
                  ))}
                </div>
                <div style={{padding:"5px 12px",borderTop:"1px solid #0A1828",textAlign:"center"}}>
                  <span style={{...P,fontSize:7,color:"#1A3A5A"}}>click ◈ ANALYSIS to close</span>
                </div>
              </div>
            )}
          </div>

          {/* ── Update 4 · BATTERY button ── */}
          <div style={{position:"relative",pointerEvents:"all"}}>
            <button onClick={()=>setShowBatteryPanel(p=>!p)} style={{background:showBatteryPanel?"#001A0D":"transparent",border:`1px solid ${showBatteryPanel?"#00CC44":"#0A2840"}`,borderRadius:4,cursor:"pointer",padding:"4px 10px",display:"flex",alignItems:"center",gap:6}}>
              <span style={{fontSize:10}}>⚡</span>
              <span style={{...P,fontSize:9,color:showBatteryPanel?"#00CC44":"#3A6A8A",letterSpacing:2}}>BATTERY</span>
            </button>
            {showBatteryPanel&&(
              <div style={{position:"absolute",top:"calc(100% + 8px)",right:0,width:260,zIndex:200,background:"#060C16",border:"1px solid #00CC4433",borderRadius:6,boxShadow:"0 8px 32px rgba(0,0,0,0.7)"}}>
                <div style={{padding:"8px 12px",background:"linear-gradient(135deg,#030D0A,#060C16)",borderBottom:"1px solid #0A1828",display:"flex",alignItems:"center",gap:8}}>
                  <span style={{fontSize:12}}>⚡</span>
                  <span style={{...P,color:"#00CC44",letterSpacing:3,fontSize:10}}>BATTERY CONTROL</span>
                  <span style={{marginLeft:"auto",...P,fontSize:7,color:"#1A3A5A"}}>DRAG TO ADJUST</span>
                </div>
                <div style={{padding:"8px 12px",display:"flex",flexDirection:"column",gap:8}}>
                  {DRONE_CFG.map(cfg=>{
                    const bat=batteries[cfg.id]??cfg.bat;
                    const isRTB=returningDrones.includes(cfg.id);
                    const isSwapping=swapAnimations.some(s=>s.droneId===cfg.id);
                    const barCol=bat>60?"#00CC44":bat>30?"#FBBF24":"#FF3333";
                    return(
                      <div key={cfg.id}>
                        <div style={{display:"flex",alignItems:"center",gap:6,marginBottom:4}}>
                          <div style={{width:7,height:7,borderRadius:"50%",background:cfg.hex,boxShadow:`0 0 5px ${cfg.hex}`}}/>
                          <span style={{...P,fontSize:8,color:cfg.hex}}>D{cfg.id} · ZONE {cfg.zone}</span>
                          {isSwapping
                            ?<span style={{marginLeft:"auto",...P,fontSize:7,color:"#17D4FF",background:"#001A2A",border:"1px solid #17D4FF33",borderRadius:3,padding:"1px 4px"}}>DOCKED</span>
                            :isRTB
                            ?<span style={{marginLeft:"auto",...P,fontSize:7,color:"#FF6B6B",background:"#1A0A0A",border:"1px solid #FF333355",borderRadius:3,padding:"1px 4px"}}>RTB</span>
                            :<span style={{marginLeft:"auto",...P,fontSize:7.5,color:barCol,fontWeight:"bold"}}>{bat}%</span>
                          }
                        </div>
                        <input type="range" min={0} max={100} value={bat}
                          onChange={e=>{
                            const val=Number(e.target.value);
                            setBatteries(prev=>({...prev,[cfg.id]:val}));
                            if(val>25&&dronesAnim.current[cfg.id-1]?.rtb){
                              dronesAnim.current[cfg.id-1].rtb=false;
                              dronesAnim.current[cfg.id-1].path=PATROL_PATHS[cfg.id-1];
                              dronesAnim.current[cfg.id-1].t=0;
                              setReturningDrones(prev=>prev.filter(id=>id!==cfg.id));
                              addLog(`D${cfg.id} RTB cancelled · Battery restored`,"sys");
                            }
                          }}
                          style={{width:"100%",height:4,appearance:"none",WebkitAppearance:"none",background:`linear-gradient(to right,${barCol} ${bat}%,#0A1828 ${bat}%)`,borderRadius:2,outline:"none",cursor:"pointer",opacity:isSwapping?0.4:1}}
                          disabled={isSwapping}
                        />
                        {isRTB&&!isSwapping&&(
                          <div style={{marginTop:3,display:"flex",alignItems:"center",gap:5}}>
                            <div style={{flex:1,height:2,background:"#0A1828",borderRadius:1,overflow:"hidden"}}>
                              <div style={{height:"100%",borderRadius:1,background:"#17D4FF",animation:"rtbPulse 1.2s ease-in-out infinite"}}/>
                            </div>
                            <span style={{...P,fontSize:6.5,color:"#17D4FF"}}>RETURNING TO BASE</span>
                          </div>
                        )}
                        {isSwapping&&(
                          <div style={{marginTop:3,display:"flex",alignItems:"center",gap:5}}>
                            <div style={{flex:1,height:2,background:"#0A1828",borderRadius:1,overflow:"hidden"}}>
                              <div style={{height:"100%",width:`${bat}%`,borderRadius:1,background:"#00CC44",transition:"width 0.5s linear"}}/>
                            </div>
                            <span style={{...P,fontSize:6.5,color:"#00CC44"}}>RECHARGING</span>
                          </div>
                        )}
                      </div>
                    );
                  })}
                </div>
                <div style={{padding:"8px 12px",borderTop:"1px solid #0A1828"}}>
                  <div style={{...P,fontSize:8,color:"#3A6A8A",letterSpacing:2,marginBottom:6}}>STANDBY UNITS</div>
                  <div style={{display:"flex",gap:8}}>
                    {standbyDrones.map(sb=>{
                      const isLaunching=swapAnimations.length>0&&sb.id===7;
                      return(
                        <div key={sb.id} style={{flex:1,padding:"5px 7px",borderRadius:4,background:isLaunching?"#001A2A":"#030608",border:`1px solid ${isLaunching?"#17D4FF55":"#0A1828"}`,textAlign:"center"}}>
                          <div style={{fontSize:14,marginBottom:2}}>{isLaunching?"🚁":"⬡"}</div>
                          <div style={{...P,fontSize:7.5,color:isLaunching?"#17D4FF":"#3A6A8A"}}>{sb.zone}</div>
                          <div style={{...P,fontSize:6.5,color:isLaunching?"#17D4FF":"#1A3A5A",marginTop:1}}>{isLaunching?"DEPLOYING":"READY"}</div>
                        </div>
                      );
                    })}
                  </div>
                </div>
                <div style={{padding:"5px 12px",borderTop:"1px solid #0A1828",textAlign:"center"}}>
                  <span style={{...P,fontSize:7,color:"#1A3A5A"}}>click ⚡ BATTERY to close</span>
                </div>
              </div>
            )}
          </div>

          {/* ── Update 5 · AI ROUTING button ── */}
          <div style={{position:"relative",pointerEvents:"all"}}>
            <button onClick={()=>setShowRoutingPanel(p=>!p)} style={{background:showRoutingPanel?"#0A001A":"transparent",border:`1px solid ${showRoutingPanel?"#A78BFA":"#0A2840"}`,borderRadius:4,cursor:"pointer",padding:"4px 10px",display:"flex",alignItems:"center",gap:6}}>
              <span style={{fontSize:10}}>⬡</span>
              <span style={{...P,fontSize:9,color:showRoutingPanel?"#A78BFA":"#3A6A8A",letterSpacing:2}}>AI ROUTING</span>
            </button>
            {showRoutingPanel&&(
              <div style={{position:"absolute",top:"calc(100% + 8px)",right:0,width:280,zIndex:200,background:"#060C16",border:"1px solid #A78BFA33",borderRadius:6,boxShadow:"0 8px 32px rgba(0,0,0,0.7),0 0 20px #A78BFA11",overflow:"hidden"}}>
                {/* Header */}
                <div style={{padding:"8px 12px",background:"linear-gradient(135deg,#0D030D,#060C16)",borderBottom:"1px solid #0A1828",display:"flex",alignItems:"center",gap:8}}>
                  <span style={{fontSize:12}}>⬡</span>
                  <span style={{...P,color:"#A78BFA",letterSpacing:3,fontSize:10}}>AI ROUTING ENGINE</span>
                  <span style={{marginLeft:"auto",...P,fontSize:7,color:"#3A2A5A",background:"#0A0014",border:"1px solid #A78BFA22",borderRadius:3,padding:"1px 5px"}}>LIVE</span>
                </div>

                {/* Zone score grid */}
                <div style={{padding:"10px 12px",borderBottom:"1px solid #0A1828"}}>
                  <div style={{...P,fontSize:8,color:"#3A6A8A",letterSpacing:2,marginBottom:8}}>ZONE PRIORITY SCORES</div>
                  <div style={{display:"grid",gridTemplateColumns:"1fr 1fr 1fr",gap:6}}>
                    {DRONE_CFG.map(cfg=>{
                      const score=zoneScores.current[cfg.zone]??0;
                      const col=score>0.65?"#FF6B6B":score>0.35?"#FBBF24":"#00CC44";
                      return(
                        <div key={cfg.zone} style={{padding:"6px 5px",borderRadius:4,textAlign:"center",background:"#030608",border:`1px solid ${col}33`,boxShadow:score>0.65?`0 0 8px ${col}22`:"none"}}>
                          <div style={{...P,fontSize:7,color:cfg.hex,marginBottom:2}}>ZONE {cfg.zone}</div>
                          <div style={{...P,fontSize:13,color:col,fontWeight:"bold",lineHeight:1}}>{(score*100).toFixed(0)}</div>
                          <div style={{...P,fontSize:6,color:"#1A3A5A",marginTop:1}}>PRIORITY</div>
                          <div style={{marginTop:4,height:2,background:"#0A1828",borderRadius:1}}>
                            <div style={{height:"100%",width:`${score*100}%`,background:col,borderRadius:1,transition:"width 0.8s ease,background 0.5s"}}/>
                          </div>
                        </div>
                      );
                    })}
                  </div>
                </div>

                {/* Scoring weights */}
                <div style={{padding:"8px 12px",borderBottom:"1px solid #0A1828"}}>
                  <div style={{...P,fontSize:8,color:"#3A6A8A",letterSpacing:2,marginBottom:6}}>SCORING WEIGHTS</div>
                  <div style={{display:"flex",flexDirection:"column",gap:4}}>
                    {[{label:"INCIDENT URGENCY",weight:"40%",col:"#FF6B6B"},{label:"COVERAGE GAP",weight:"30%",col:"#17D4FF"},{label:"HAZARD PROXIMITY",weight:"15%",col:"#FBBF24"},{label:"ZONE STALENESS",weight:"15%",col:"#A78BFA"}].map(f=>(
                      <div key={f.label} style={{display:"flex",alignItems:"center",gap:6}}>
                        <div style={{width:6,height:6,borderRadius:1,background:f.col,flexShrink:0}}/>
                        <span style={{...P,fontSize:7.5,color:"#3A6A8A",flex:1}}>{f.label}</span>
                        <span style={{...P,fontSize:7.5,color:f.col,fontWeight:"bold"}}>{f.weight}</span>
                      </div>
                    ))}
                  </div>
                </div>

                {/* Reroute decisions log */}
                <div style={{padding:"8px 12px",maxHeight:160,overflowY:"auto"}}>
                  <div style={{...P,fontSize:8,color:"#3A6A8A",letterSpacing:2,marginBottom:6}}>REROUTE DECISIONS · {routingLog.length}</div>
                  {routingLog.length===0&&(
                    <div style={{...P,fontSize:8,color:"#1A3A5A",textAlign:"center",padding:"10px 0"}}>Evaluating zones…</div>
                  )}
                  {routingLog.map(entry=>(
                    <div key={entry.id} style={{padding:"5px 7px",marginBottom:5,borderRadius:4,background:"#030608",border:"1px solid #A78BFA22",animation:"slideIn 0.3s ease"}}>
                      <div style={{display:"flex",alignItems:"center",gap:5,marginBottom:3}}>
                        <span style={{...P,fontSize:8,color:DRONE_CFG[entry.droneId-1]?.hex}}>D{entry.droneId}</span>
                        <span style={{...P,fontSize:9,color:"#3A6A8A"}}>{entry.from} → {entry.to}</span>
                        <span style={{marginLeft:"auto",...P,fontSize:7.5,color:"#A78BFA",fontWeight:"bold"}}>{(entry.score*100).toFixed(0)} PTS</span>
                      </div>
                      <div style={{display:"flex",alignItems:"center",gap:5}}>
                        <span style={{...P,fontSize:7,letterSpacing:1,
                          color:entry.reason==="INCIDENT PRIORITY"?"#FF6B6B":entry.reason==="COVERAGE GAP"?"#17D4FF":"#FBBF24",
                          background:entry.reason==="INCIDENT PRIORITY"?"#1A0505":entry.reason==="COVERAGE GAP"?"#001020":"#1A1000",
                          border:`1px solid ${entry.reason==="INCIDENT PRIORITY"?"#FF3333":entry.reason==="COVERAGE GAP"?"#17D4FF":"#FBBF24"}33`,
                          borderRadius:3,padding:"1px 5px"}}>
                          {entry.reason}
                        </span>
                        <span style={{marginLeft:"auto",...P,fontSize:6.5,color:"#1A3A5A"}}>{entry.time}</span>
                      </div>
                      <div style={{display:"flex",gap:3,marginTop:4}}>
                        {[{key:"incident",col:"#FF6B6B"},{key:"coverage",col:"#17D4FF"},{key:"hazard",col:"#FBBF24"},{key:"staleness",col:"#A78BFA"}].map(f=>(
                          <div key={f.key} style={{flex:1}}>
                            <div style={{height:2,background:"#0A1828",borderRadius:1}}>
                              <div style={{height:"100%",borderRadius:1,background:f.col,width:`${(entry.breakdown[f.key]||0)*100}%`}}/>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>
                  ))}
                </div>

                <div style={{padding:"5px 12px",borderTop:"1px solid #0A1828",textAlign:"center"}}>
                  <span style={{...P,fontSize:7,color:"#1A3A5A"}}>click ⬡ AI ROUTING to close</span>
                </div>
              </div>
            )}
          </div>
        </div>

        {/* Bottom prompt */}
        <div style={{position:"absolute",bottom:12,left:"50%",transform:"translateX(-50%)",background:"rgba(2,5,10,0.92)",border:"1px solid #0096A0",borderRadius:20,padding:"5px 20px",pointerEvents:"none"}}>
          <span style={{color:"#17D4FF",...P,fontSize:10.5,letterSpacing:1}}>
            {waypoints.length===0?"◈  CLICK FLOOR TILES TO SET WAYPOINTS  —  PATH COMPUTES AUTOMATICALLY"
              :waypoints.length===1?"◉  CLICK TO ADD END POINT  —  KEEP CLICKING TO ADD MORE WAYPOINTS"
              :`◉  ${waypoints.length} WAYPOINTS SET  —  CLICK TO ADD MORE OR [ CLEAR ROUTE ] TO RESET`}
          </span>
        </div>

        {/* Camera controls legend */}
        <div style={{position:"absolute",bottom:48,right:10,background:"rgba(2,5,10,0.85)",border:"1px solid #0A1828",borderRadius:4,padding:"5px 9px",pointerEvents:"none"}}>
          {[["L-DRAG","Rotate"],["R-DRAG","Pan"],["SCROLL","Zoom"]].map(([k,v])=>(
            <div key={k} style={{display:"flex",gap:8,marginBottom:2}}>
              <span style={{...P,color:"#0A2840",fontSize:8.5,width:36}}>{k}</span>
              <span style={{...P,fontSize:8.5}}>{v}</span>
            </div>
          ))}
        </div>
      </div>

      {/* ══ RIGHT PANEL ══ */}
      <div style={{width:232,background:"#060C16",borderLeft:"1px solid #0A1828",display:"flex",flexDirection:"column",flexShrink:0}}>

        {/* Alert Feed */}
        <div style={{padding:"9px 10px",borderBottom:"1px solid #0A1828",minHeight:42}}>
          <div style={{...P,color:"#0096A0",letterSpacing:3,marginBottom:5}}>ALERT FEED</div>
          <div style={{display:"flex",flexDirection:"column",gap:4}}>
            {alerts.length===0&&<div style={{...P,color:"#1A3A5A",fontSize:9}}>Monitoring…</div>}
            {alerts.map(a=>{
              const ac=alertColors[a.type]||alertColors.info;
              return(
                <div key={a.id} style={{padding:"4px 7px",background:ac.bg,border:`1px solid ${ac.border}`,borderRadius:3,animation:"slideIn 0.3s ease"}}>
                  <div style={{display:"flex",gap:5,alignItems:"center",marginBottom:1}}>
                    <span style={{...P,color:ac.label,fontSize:9,fontWeight:"bold"}}>{a.type.toUpperCase()}</span>
                    <span style={{...P,marginLeft:"auto",fontSize:8}}>{a.time}</span>
                  </div>
                  <div style={{...P,color:"#AACCDD",fontSize:9,lineHeight:1.3}}>{a.msg}</div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Camera Feeds (Update · tactical UI) */}
        <div style={{padding:"9px 10px",borderBottom:"1px solid #0A1828"}}>
          <div style={{display:"flex",alignItems:"center",marginBottom:7}}>
            <div style={{...P,color:"#0096A0",letterSpacing:3}}>DRONE FEEDS</div>
            <div style={{marginLeft:"auto",...P,fontSize:8,color:"#1A5A40",background:"#001A0D",border:"1px solid #00CC4433",borderRadius:3,padding:"1px 5px"}}>LIVE · 6/6</div>
          </div>
          <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:6}}>
            {DRONE_CFG.map((cfg,i)=>(
              <div key={cfg.id} style={{position:"relative",border:`1px solid ${cfg.hex}44`,borderRadius:4,background:"#020508",boxShadow:`0 0 6px ${cfg.hex}22`,overflow:"hidden"}}>
                {/* Top bar */}
                <div style={{position:"absolute",top:0,left:0,right:0,zIndex:2,display:"flex",alignItems:"center",gap:3,background:`linear-gradient(180deg,${cfg.hex}22 0%,transparent 100%)`,padding:"2px 4px"}}>
                  <div style={{width:4,height:4,borderRadius:"50%",background:cfg.hex,boxShadow:`0 0 4px ${cfg.hex}`}}/>
                  <span style={{...P,fontSize:7,color:cfg.hex}}>D{cfg.id}·{cfg.zone}</span>
                  <span style={{...P,fontSize:6.5,color:"#3A6A8A",marginLeft:"auto"}}>{cfg.status}</span>
                </div>
                <canvas ref={el=>{feedRefs.current[i]=el;}} width={100} height={75} style={{display:"block",width:"100%",imageRendering:"pixelated"}}/>
                {/* Bottom bar: battery */}
                <div style={{position:"absolute",bottom:0,left:0,right:0,zIndex:2,background:`linear-gradient(0deg,${cfg.hex}18 0%,transparent 100%)`,padding:"2px 4px",display:"flex",alignItems:"center",gap:3}}>
                  <div style={{flex:1,height:3,background:"#0A1828",borderRadius:2,overflow:"hidden"}}>
                    <div style={{width:`${batteries[cfg.id]??cfg.bat}%`,height:"100%",borderRadius:2,background:(batteries[cfg.id]??cfg.bat)>60?"#00CC44":(batteries[cfg.id]??cfg.bat)>30?"#FBBF24":"#FF3333"}}/>
                  </div>
                  <span style={{...P,fontSize:6,color:"#3A6A8A"}}>{batteries[cfg.id]??cfg.bat}%</span>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Update 1+2 · Active Incidents */}
        {incidents.length>0&&(
          <div style={{padding:"9px 10px",borderBottom:"1px solid #0A1828"}}>
            <div style={{...P,color:"#FF6B6B",letterSpacing:3,marginBottom:6}}>ACTIVE INCIDENTS · {incidents.length}</div>
            <div style={{display:"flex",flexDirection:"column",gap:5}}>
              {incidents.map(inc=>{
                const elapsedSec=Math.floor((Date.now()-inc.detectedAt)/1000);
                const mm=String(Math.floor(elapsedSec/60)).padStart(2,"0");
                const ss=String(elapsedSec%60).padStart(2,"0");
                const urgency=elapsedSec<30?"#FBBF24":elapsedSec<90?"#FF6B6B":"#FF0000";
                const isDispatched=inc.dispatched;
                const nearestIdx=DRONE_CFG.findIndex(d=>d.id===inc.droneId);
                return(
                  <div key={inc.id} style={{padding:"5px 7px",borderRadius:4,background:"#0A0A0F",border:`1px solid ${isDispatched?"#17D4FF55":urgency+"55"}`,boxShadow:`0 0 6px ${isDispatched?"#17D4FF22":urgency+"22"}`}}>
                    <div style={{display:"flex",alignItems:"center",gap:5,marginBottom:3}}>
                      <span style={{...P,fontSize:8,color:isDispatched?"#17D4FF":urgency,fontWeight:"bold"}}>{isDispatched?"✓":"●"} {inc.label}</span>
                      <span style={{marginLeft:"auto",...P,fontSize:9,color:isDispatched?"#17D4FF":urgency,fontWeight:"bold",fontVariantNumeric:"tabular-nums"}}>{mm}:{ss}</span>
                    </div>
                    <div style={{display:"flex",gap:6,marginBottom:4}}>
                      <span style={{...P,fontSize:7.5,color:"#3A6A8A"}}>R{inc.r} C{inc.c}</span>
                      {isDispatched
                        ?<span style={{...P,fontSize:7.5,color:"#17D4FF",marginLeft:"auto"}}>D{inc.dispatchedDrone} EN ROUTE</span>
                        :<span style={{...P,fontSize:7.5,color:"#00CC44",marginLeft:"auto"}}>ETA D{inc.droneId} · {inc.eta}s</span>
                      }
                    </div>
                    {isDispatched?(
                      <div style={{padding:"3px 6px",borderRadius:3,textAlign:"center",background:"#001A2A",border:"1px solid #17D4FF33"}}>
                        <span style={{...P,fontSize:7.5,color:"#17D4FF",letterSpacing:2}}>⬡ UNIT DISPATCHED</span>
                      </div>
                    ):(
                      <button onClick={()=>{if(!dispatchedSet.current.has(inc.id)&&nearestIdx>=0)dispatchDrone(nearestIdx,inc.r,inc.c,inc.id);}}
                        style={{width:"100%",padding:"3px 0",borderRadius:3,cursor:"pointer",background:"#0A1F0A",border:"1px solid #00CC4466",color:"#00CC44",fontFamily:"'Courier New',monospace",fontSize:8,letterSpacing:2}}>
                        ▶ DISPATCH D{inc.droneId}
                      </button>
                    )}
                    <div style={{marginTop:4,height:2,background:"#0A1828",borderRadius:1}}>
                      <div style={{height:"100%",borderRadius:1,width:`${Math.min(100,(elapsedSec/120)*100)}%`,background:isDispatched?"#17D4FF":urgency,transition:"width 1s linear,background 1s"}}/>
                    </div>
                  </div>
                );
              })}
            </div>
          </div>
        )}

        {/* Mission Log */}
        <div style={{flex:1,padding:"9px 10px",display:"flex",flexDirection:"column",overflow:"hidden"}}>
          <div style={{...P,color:"#0096A0",letterSpacing:3,marginBottom:6}}>MISSION LOG</div>
          <div style={{flex:1,overflowY:"auto",display:"flex",flexDirection:"column",gap:3}}>
            {log.map(entry=>(
              <div key={entry.id} style={{display:"flex",gap:5,alignItems:"flex-start",padding:"2px 0",borderBottom:"1px solid #0A1828"}}>
                <span style={{...P,color:"#1A3A5A",fontSize:8,flexShrink:0,width:28}}>{entry.time}</span>
                <span style={{...P,color:logColors[entry.type]||"#3A6A8A",fontSize:9,lineHeight:1.3}}>{entry.msg}</span>
              </div>
            ))}
          </div>
        </div>
      </div>

      {/* ══ DRONE CAMERA OVERLAY ══ */}
      {selectedDrone!==null&&(()=>{
        const cfg=DRONE_CFG[selectedDrone];
        const resumePatrol=()=>{ dronesAnim.current[selectedDrone].paused=false; droneLockRef.current=null; setDroneCtrl({x:0,z:0}); };
        const closeOverlay=()=>{ setSelectedDrone(null); resumePatrol(); };
        return(
          <div style={{position:"fixed",top:0,left:0,right:0,bottom:0,background:"rgba(2,5,10,0.82)",zIndex:100,display:"flex",alignItems:"center",justifyContent:"center"}} onClick={e=>{if(e.target===e.currentTarget)closeOverlay();}}>
            <div style={{width:560,background:"#060C16",border:`2px solid ${cfg.hex}`,borderRadius:6,overflow:"hidden"}}>
              <div style={{padding:"8px 14px",background:"#030709",display:"flex",alignItems:"center",gap:10}}>
                <div style={{width:8,height:8,borderRadius:"50%",background:cfg.hex,boxShadow:`0 0 8px ${cfg.hex}`}}/>
                <span style={{color:cfg.hex,fontFamily:"'Courier New',monospace",fontSize:13,fontWeight:"bold",letterSpacing:4}}>DRONE {cfg.id} / ZONE {cfg.zone}</span>
                <span style={{marginLeft:"auto",fontFamily:"'Courier New',monospace",fontSize:9,color:"#3A6A8A"}}>{dronesAnim.current[selectedDrone]?.paused?"⬡ MANUAL CONTROL":"⬡ PATROL MODE"}</span>
                <button onClick={closeOverlay} style={{background:"none",border:"none",color:"#3A6A8A",cursor:"pointer",fontSize:16,lineHeight:1,padding:"0 4px"}}>✕</button>
              </div>
              <canvas width={560} height={320} style={{display:"block",width:"100%",background:"#020508",imageRendering:"pixelated"}}
                ref={el=>{ if(!el)return; const src=feedRefs.current[selectedDrone]; if(src)el.getContext("2d").drawImage(src,0,0,el.width,el.height); }}/>
              <div style={{padding:"12px 14px",borderTop:`1px solid ${cfg.hex}33`}}>
                <div style={{fontFamily:"'Courier New',monospace",fontSize:9,color:"#3A6A8A",marginBottom:8,letterSpacing:2}}>MANUAL CONTROL  —  WASD / ARROW KEYS</div>
                <div style={{display:"flex",justifyContent:"center",gap:6}}>
                  {[{label:"W ▲",dr:-1,dc:0},{label:"S ▼",dr:1,dc:0},{label:"A ◄",dr:0,dc:-1},{label:"D ►",dr:0,dc:1}].map(btn=>(
                    <button key={btn.label}
                      onMouseDown={()=>{
                        const mesh=dronesMesh.current[selectedDrone]; if(!mesh)return;
                        dronesAnim.current[selectedDrone].paused=true;
                        droneLockRef.current=selectedDrone;
                        const STEP=CS*0.8;
                        const nx=Math.max(0,Math.min((COLS-1)*CS,mesh.position.x+btn.dc*STEP));
                        const nz=Math.max(0,Math.min((ROWS-1)*CS,mesh.position.z+btn.dr*STEP));
                        const nr=Math.round(nz/CS),nc=Math.round(nx/CS);
                        if(MAP_DATA[nr]?.[nc]===WALL||MAP_DATA[nr]?.[nc]===RUBBLE)return;
                        mesh.position.x=nx; mesh.position.z=nz;
                        setDroneCtrl({x:nx,z:nz});
                      }}
                      style={{width:56,height:36,background:"#03090F",border:`1px solid ${cfg.hex}55`,borderRadius:4,color:cfg.hex,fontFamily:"'Courier New',monospace",fontSize:10,cursor:"pointer"}}>{btn.label}</button>
                  ))}
                  <button onClick={resumePatrol} style={{width:72,height:36,background:"#030F08",border:"1px solid #00CC4455",borderRadius:4,color:"#00CC44",fontFamily:"'Courier New',monospace",fontSize:9,cursor:"pointer",letterSpacing:1}}>RESUME<br/>PATROL</button>
                </div>
                <div style={{fontFamily:"'Courier New',monospace",fontSize:8,color:"#1A3A5A",marginTop:8,textAlign:"center"}}>
                  BAT {batteries[cfg.id]??cfg.bat}%  ·  STATUS {cfg.status}  ·  POS ({Math.round((dronesMesh.current[selectedDrone]?.position.x||0)/CS)},{Math.round((dronesMesh.current[selectedDrone]?.position.z||0)/CS)})
                </div>
              </div>
            </div>
          </div>
        );
      })()}

      <style>{`
        @keyframes slideIn  { from{opacity:0;transform:translateX(12px)} to{opacity:1;transform:translateX(0)} }
        @keyframes rtbPulse { 0%{width:15%;opacity:1} 50%{width:85%;opacity:0.6} 100%{width:15%;opacity:1} }
        input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; width:10px; height:10px; border-radius:50%; background:#17D4FF; cursor:pointer; }
        input[type=range]::-moz-range-thumb { width:10px; height:10px; border-radius:50%; background:#17D4FF; cursor:pointer; border:none; }
        ::-webkit-scrollbar{width:4px;background:#020508}
        ::-webkit-scrollbar-thumb{background:#0A2840;border-radius:2px}
      `}</style>
    </div>
  );
}
