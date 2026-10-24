const $ = id => document.getElementById(id);
const canvas = $('scene');
const gl = canvas.getContext('webgl', {antialias: true});
const colors = [[1,.3,.3], [1,.7,.2], [.8,.5,1]];
let yaw = -.65, pitch = .85, zoom = 1, fixture, school, drone, wire, grid;
let payload, selected, elapsed = 0, playing = false, last = 0, duration = 0;
let candidateLines = [], marks = [], labels = [], droneLabel;
const plans = new Map();

function message(text) { $('loading').textContent = text; $('loading').hidden = false; }
function shader(kind, source) {
  const s = gl.createShader(kind); gl.shaderSource(s, source); gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
  return s;
}
let program, attr, uniforms;
function initGL() {
  if (!gl) throw Error('WebGL을 사용할 수 없습니다. 브라우저의 하드웨어 가속 설정을 확인하세요.');
  program = gl.createProgram();
  gl.attachShader(program, shader(gl.VERTEX_SHADER, `
    attribute vec3 position; attribute vec3 normal;
    uniform vec3 offset, scale; uniform vec2 angle, viewport;
    uniform float zoom; varying float light;
    void main() {
      vec3 p = position * scale + offset - vec3(30.,30.,0.);
      float x = cos(angle.x)*p.x - sin(angle.x)*p.y;
      float y = sin(angle.x)*p.x + cos(angle.x)*p.y;
      float h = sin(angle.y)*y + cos(angle.y)*p.z;
      float d = cos(angle.y)*y - sin(angle.y)*p.z;
      gl_Position = vec4(x*zoom/viewport.x, h*zoom/viewport.y, d/500., 1.);
      light = length(normal) < .1 ? 1. : .5 + .5*abs(dot(normalize(normal), normalize(vec3(-.4,-.6,1.))));
    }`));
  gl.attachShader(program, shader(gl.FRAGMENT_SHADER, `
    precision mediump float; uniform vec3 color; varying float light;
    void main() { gl_FragColor = vec4(color*light, 1.); }`));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);
  attr = Object.fromEntries(['position','normal'].map(k => [k, gl.getAttribLocation(program,k)]));
  uniforms = Object.fromEntries(['offset','scale','angle','viewport','zoom','color'].map(k => [k,gl.getUniformLocation(program,k)]));
  gl.enableVertexAttribArray(attr.position); gl.enableVertexAttribArray(attr.normal);
  gl.enable(gl.DEPTH_TEST); gl.clearColor(.039,.071,.125,1);
}
function mesh(values, mode) {
  const buffer = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(values), gl.STATIC_DRAW);
  return {buffer, count: values.length / 6, mode};
}
function lines(points) { return mesh(points.flatMap(p => [...p,0,0,0]), gl.LINES); }
function pathLine(points) { return lines(points.slice(1).flatMap((p,i) => [points[i],p])); }
async function stl(url) {
  const response = await fetch(url);
  if (!response.ok) throw Error(`모델 파일을 읽을 수 없습니다 (${response.status}): ${url}`);
  const bytes = await response.arrayBuffer();
  if (bytes.byteLength < 84) throw Error(`잘못된 STL: ${url}`);
  const view = new DataView(bytes), count = view.getUint32(80,true);
  if (84 + count*50 !== bytes.byteLength) throw Error(`지원하지 않는 STL 형식: ${url}`);
  const vertices = new Float32Array(count*18);
  for (let i=0; i<count; i++) {
    const start=84+i*50;
    const p = Array.from({length:3}, (_,v) => Array.from({length:3}, (_,a) => view.getFloat32(start+12+v*12+a*4,true)));
    const a=p[1].map((x,j)=>x-p[0][j]), b=p[2].map((x,j)=>x-p[0][j]);
    const n=[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
    const length=Math.hypot(...n)||1;
    for(let v=0;v<3;v++) vertices.set([...p[v],...n.map(x=>x/length)],i*18+v*6);
  }
  return mesh(vertices, gl.TRIANGLES);
}
function draw(object, color, offset=[0,0,0], scale=[1,1,1]) {
  gl.bindBuffer(gl.ARRAY_BUFFER,object.buffer);
  gl.vertexAttribPointer(attr.position,3,gl.FLOAT,false,24,0);
  gl.vertexAttribPointer(attr.normal,3,gl.FLOAT,false,24,12);
  gl.uniform3fv(uniforms.color,color); gl.uniform3fv(uniforms.offset,offset); gl.uniform3fv(uniforms.scale,scale);
  gl.drawArrays(object.mode,0,object.count);
}
function geo(w) {
  const rad=Math.PI/180, r=6371000;
  return [(w.lon-fixture.origin.lon)*rad*r*Math.cos(fixture.origin.lat*rad),(w.lat-fixture.origin.lat)*rad*r,w.altitude_m];
}
async function json(url, optional=false) {
  const response=await fetch(url, {cache:'no-store'});
  if(optional && response.status===404) return null;
  if(!response.ok) throw Error(`데이터를 읽지 못했습니다 (${response.status}): ${url}`);
  return response.json();
}
function addLabel(text, point, color) {
  const element=document.createElement('span'); element.className='map-label'; element.textContent=text; element.style.color=color;
  canvas.parentElement.append(element); const label={element,point}; labels.push(label); return label;
}
function screen(p) {
  const [x,y,z]=p.map((v,i)=>v-[30,30,0][i]);
  const h=Math.sin(yaw)*x+Math.cos(yaw)*y;
  const span=85/zoom;
  return [canvas.clientWidth/2+(Math.cos(yaw)*x-Math.sin(yaw)*y)*canvas.clientHeight/(span*2),canvas.clientHeight/2-(Math.sin(pitch)*h+Math.cos(pitch)*z)*canvas.clientHeight/(span*2)];
}
function waypointAt(t) {
  const points=selected.waypoints.map(geo), speed=payload.constraints.speed_mps;
  let distance=Math.min(t,duration)*speed;
  for(let i=1;i<points.length;i++) {
    const a=points[i-1], b=points[i], length=Math.hypot(...b.map((v,j)=>v-a[j]));
    if(distance<=length) return a.map((v,j)=>v+(b[j]-v)*(length ? distance/length : 0));
    distance-=length;
  }
  return points.at(-1);
}
function choose() {
  const plan=plans.get($('mission').value);
  playing=false; elapsed=0; $('play').textContent='경로 재생';
  for(const line of candidateLines) gl.deleteBuffer(line.mesh.buffer);
  candidateLines=[];
  if(!plan) {
    payload=selected=null; $('play').disabled=true; $('progress').disabled=true; $('metrics').replaceChildren();
    $('source').textContent='저장된 경로가 없습니다. make school-test 또는 make school-test-live 실행 후 새로고침하세요.';
    return;
  }
  payload=plan.payload; selected=payload.candidates[payload.selected_index];
  if(!selected || !payload.validations?.[payload.selected_index]?.valid) throw Error('유효한 선택 경로가 없는 테스트 계획입니다. 테스트를 다시 실행하세요.');
  duration=payload.validations[payload.selected_index].estimated_duration_s;
  if(!Number.isFinite(duration) || duration<=0 || !Number.isFinite(payload.constraints.speed_mps) || payload.constraints.speed_mps<=0 || selected.waypoints.length<2 || !selected.waypoints.every(w=>geo(w).every(Number.isFinite))) throw Error('경로 좌표·시간·속도가 올바르지 않습니다. 테스트를 다시 실행하세요.');
  payload.candidates.forEach((candidate,i)=>candidateLines.push({mesh:pathLine(candidate.waypoints.map(geo)), selected:i===payload.selected_index, valid:payload.validations[i]?.valid}));
  const m=payload.validations[payload.selected_index];
  $('metrics').replaceChildren();
  for(const [name,value] of [['이동 거리',`${m.distance_m.toFixed(1)} m`],['대기 포함 시간',`${duration.toFixed(1)} s`],['예상 전력량',`${m.estimated_energy_wh.toFixed(2)} Wh`],['창문 정지 거리',payload.window_target ? `${payload.window_target.standoff_m} m` : '—']]) {
    const box=document.createElement('div'); box.className='metric';
    const label=document.createElement('span'), val=document.createElement('b'); label.textContent=name; val.textContent=value; box.append(label,val); $('metrics').append(box);
  }
  $('source').textContent=`${plan.source} · 저장된 계획의 이동·대기 시간 재생 (실시간 텔레메트리 아님)`;
  $('play').disabled=false; $('progress').disabled=false;
}
function animate(now) {
  const dt=last ? Math.min((now-last)/1000,.25) : 0; last=now;
  if(playing) {
    elapsed=Math.min(duration,elapsed+dt*Number($('speed').value));
    if(elapsed>=duration) { playing=false; $('play').textContent='다시 재생'; }
  }
  const dpr=Math.min(window.devicePixelRatio||1,2), w=Math.round(canvas.clientWidth*dpr), h=Math.round(canvas.clientHeight*dpr);
  if(canvas.width!==w||canvas.height!==h) {canvas.width=w;canvas.height=h;}
  gl.viewport(0,0,w,h); gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  gl.uniform2f(uniforms.viewport,85*w/h,85); gl.uniform2f(uniforms.angle,yaw,pitch); gl.uniform1f(uniforms.zoom,zoom);
  gl.enable(gl.DEPTH_TEST);
  draw(grid,[.15,.24,.34]); draw(school,[.55,.65,.76],fixture.campus.mesh_pose_enu,fixture.campus.mesh_scale);
  // Planned routes and risk markers stay visible through the visual campus mesh.
  gl.disable(gl.DEPTH_TEST);
  for(const b of fixture.buildings) draw(wire,[.76,.38,.29],[b.center_enu[0],b.center_enu[1],b.no_fly_ceiling_m/2],[...b.size_m.slice(0,2),b.no_fly_ceiling_m]);
  for(const line of candidateLines) if(!line.selected) draw(line.mesh,line.valid ? [.32,.4,.5] : [.8,.3,.25]);
  for(const line of candidateLines) if(line.selected) draw(line.mesh,[.22,.86,.76]);
  for(const mark of marks) draw(wire,mark.color,mark.point,mark.size);
  const pos=selected ? waypointAt(elapsed) : fixture.route_start_enu;
  droneLabel.point=pos;
  draw(drone,[.94,.94,1],pos.map((v,i)=>v+[-.00257,.00702,-.25][i]),[.01,.01,.01]);
  draw(wire,[.9,.96,1],pos,[2,2,1]);
  for(const label of labels) {
    const [x,y]=screen(label.point); label.element.style.left=`${x}px`;label.element.style.top=`${y}px`;
  }
  $('position').textContent=`드론 ENU: ${pos.map(v=>v.toFixed(1)).join(', ')} m${selected && elapsed>=(duration-(payload.constraints.hold_seconds||0)) && payload.window_target ? ' · 창문 밖 수령 대기' : ''}`;
  $('progress').value=duration ? elapsed/duration*1000 : 0; $('progress-label').value=`${duration ? Math.round(elapsed/duration*100) : 0}%`;
  requestAnimationFrame(animate);
}
async function start() {
  initGL();
  fixture=await json('/fixtures/school.json');
  const corners=[[-.5,-.5,-.5],[.5,-.5,-.5],[.5,.5,-.5],[-.5,.5,-.5],[-.5,-.5,.5],[.5,-.5,.5],[.5,.5,.5],[-.5,.5,.5]];
  wire=lines([[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]].flatMap(e=>e.map(i=>corners[i])));
  grid=lines(Array.from({length:13},(_,i)=>{const x=i*10-30;return [[x,-30,0],[x,90,0],[-30,x,0],[90,x,0]];}).flat());
  [school,drone]=await Promise.all([stl('/models/environments/school-demo/meshes/school_map.stl'),stl('/models/drones/x500_base/meshes/drone.stl')]);
  for(const o of fixture.obstacles) {
    const color=colors[o.priority-1]; marks.push({point:o.position_enu,color,size:Array(3).fill(Math.max(o.radius_m*2,1))});
    addLabel(`${o.priority} · ${o.id}`,o.position_enu,`rgb(${color.map(x=>x*255).join(',')})`);
    const row=document.createElement('div');row.className='risk';row.style.color=`rgb(${color.map(x=>x*255).join(',')})`;
    row.textContent=`${o.priority}단계 · ${o.id} (${o.label})`; $('obstacles').append(row);
  }
  const win=fixture.window.position_enu, bearing=fixture.window.outward_bearing_deg*Math.PI/180;
  const hover=[win[0]+Math.sin(bearing)*fixture.window.standoff_m,win[1]+Math.cos(bearing)*fixture.window.standoff_m,win[2]];
  marks.push({point:win,color:[.5,.7,1],size:[1.5,1.5,1.5]},{point:hover,color:[.5,.7,1],size:[1,1,1]});
  addLabel('창문 밖 정지',hover,'#87b8ff');
  droneLabel=addLabel('◆ 드론',fixture.route_start_enu,'#ffffff');
  droneLabel.element.style.transform='translate(-50%,-220%)';
  for(const [key,api,offline] of [['window','school-api-window','school-window'],['outdoor','school-api-outdoor','school-detour']]) {
    const actual=await json(`/logs/school-tests/${api}.json`,true);
    const plan=actual || await json(`/logs/school-tests/${offline}.json`,true);
    if(plan) plans.set(key,{...plan,source:actual ? '실제 API 테스트 계획' : '오프라인 테스트 계획'});
  }
  const report=await json('/logs/school-tests/report.json',true);
  $('summary').textContent=report ? `회피 ${report.avoidance.passed}개 통과 · 경로 ${report.cross_checks.length}개 검증${report.backend_api ? ' · API 배송 상태 검증 통과' : ''}` : '저장된 결과 없음 · make school-test 실행 후 새로고침';
  const actions={proceed:'진행',slow:'감속',hold:'정지',request_replan:'재계획',projected:'좌표 변환',unlocalized:'위치 불명'};
  for(const c of report?.avoidance.cases || []) {
    const row=document.createElement('div'), value=document.createElement('span');row.className='case';row.textContent=c.name;
    value.textContent=actions[c.action] || c.action;row.append(value);$('cases').append(row);
  }
  choose(); $('loading').hidden=true;
  $('mission').onchange=()=>{try{choose();}catch(e){message(e.message);}};
  $('play').onclick=()=>{if(elapsed>=duration) elapsed=0; playing=!playing;$('play').textContent=playing?'일시 정지':'경로 재생';};
  $('progress').oninput=()=>{elapsed=Number($('progress').value)/1000*duration;};
  $('reset').onclick=()=>{yaw=-.65;pitch=.85;zoom=1;};
  let drag;
  canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
  canvas.onpointermove=e=>{if(!drag)return; yaw+=(e.clientX-drag[0])*.006;pitch=Math.max(.15,Math.min(1.55,pitch+(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];};
  canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;};
  canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.4,Math.min(8,zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
  canvas.addEventListener('webglcontextlost',e=>{e.preventDefault();playing=false;message('그래픽 연결이 끊겼습니다. 페이지를 새로고침하세요.');});
  requestAnimationFrame(animate);
}
start().catch(error=>message(error.message));
