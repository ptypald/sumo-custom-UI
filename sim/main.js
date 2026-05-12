import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import GUI from 'https://cdn.jsdelivr.net/npm/lil-gui@0.18/+esm';

console.log('MAIN JS LOADED');
window.__SUMO_DEBUG__ = true;

const state = {
  latest: null,
  selectedVehicleId: null,
  followSelected: false,
  playing: true,
  colorMode: 'ego',
  initialized: false,
  pollHandle: null,

  sceneReady: false,
  animationHandle: null,

  vehicleMeshes: new Map(),
  tlsMeshes: new Map(),

  roadGroup: null,
  roadSurfaceGroup: null,
  roadMarkingGroup: null,
  roadEdgeGroup: null,
  junctionGroup: null,
  tlsGroup: null,
  vehicleGroup: null,

  groundGroup: null,
  sideGreeneryGroup: null,
  trailGroup: null,
  planGroup: null,

  followHeight: 18,
  followDistance: 24,
  followLerp: 0.12,

  cameraMode: 'iso',
  trajectoryHistory: new Map(),
  selectedTrajectoryEgoIds: new Set(),
  trajectoryWindowSec: 40,

  introPlayed: false,
  debugGui: null,
  lastUiRefreshMs: 0,
  uiRefreshIntervalMs: 220,
  visualParams: {
    bgColor: '#d7e6ef',
    fogNear: 700,
    fogFar: 2200,
    ambientIntensity: 0.42,
    hemiIntensity: 0.95,
    dirIntensity: 0.98,
    roadColor: '#5a6470',
    junctionColor: '#687382',
    edgeColor: '#3a4350',
    markingColor: '#f6f8fb',
    egoColor: '#dc2626',
    carColor: '#2563eb',
    trailOpacity: 0.82,
    groundColor: '#dbe7ee',
  },
};

const els = {
  canvas: document.getElementById('viewer'),
  wrap: document.getElementById('viewerWrap'),
  playPauseBtn: document.getElementById('playPauseBtn'),
  stepBtn: document.getElementById('stepBtn'),
  followBtn: document.getElementById('followBtn'),
  resetViewBtn: document.getElementById('resetViewBtn'),
  colorMode: document.getElementById('colorMode'),
  runState: document.getElementById('runState'),
  timeValue: document.getElementById('timeValue'),
  vehicleCount: document.getElementById('vehicleCount'),
  egoCount: document.getElementById('egoCount'),
  meanSpeed: document.getElementById('meanSpeed'),
  selectedCard: document.getElementById('selectedCard'),
  tlsCard: document.getElementById('tlsCard'),
  vehicleList: document.getElementById('vehicleList'),
  vehicleFilter: document.getElementById('vehicleFilter'),
  trajEgoSelect: document.getElementById('trajEgoSelect'),
  trajWindowSec: document.getElementById('trajWindowSec'),
  trajX: document.getElementById('trajX'),
  trajY: document.getElementById('trajY'),
  trajVx: document.getElementById('trajVx'),
  trajVy: document.getElementById('trajVy'),
  trajValues: document.getElementById('trajValues'),
};

let renderer;
let scene;
let camera;
let controls;
let raycaster;
let mouseNdc;
let ambientLight;
let hemiLight;
let dirLight;

const chartDpr = Math.max(1, Math.min(window.devicePixelRatio || 1, 2));
// const hasTween = typeof TWEEN !== 'undefined';
// const hasDatGui = typeof dat !== 'undefined' && typeof dat.GUI !== 'undefined';
const hasTween = false;
const hasDatGui = true;

function api(path, options = {}) {
  return fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  }).then(async (res) => {
    const data = await res.json();
    if (!res.ok) throw new Error(data.error || `Request failed: ${res.status}`);
    return data;
  });
}

function speedToColor(speed, maxSpeed = 20) {
  const t = Math.max(0, Math.min(1, speed / maxSpeed));
  const r = Math.round(255 * t);
  const g = Math.round(255 * (1 - t));
  return new THREE.Color(`rgb(${r},${g},80)`);
}

function stringToColor(str) {
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    hash = ((hash << 5) - hash) + str.charCodeAt(i);
    hash |= 0;
  }
  const r = 80 + (Math.abs(hash) % 120);
  const g = 80 + (Math.abs(hash >> 8) % 120);
  const b = 80 + (Math.abs(hash >> 16) % 120);
  return new THREE.Color(`rgb(${r},${g},${b})`);
}

function seriesColorHex(id) {
  let hash = 0;
  for (let i = 0; i < id.length; i++) {
    hash = ((hash << 5) - hash) + id.charCodeAt(i);
    hash |= 0;
  }
  const hue = Math.abs(hash) % 360;
  return `hsl(${hue}, 75%, 62%)`;
}

function getVehicleColor(veh) {
  if (veh.is_ego) return new THREE.Color(state.visualParams.egoColor);
  if (state.colorMode === 'speed') return speedToColor(veh.speed || 0, 20);
  if (state.colorMode === 'type') return stringToColor(veh.type_id || 'default');
  return new THREE.Color(state.visualParams.carColor);
}

function disposeNode(node) {
  node.traverse?.((child) => {
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
      else child.material.dispose();
    }
  });
}

function clearGroup(group) {
  if (!group) return;
  while (group.children.length) {
    const child = group.children[0];
    group.remove(child);
    disposeNode(child);
  }
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function lerpAngle(a, b, t) {
  let d = b - a;
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return a + d * t;
}

function setupDebugGui() {
  if (!hasDatGui || state.debugGui) return;

  // const gui = new dat.GUI({ width: 300 });
  const gui = new GUI({ width: 300 });
  state.debugGui = gui;

  gui.addColor(state.visualParams, 'bgColor').name('Background').onChange((v) => {
    scene.background.set(v);
    if (scene.fog) scene.fog.color.set(v);
  });

  gui.addColor(state.visualParams, 'groundColor').name('Ground').onChange(() => {
    if (state.latest) buildGroundPlane(state.latest);
  });

  gui.add(state.visualParams, 'fogNear', 50, 5000, 1).name('Fog near').onChange((v) => {
    scene.fog.near = v;
  });

  gui.add(state.visualParams, 'fogFar', 100, 6000, 1).name('Fog far').onChange((v) => {
    scene.fog.far = v;
  });

  gui.add(state.visualParams, 'ambientIntensity', 0, 3, 0.01).name('Ambient').onChange((v) => {
    ambientLight.intensity = v;
  });

  gui.add(state.visualParams, 'hemiIntensity', 0, 3, 0.01).name('Hemi').onChange((v) => {
    hemiLight.intensity = v;
  });

  gui.add(state.visualParams, 'dirIntensity', 0, 3, 0.01).name('Sun').onChange((v) => {
    dirLight.intensity = v;
  });

  gui.add(dirLight.position, 'x', -1500, 1500, 1).name('Sun X');
  gui.add(dirLight.position, 'y', -1500, 1500, 1).name('Sun Y');
  gui.add(dirLight.position, 'z', 0, 1500, 1).name('Sun Z');
}

function setupThree() {
  renderer = new THREE.WebGLRenderer({
    canvas: els.canvas,
    antialias: true,
    alpha: false,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(els.wrap.clientWidth, els.wrap.clientHeight || window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;
  if ('outputColorSpace' in renderer) renderer.outputColorSpace = THREE.SRGBColorSpace;
  else renderer.outputEncoding = THREE.sRGBEncoding;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(state.visualParams.bgColor);
  scene.fog = new THREE.Fog(state.visualParams.bgColor, state.visualParams.fogNear, state.visualParams.fogFar);

  camera = new THREE.PerspectiveCamera(
    50,
    Math.max(1, els.wrap.clientWidth) / Math.max(1, els.wrap.clientHeight || window.innerHeight),
    0.1,
    6000
  );
  camera.position.set(-60, -60, 50);
  camera.up.set(0, 0, 1);

  // controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.screenSpacePanning = true;
  controls.maxPolarAngle = Math.PI / 2.02;
  controls.minDistance = 8;
  controls.maxDistance = 2400;
  controls.target.set(0, 0, 0);

  raycaster = new THREE.Raycaster();
  mouseNdc = new THREE.Vector2();

  ambientLight = new THREE.AmbientLight(0xffffff, state.visualParams.ambientIntensity);
  scene.add(ambientLight);

  hemiLight = new THREE.HemisphereLight(0xe8f5ff, 0xb5c3a1, state.visualParams.hemiIntensity);
  scene.add(hemiLight);

  dirLight = new THREE.DirectionalLight(0xfff7da, state.visualParams.dirIntensity);
  dirLight.position.set(350, -260, 420);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.camera.near = 1;
  dirLight.shadow.camera.far = 3000;
  dirLight.shadow.camera.left = -900;
  dirLight.shadow.camera.right = 900;
  dirLight.shadow.camera.top = 900;
  dirLight.shadow.camera.bottom = -900;
  scene.add(dirLight);

  state.groundGroup = new THREE.Group();
  state.sideGreeneryGroup = new THREE.Group();
  state.roadGroup = new THREE.Group();
  state.roadSurfaceGroup = new THREE.Group();
  state.roadMarkingGroup = new THREE.Group();
  state.roadEdgeGroup = new THREE.Group();
  state.junctionGroup = new THREE.Group();
  state.tlsGroup = new THREE.Group();
  state.vehicleGroup = new THREE.Group();
  state.trailGroup = new THREE.Group();
  state.planGroup = new THREE.Group();

  state.roadGroup.add(state.roadSurfaceGroup);
  state.roadGroup.add(state.roadMarkingGroup);
  state.roadGroup.add(state.roadEdgeGroup);

  scene.add(state.groundGroup);
  scene.add(state.sideGreeneryGroup);
  scene.add(state.roadGroup);
  scene.add(state.junctionGroup);
  scene.add(state.tlsGroup);
  scene.add(state.vehicleGroup);
  scene.add(state.trailGroup);
  scene.add(state.planGroup);

  state.sceneReady = true;
  setupDebugGui();
}

function onResize() {
  if (!renderer || !camera) return;
  const width = Math.max(1, els.wrap.clientWidth);
  const height = Math.max(1, els.wrap.clientHeight || window.innerHeight);
  renderer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function getRoadBounds(snapshot) {
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  let found = false;

  for (const edge of snapshot?.network?.edges || []) {
    for (const lane of edge.lanes || []) {
      for (const p of lane.shape || []) {
        if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) continue;
        found = true;
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
      }
    }
  }

  if (!found) {
    const b = snapshot?.bounds || { xmin: -50, xmax: 50, ymin: -50, ymax: 50 };
    return {
      xmin: b.xmin,
      xmax: b.xmax,
      ymin: b.ymin,
      ymax: b.ymax,
      cx: (b.xmin + b.xmax) / 2,
      cy: (b.ymin + b.ymax) / 2,
      width: Math.max(1, b.xmax - b.xmin),
      height: Math.max(1, b.ymax - b.ymin),
      span: Math.max(b.xmax - b.xmin, b.ymax - b.ymin),
    };
  }

  return {
    xmin: minX,
    xmax: maxX,
    ymin: minY,
    ymax: maxY,
    cx: (minX + maxX) / 2,
    cy: (minY + maxY) / 2,
    width: Math.max(1, maxX - minX),
    height: Math.max(1, maxY - minY),
    span: Math.max(maxX - minX, maxY - minY),
  };
}

function setTopView(snapshot) {
  const b = getRoadBounds(snapshot);
  const span = b.span + Math.max(20, b.span * 0.08);

  controls.target.set(b.cx, b.cy, 0);
  camera.position.set(b.cx, b.cy, Math.max(45, span * 1.05));
  camera.up.set(0, 1, 0);
  camera.lookAt(b.cx, b.cy, 0);
  controls.update();
  state.cameraMode = 'top';
}

function setIsoView(snapshot) {
  const b = getRoadBounds(snapshot);
  const span = b.span + Math.max(20, b.span * 0.08);

  controls.target.set(b.cx, b.cy, 0);
  camera.position.set(
    b.cx - span * 0.75,
    b.cy - span * 0.55,
    Math.max(35, span * 0.42)
  );
  camera.up.set(0, 0, 1);
  camera.lookAt(b.cx, b.cy, 0);
  controls.update();
  state.cameraMode = 'iso';
}

function setChaseMode() {
  state.cameraMode = 'chase';
}

// function runIntroCamera(snapshot) {
//   if (!hasTween || state.introPlayed) {
//     if (!state.introPlayed) state.introPlayed = true;
//     return;
//   }

//   const b = getRoadBounds(snapshot);
//   controls.enabled = false;
//   state.introPlayed = true;

//   const camState = {
//     x: b.cx - b.span * 1.2,
//     y: b.cy - b.span * 0.95,
//     z: Math.max(80, b.span * 0.75),
//     tx: b.cx,
//     ty: b.cy,
//     tz: 0,
//   };

//   const camTarget = {
//     x: b.cx - b.span * 0.75,
//     y: b.cy - b.span * 0.55,
//     z: Math.max(35, b.span * 0.42),
//     tx: b.cx,
//     ty: b.cy,
//     tz: 0,
//   };

//   camera.position.set(camState.x, camState.y, camState.z);
//   controls.target.set(camState.tx, camState.ty, camState.tz);

//   new TWEEN.Tween(camState)
//     .to(camTarget, 2800)
//     .easing(TWEEN.Easing.Quartic.InOut)
//     .onUpdate(() => {
//       camera.position.set(camState.x, camState.y, camState.z);
//       controls.target.set(camState.tx, camState.ty, camState.tz);
//       camera.lookAt(camState.tx, camState.ty, camState.tz);
//     })
//     .onComplete(() => {
//       controls.enabled = true;
//       state.cameraMode = 'iso';
//     })
//     .start();
// }
function runIntroCamera(snapshot) {
  if (state.introPlayed) return;
  state.introPlayed = true;
  setIsoView(snapshot);
}

function computeRoadEnvelope(shape, width) {
  if (!shape || shape.length < 2) return null;

  const left = [];
  const right = [];
  const half = width / 2;

  for (let i = 0; i < shape.length; i++) {
    const prev = shape[Math.max(0, i - 1)];
    const next = shape[Math.min(shape.length - 1, i + 1)];
    const dx = next.x - prev.x;
    const dy = next.y - prev.y;
    const len = Math.hypot(dx, dy) || 1;
    const nx = -dy / len;
    const ny = dx / len;

    left.push(new THREE.Vector2(shape[i].x + nx * half, shape[i].y + ny * half));
    right.push(new THREE.Vector2(shape[i].x - nx * half, shape[i].y - ny * half));
  }

  return [...left, ...right.reverse()];
}

function addLaneSurface(shape, width) {
  const poly = computeRoadEnvelope(shape, width);
  if (!poly || poly.length < 3) return;

  const roadShape = new THREE.Shape(poly);
  const geo = new THREE.ShapeGeometry(roadShape);
  const mat = new THREE.MeshStandardMaterial({
    color: new THREE.Color(state.visualParams.roadColor),
    roughness: 0.93,
    metalness: 0.02,
    side: THREE.DoubleSide,
    polygonOffset: true,
    polygonOffsetFactor: 1,
    polygonOffsetUnits: 1,
  });

  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.z = 0.0;
  mesh.receiveShadow = true;
  state.roadSurfaceGroup.add(mesh);
}

function addLaneEdgeLines(shape, width) {
  if (!shape || shape.length < 2) return;
  const half = width / 2;
  const leftPts = [];
  const rightPts = [];

  for (let i = 0; i < shape.length; i++) {
    const prev = shape[Math.max(0, i - 1)];
    const next = shape[Math.min(shape.length - 1, i + 1)];
    const dx = next.x - prev.x;
    const dy = next.y - prev.y;
    const len = Math.hypot(dx, dy) || 1;
    const nx = -dy / len;
    const ny = dx / len;

    leftPts.push(new THREE.Vector3(shape[i].x + nx * half, shape[i].y + ny * half, 0.05));
    rightPts.push(new THREE.Vector3(shape[i].x - nx * half, shape[i].y - ny * half, 0.05));
  }

  const mat = new THREE.LineBasicMaterial({
    color: state.visualParams.edgeColor,
    transparent: true,
    opacity: 0.68,
  });

  const left = new THREE.Line(new THREE.BufferGeometry().setFromPoints(leftPts), mat);
  const right = new THREE.Line(new THREE.BufferGeometry().setFromPoints(rightPts), mat.clone());

  state.roadEdgeGroup.add(left);
  state.roadEdgeGroup.add(right);
}

function addCenterMarkings(shape) {
  if (!shape || shape.length < 2) return;

  const dashLen = 3.2;
  const gapLen = 6.2;

  for (let i = 0; i < shape.length - 1; i++) {
    const a = shape[i];
    const b = shape[i + 1];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const segLen = Math.hypot(dx, dy);
    if (segLen < 0.5) continue;

    const dirX = dx / segLen;
    const dirY = dy / segLen;
    const angle = Math.atan2(dy, dx);

    let s = 2;
    while (s < segLen - 1) {
      const dashCenter = s + dashLen / 2;
      if (dashCenter >= segLen) break;

      const mx = a.x + dirX * dashCenter;
      const my = a.y + dirY * dashCenter;

      const geo = new THREE.BoxGeometry(dashLen, 0.12, 0.03);
      const mat = new THREE.MeshStandardMaterial({
        color: new THREE.Color(state.visualParams.markingColor),
        roughness: 0.55,
        metalness: 0.0,
        emissive: new THREE.Color('#aab4c0'),
        emissiveIntensity: 0.08,
      });

      const dash = new THREE.Mesh(geo, mat);
      dash.position.set(mx, my, 0.06);
      dash.rotation.z = angle;
      state.roadMarkingGroup.add(dash);

      s += dashLen + gapLen;
    }
  }
}

function buildRoads(snapshot) {
  clearGroup(state.roadSurfaceGroup);
  clearGroup(state.roadMarkingGroup);
  clearGroup(state.roadEdgeGroup);
  clearGroup(state.junctionGroup);

  for (const edge of snapshot.network.edges || []) {
    const lanes = edge.lanes || [];
    if (!lanes.length) continue;

    for (const lane of lanes) {
      const width = Math.max(2.5, lane.width || 3.2);
      addLaneSurface(lane.shape, width);
      addLaneEdgeLines(lane.shape, width);
    }

    if (lanes.length > 1) {
      for (const lane of lanes) addCenterMarkings(lane.shape);
    }
  }

}

function buildJunctionMarkers(snapshot) {
  const existing = state.junctionGroup.children.filter((c) => c.userData.kind === 'junctionMarker');
  for (const c of existing) {
    state.junctionGroup.remove(c);
    disposeNode(c);
  }

  for (const j of snapshot.network.junctions || []) {
    const geo = new THREE.CylinderGeometry(0.55, 0.55, 0.16, 14);
    const mat = new THREE.MeshPhongMaterial({ color: 0xef4444 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(j.x, j.y, 0.18);
    mesh.userData.kind = 'junctionMarker';
    state.junctionGroup.add(mesh);
  }
}

function buildGroundPlane(snapshot) {
  clearGroup(state.groundGroup);

  const b = getRoadBounds(snapshot);
  const sizeX = Math.max(220, b.width + 140);
  const sizeY = Math.max(220, b.height + 140);

  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(sizeX, sizeY),
    new THREE.MeshStandardMaterial({
      color: state.visualParams.groundColor,
      roughness: 1.0,
      metalness: 0.0,
    })
  );
  plane.position.set(b.cx, b.cy, -0.08);
  plane.receiveShadow = true;
  state.groundGroup.add(plane);
}

function seededRandom(seed) {
  const x = Math.sin(seed * 127.1 + 311.7) * 43758.5453;
  return x - Math.floor(x);
}

function getRoadSurfaceBounds(snapshot) {
  if (state.roadSurfaceGroup && state.roadSurfaceGroup.children.length) {
    const box = new THREE.Box3().setFromObject(state.roadSurfaceGroup);
    if (Number.isFinite(box.min.x) && Number.isFinite(box.max.x) && Number.isFinite(box.min.y) && Number.isFinite(box.max.y)) {
      return {
        xmin: box.min.x,
        xmax: box.max.x,
        ymin: box.min.y,
        ymax: box.max.y,
        cx: (box.min.x + box.max.x) * 0.5,
        cy: (box.min.y + box.max.y) * 0.5,
        width: Math.max(1, box.max.x - box.min.x),
        height: Math.max(1, box.max.y - box.min.y),
      };
    }
  }
  return getRoadBounds(snapshot);
}

function addRoadsideTree(x, y, seed) {
  const trunkH = 1.6 + seededRandom(seed + 1) * 1.8;
  const crownH = 2.4 + seededRandom(seed + 2) * 3.4;
  const crownW = 1.4 + seededRandom(seed + 3) * 1.9;
  const crownColor = new THREE.Color(
    seededRandom(seed + 4) > 0.5 ? '#6b9f1f' : '#5f8f1a'
  );

  const tree = new THREE.Group();

  const trunk = new THREE.Mesh(
    new THREE.BoxGeometry(0.45, 0.45, trunkH),
    new THREE.MeshStandardMaterial({
      color: '#4a2f1f',
      roughness: 0.9,
      metalness: 0.0,
      flatShading: true,
    })
  );
  trunk.position.set(0, 0, trunkH * 0.5 + 0.05);
  trunk.castShadow = true;
  trunk.receiveShadow = true;
  tree.add(trunk);

  const crown = new THREE.Mesh(
    new THREE.BoxGeometry(crownW, crownW, crownH),
    new THREE.MeshStandardMaterial({
      color: crownColor,
      roughness: 0.85,
      metalness: 0.0,
      flatShading: true,
    })
  );
  crown.position.set(0, 0, trunkH + crownH * 0.5 + 0.05);
  crown.castShadow = true;
  crown.receiveShadow = true;
  tree.add(crown);

  const top = new THREE.Mesh(
    new THREE.BoxGeometry(crownW * 0.72, crownW * 0.72, crownH * 0.42),
    new THREE.MeshStandardMaterial({
      color: crownColor.clone().offsetHSL(0.0, 0.05, 0.03),
      roughness: 0.85,
      metalness: 0.0,
      flatShading: true,
    })
  );
  top.position.set(0, 0, trunkH + crownH * 1.02 + 0.05);
  top.castShadow = true;
  top.receiveShadow = true;
  tree.add(top);

  tree.position.set(x, y, 0);
  state.sideGreeneryGroup.add(tree);
}

function addGreeneryStrip(cx, cy, sx, sy, seedBase, alongX) {
  const strip = new THREE.Mesh(
    new THREE.PlaneGeometry(sx, sy),
    new THREE.MeshStandardMaterial({
      color: '#a8c964',
      roughness: 1.0,
      metalness: 0.0,
      flatShading: true,
      side: THREE.DoubleSide,
    })
  );
  strip.position.set(cx, cy, -0.075);
  strip.receiveShadow = true;
  state.sideGreeneryGroup.add(strip);

  const longDim = alongX ? sx : sy;
  const treeCount = Math.max(8, Math.round(longDim / 18));
  const crossBand = (alongX ? sy : sx) * 0.34;

  for (let i = 0; i < treeCount; i++) {
    const t = (i + 0.5) / treeCount;
    const jitterLong = (seededRandom(seedBase * 100 + i * 17) - 0.5) * 7.0;
    const jitterCross = (seededRandom(seedBase * 100 + i * 31) - 0.5) * crossBand;

    let x = cx;
    let y = cy;

    if (alongX) {
      x = cx - sx * 0.5 + t * sx + jitterLong;
      y = cy + jitterCross;
    } else {
      x = cx + jitterCross;
      y = cy - sy * 0.5 + t * sy + jitterLong;
    }

    addRoadsideTree(x, y, seedBase * 1000 + i * 13);
  }
}

function buildRoadsideGreenery(snapshot) {
  clearGroup(state.sideGreeneryGroup);

  const b = getRoadSurfaceBounds(snapshot);
  const dominantX = b.width >= b.height;
  const roadThickness = dominantX ? b.height : b.width;
  const shoulderOffset = THREE.MathUtils.clamp(roadThickness * 0.08, 1.2, 3.0);
  const stripWidth = THREE.MathUtils.clamp(roadThickness * 0.9, 9.0, 18.0);
  const extraLen = 80;

  if (dominantX) {
    const stripLen = b.width + extraLen;
    const yTop = b.ymax + shoulderOffset + stripWidth * 0.5;
    const yBottom = b.ymin - shoulderOffset - stripWidth * 0.5;
    addGreeneryStrip(b.cx, yTop, stripLen, stripWidth, 11, true);
    addGreeneryStrip(b.cx, yBottom, stripLen, stripWidth, 19, true);
  } else {
    const stripLen = b.height + extraLen;
    const xRight = b.xmax + shoulderOffset + stripWidth * 0.5;
    const xLeft = b.xmin - shoulderOffset - stripWidth * 0.5;
    addGreeneryStrip(xRight, b.cy, stripWidth, stripLen, 23, false);
    addGreeneryStrip(xLeft, b.cy, stripWidth, stripLen, 29, false);
  }
}

function buildNetwork(snapshot) {
  if (!snapshot?.network?.edges || !snapshot.bounds) return;

  console.log('buildNetwork called');

  buildRoads(snapshot);
  buildJunctionMarkers(snapshot);
  // buildGroundPlane(snapshot);
  buildRoadsideGreenery(snapshot);

  if (!state.introPlayed) {
    runIntroCamera(snapshot);
  } else if (state.cameraMode === 'top') {
    setTopView(snapshot);
  } else if (!state.followSelected) {
    setIsoView(snapshot);
  }

  if (!hasTween) {
    setIsoView(snapshot);
    state.introPlayed = true;
  }
}

function createCarMesh(veh) {
  const length = Math.max(3.1, veh.length || 4.5);
  const width = Math.max(1.7, veh.width || 1.8);
  const bodyH = Math.max(1.0, Math.min(1.55, width * 0.62));
  const cabinH = bodyH * 0.64;

  const car = new THREE.Group();

  const bodyGeo = new THREE.BoxGeometry(length, width, bodyH);
  const bodyMat = new THREE.MeshStandardMaterial({
    color: getVehicleColor(veh),
    roughness: 0.6,
    metalness: 0.02,
    flatShading: true,
  });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.position.z = bodyH * 0.5 + 0.06;
  body.castShadow = true;
  body.receiveShadow = true;
  car.add(body);

  const cabinGeo = new THREE.BoxGeometry(length * 0.5, width * 0.72, cabinH);
  const cabinMat = new THREE.MeshStandardMaterial({
    color: 0x1e293b,
    roughness: 0.35,
    metalness: 0.0,
    flatShading: true,
  });
  const cabin = new THREE.Mesh(cabinGeo, cabinMat);
  cabin.position.set(-length * 0.05, 0, bodyH + cabinH * 0.5 + 0.08);
  cabin.castShadow = true;
  cabin.receiveShadow = true;
  car.add(cabin);

  const wheelMat = new THREE.MeshStandardMaterial({
    color: 0x191c20,
    roughness: 0.9,
    metalness: 0.0,
    flatShading: true,
  });
  const wheelGeo = new THREE.BoxGeometry(length * 0.16, 0.22, bodyH * 0.42);
  const wheelOffsets = [
    [length * 0.3, width * 0.51],
    [length * 0.3, -width * 0.51],
    [-length * 0.3, width * 0.51],
    [-length * 0.3, -width * 0.51],
  ];

  for (const [x, y] of wheelOffsets) {
    const wheel = new THREE.Mesh(wheelGeo, wheelMat);
    wheel.position.set(x, y, bodyH * 0.22);
    wheel.castShadow = true;
    wheel.receiveShadow = true;
    car.add(wheel);
  }

  const ringGeo = new THREE.RingGeometry(Math.max(length, width) * 0.58, Math.max(length, width) * 0.72, 36);
  const ringMat = new THREE.MeshBasicMaterial({
    color: 0x60a5fa,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0,
  });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = Math.PI / 2;
  ring.position.z = 0.06;
  car.add(ring);

  const egoBeaconGeo = new THREE.SphereGeometry(0.22, 10, 10);
  const egoBeaconMat = new THREE.MeshBasicMaterial({
    color: 0xfca5a5,
    transparent: true,
    opacity: veh.is_ego ? 0.85 : 0.0,
  });
  const egoBeacon = new THREE.Mesh(egoBeaconGeo, egoBeaconMat);
  egoBeacon.position.set(0, 0, bodyH + cabinH + 0.35);
  car.add(egoBeacon);

  car.userData.vehicleId = veh.id;
  car.userData.body = body;
  car.userData.ring = ring;
  car.userData.egoBeacon = egoBeacon;
  car.position.set(veh.x, veh.y, 0);
  car.rotation.z = ((veh.angle || 0) - 90) * Math.PI / 180;

  return car;
}

function updateVehicleMesh(mesh, veh) {
  const targetAngle = ((veh.angle || 0) - 90) * Math.PI / 180;
  const alpha = 0.28;

  mesh.position.x = lerp(mesh.position.x, Number(veh.x || 0), alpha);
  mesh.position.y = lerp(mesh.position.y, Number(veh.y || 0), alpha);
  mesh.rotation.z = lerpAngle(mesh.rotation.z || 0, targetAngle, alpha);

  if (mesh.userData.body?.material) {
    mesh.userData.body.material.color.copy(getVehicleColor(veh));
  }

  if (mesh.userData.egoBeacon?.material) {
    mesh.userData.egoBeacon.material.opacity = veh.is_ego ? 0.85 : 0.0;
  }

  const isSelected = state.selectedVehicleId === veh.id;
  if (mesh.userData.ring?.material) {
    mesh.userData.ring.material.opacity = isSelected ? 0.28 : 0.0;
  }

  mesh.userData.vehicleId = veh.id;
}

function syncVehicles(snapshot) {
  const activeIds = new Set((snapshot.vehicles || []).map((v) => v.id));

  for (const [vehId, mesh] of state.vehicleMeshes.entries()) {
    if (!activeIds.has(vehId)) {
      state.vehicleGroup.remove(mesh);
      disposeNode(mesh);
      state.vehicleMeshes.delete(vehId);
    }
  }

  for (const veh of snapshot.vehicles || []) {
    let mesh = state.vehicleMeshes.get(veh.id);
    if (!mesh) {
      mesh = createCarMesh(veh);
      state.vehicleGroup.add(mesh);
      state.vehicleMeshes.set(veh.id, mesh);
    }
    updateVehicleMesh(mesh, veh);
  }
}

function makeTrafficLightMesh(tls) {
  const group = new THREE.Group();

  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.12, 0.12, 4.6, 10),
    new THREE.MeshStandardMaterial({ color: 0x374151, roughness: 0.9, metalness: 0.05 })
  );
  pole.position.set(0, 0, 2.3);
  pole.castShadow = true;
  group.add(pole);

  const arm = new THREE.Mesh(
    new THREE.BoxGeometry(1.2, 0.12, 0.12),
    new THREE.MeshStandardMaterial({ color: 0x374151, roughness: 0.9, metalness: 0.05 })
  );
  arm.position.set(0.6, 0, 4.3);
  arm.castShadow = true;
  group.add(arm);

  const box = new THREE.Mesh(
    new THREE.BoxGeometry(0.42, 0.34, 1.1),
    new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.75, metalness: 0.08 })
  );
  box.position.set(1.1, 0, 4.0);
  box.castShadow = true;
  group.add(box);

  const lensPositions = [0.32, 0.0, -0.32];
  const lensColors = [0xef4444, 0xeab308, 0x22c55e];
  const lenses = [];

  for (let i = 0; i < 3; i++) {
    const bulb = new THREE.Mesh(
      new THREE.SphereGeometry(0.09, 12, 12),
      new THREE.MeshStandardMaterial({
        color: 0x222222,
        emissive: 0x111111,
        emissiveIntensity: 0.1,
        roughness: 0.3,
        metalness: 0.0,
      })
    );
    bulb.position.set(1.18, 0.18, 4.0 + lensPositions[i]);
    bulb.userData.baseColor = lensColors[i];
    group.add(bulb);
    lenses.push(bulb);
  }

  group.position.set(tls.x || 0, tls.y || 0, 0);
  group.userData.tlsId = tls.id;
  group.userData.lenses = lenses;
  return group;
}

function updateTrafficLightMesh(mesh, tls) {
  mesh.position.set(tls.x || 0, tls.y || 0, 0);

  const stateChar = (tls.state || '').toLowerCase();
  let activeIndex = 0;
  if (stateChar.includes('g')) activeIndex = 2;
  else if (stateChar.includes('y')) activeIndex = 1;
  else activeIndex = 0;

  const lenses = mesh.userData.lenses || [];
  lenses.forEach((lens, idx) => {
    const c = lens.userData.baseColor || 0xffffff;
    if (idx === activeIndex) {
      lens.material.color.setHex(c);
      lens.material.emissive.setHex(c);
      lens.material.emissiveIntensity = 1.8;
    } else {
      lens.material.color.setHex(0x222222);
      lens.material.emissive.setHex(0x111111);
      lens.material.emissiveIntensity = 0.1;
    }
  });
}

function syncTrafficLights(snapshot) {
  const activeIds = new Set((snapshot.traffic_lights || []).map((t) => t.id));

  for (const [tlsId, mesh] of state.tlsMeshes.entries()) {
    if (!activeIds.has(tlsId)) {
      state.tlsGroup.remove(mesh);
      disposeNode(mesh);
      state.tlsMeshes.delete(tlsId);
    }
  }

  for (const tls of snapshot.traffic_lights || []) {
    let mesh = state.tlsMeshes.get(tls.id);
    if (!mesh) {
      mesh = makeTrafficLightMesh(tls);
      state.tlsGroup.add(mesh);
      state.tlsMeshes.set(tls.id, mesh);
    }
    updateTrafficLightMesh(mesh, tls);
  }
}

function syncTrails() {
  clearGroup(state.trailGroup);

  const idsToDraw = new Set([
    ...state.selectedTrajectoryEgoIds,
    ...(state.selectedVehicleId ? [state.selectedVehicleId] : []),
  ]);

  for (const [vehId, samples] of state.trajectoryHistory.entries()) {
    if (!idsToDraw.size || !idsToDraw.has(vehId)) continue;
    if (samples.length < 2) continue;

    const points = samples.map((p) => new THREE.Vector3(p.x, p.y, 0.18));
    const geo = new THREE.BufferGeometry().setFromPoints(points);
    const mat = new THREE.LineBasicMaterial({
      color: vehId === state.selectedVehicleId ? 0x60a5fa : 0xf59e0b,
      transparent: true,
      opacity: state.visualParams.trailOpacity,
    });
    const line = new THREE.Line(geo, mat);
    state.trailGroup.add(line);
  }
}

function syncCsacPlans(snapshot) {
  clearGroup(state.planGroup);
  const selectedId = state.selectedVehicleId;

  for (const veh of snapshot.vehicles || []) {
    if (!veh.csac_plan?.states?.length) continue;
    if (!veh.is_ego) continue;
    const isSelected = !!selectedId && veh.id === selectedId;

    const pts = veh.csac_plan.states
      .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
      .map((p) => new THREE.Vector3(p.x, p.y, isSelected ? 0.72 : 0.62));

    if (pts.length >= 2) {
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      const mat = new THREE.LineBasicMaterial({
        color: isSelected ? 0x38bdf8 : 0xff4d6d,
        transparent: true,
        opacity: isSelected ? 0.98 : 0.78,
        depthWrite: false,
        depthTest: false,
      });
      const line = new THREE.Line(geo, mat);
      line.renderOrder = 25;
      state.planGroup.add(line);
    }
  }
}

function updateFollowCamera(snapshot) {
  if (!state.followSelected || !state.selectedVehicleId) return;
  const veh = (snapshot.vehicles || []).find((v) => v.id === state.selectedVehicleId);
  if (!veh) return;

  const angle = ((veh.angle || 0) - 90) * Math.PI / 180;
  const desiredX = veh.x - Math.cos(angle) * state.followDistance;
  const desiredY = veh.y - Math.sin(angle) * state.followDistance;
  const desiredZ = state.followHeight;

  camera.position.x = lerp(camera.position.x, desiredX, state.followLerp);
  camera.position.y = lerp(camera.position.y, desiredY, state.followLerp);
  camera.position.z = lerp(camera.position.z, desiredZ, state.followLerp);

  controls.target.x = lerp(controls.target.x, veh.x, state.followLerp);
  controls.target.y = lerp(controls.target.y, veh.y, state.followLerp);
  controls.target.z = lerp(controls.target.z, 0.8, state.followLerp);
}

function renderFrame() {
  state.animationHandle = requestAnimationFrame(renderFrame);

  // if (hasTween) TWEEN.update();
  if (controls) controls.update();
  if (state.latest) updateFollowCamera(state.latest);

  const t = performance.now() * 0.006;
  for (const mesh of state.vehicleMeshes.values()) {
    const ring = mesh.userData?.ring;
    const beacon = mesh.userData?.egoBeacon;
    if (ring && ring.material.opacity > 0.001) {
      const s = 1 + 0.06 * Math.sin(t);
      ring.scale.set(s, s, 1);
      ring.material.opacity = 0.2 + 0.08 * (0.5 + 0.5 * Math.sin(t * 1.15));
    }
    if (beacon && beacon.material.opacity > 0.001) {
      beacon.scale.setScalar(1 + 0.08 * Math.sin(t * 1.6));
    }
  }

  renderer.render(scene, camera);
}

function formatSpeed(speed) {
  return `${Number(speed || 0).toFixed(2)} m/s`;
}

function formatNum(v, digits = 2) {
  return Number(v || 0).toFixed(digits);
}

function selectVehicle(vehicleId) {
  state.selectedVehicleId = vehicleId || null;
  updateSidebar(state.latest || { vehicles: [], traffic_lights: [], stats: {}, meta: {}, time: 0 });

  for (const [vehId, mesh] of state.vehicleMeshes.entries()) {
    const veh = (state.latest?.vehicles || []).find((v) => v.id === vehId);
    if (veh) updateVehicleMesh(mesh, veh);
  }
}

function updateSidebar(snapshot) {
  const stats = snapshot.stats || {};
  const vehicles = snapshot.vehicles || [];
  const trafficLights = snapshot.traffic_lights || [];

  els.timeValue.textContent = `${formatNum(snapshot.time, 1)} s`;
  els.vehicleCount.textContent = `${vehicles.length}`;
  els.egoCount.textContent = `${stats.ego_count ?? vehicles.filter((v) => v.is_ego).length}`;
  els.meanSpeed.textContent = formatSpeed(stats.mean_speed || 0);

  const meta = snapshot.meta || {};
  if (meta.error) {
    els.runState.textContent = `Error: ${meta.error}`;
    els.runState.className = 'badge stopped';
  } else if (meta.finished) {
    els.runState.textContent = 'Finished';
    els.runState.className = 'badge stopped';
  } else if (meta.running) {
    els.runState.textContent = 'Running';
    els.runState.className = 'badge running';
  } else {
    els.runState.textContent = 'Paused';
    els.runState.className = 'badge paused';
  }

  state.playing = !!meta.running;
  els.playPauseBtn.textContent = state.playing ? 'Pause' : 'Resume';

  const selectedVeh = vehicles.find((v) => v.id === state.selectedVehicleId);
  if (selectedVeh) {
    els.selectedCard.innerHTML = `
      <div class="kv"><span>ID</span><strong>${selectedVeh.id}</strong></div>
      <div class="kv"><span>Type</span><strong>${selectedVeh.type_id || 'unknown'}</strong></div>
      <div class="kv"><span>Ego</span><strong>${selectedVeh.is_ego ? 'yes' : 'no'}</strong></div>
      <div class="kv"><span>Pos</span><strong>${formatNum(selectedVeh.x, 1)}, ${formatNum(selectedVeh.y, 1)}</strong></div>
      <div class="kv"><span>Speed</span><strong>${formatSpeed(selectedVeh.speed)}</strong></div>
      <div class="kv"><span>vx / vy</span><strong>${formatNum(selectedVeh.vx, 2)} / ${formatNum(selectedVeh.vy, 2)}</strong></div>
      <div class="kv"><span>Road/Lane</span><strong>${selectedVeh.road_id || '-'} / ${selectedVeh.lane_id || '-'}</strong></div>
    `;
  } else {
    els.selectedCard.innerHTML = '<div class="hint">Click a vehicle to inspect it.</div>';
  }

  if (trafficLights.length) {
    const tls = trafficLights.slice(0, 8).map((t) => `
      <div class="kv">
        <span>${t.id}</span>
        <strong>${t.state || '-'} ${Number.isFinite(t.time_to_switch) ? `(${formatNum(t.time_to_switch, 1)} s)` : ''}</strong>
      </div>
    `).join('');
    els.tlsCard.innerHTML = tls;
  } else {
    els.tlsCard.innerHTML = '<div class="hint">No traffic lights in view.</div>';
  }

  const filter = (els.vehicleFilter.value || '').toLowerCase().trim();
  const vehicleRows = vehicles
    .filter((v) => !filter || v.id.toLowerCase().includes(filter) || (v.type_id || '').toLowerCase().includes(filter))
    .slice(0, 200)
    .map((veh) => {
      const selected = veh.id === state.selectedVehicleId ? ' selected' : '';
      return `
        <button class="vehicle-row${selected}" data-vehicle-id="${veh.id}">
          <span class="vehicle-id">${veh.id}</span>
          <span>${veh.is_ego ? 'ego' : (veh.type_id || 'veh')}</span>
          <span>${formatNum(veh.speed, 1)} m/s</span>
        </button>
      `;
    }).join('');

  els.vehicleList.innerHTML = vehicleRows || '<div class="hint">No matching vehicles.</div>';
}

function updateTrajectoryHistory(snapshot) {
  const now = Number(snapshot.time || 0);
  const keepAfter = now - state.trajectoryWindowSec - 5;
  const activeIds = new Set();

  for (const veh of snapshot.vehicles || []) {
    if (!veh.is_ego) continue;
    activeIds.add(veh.id);

    const arr = state.trajectoryHistory.get(veh.id) || [];
    arr.push({
      t: now,
      x: Number(veh.x || 0),
      y: Number(veh.y || 0),
      vx: Number(veh.vx ?? veh.speed ?? 0),
      vy: Number(veh.vy ?? 0),
    });

    while (arr.length && arr[0].t < keepAfter) arr.shift();
    state.trajectoryHistory.set(veh.id, arr);
  }

  for (const key of [...state.trajectoryHistory.keys()]) {
    if (!activeIds.has(key)) {
      const arr = state.trajectoryHistory.get(key) || [];
      const filtered = arr.filter((p) => p.t >= keepAfter);
      if (filtered.length) state.trajectoryHistory.set(key, filtered);
      else state.trajectoryHistory.delete(key);
    }
  }
}

function updateTrajectorySelector(snapshot) {
  const egoVehicles = (snapshot.vehicles || []).filter((v) => v.is_ego);
  const current = new Set([...state.selectedTrajectoryEgoIds]);
  const activeIds = egoVehicles.map((v) => v.id);

  if (!current.size && egoVehicles.length) current.add(egoVehicles[0].id);

  state.selectedTrajectoryEgoIds = new Set([...current].filter((id) => activeIds.includes(id)));
  if (!state.selectedTrajectoryEgoIds.size && egoVehicles.length) {
    state.selectedTrajectoryEgoIds.add(egoVehicles[0].id);
  }

  els.trajEgoSelect.innerHTML = egoVehicles.map((veh) => {
    const selected = state.selectedTrajectoryEgoIds.has(veh.id) ? 'selected' : '';
    return `<option value="${veh.id}" ${selected}>${veh.id}</option>`;
  }).join('');
}

function setupChartCanvas(canvas) {
  if (!canvas) return null;
  const width = Math.max(160, canvas.clientWidth || canvas.width || 320);
  const height = Math.max(80, canvas.clientHeight || canvas.height || 140);
  canvas.width = Math.round(width * chartDpr);
  canvas.height = Math.round(height * chartDpr);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(chartDpr, 0, 0, chartDpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  return { ctx, width, height };
}

function extractCsacPlanPoints(veh, nowTime) {
  const out = [];
  if (!veh?.csac_plan?.states?.length) return out;
  const dt = Number(veh.csac_plan.dt || 0);
  const t0 = Number.isFinite(veh.csac_plan.t0) ? Number(veh.csac_plan.t0) : nowTime;

  veh.csac_plan.states.forEach((p, i) => {
    const t = t0 + i * dt;
    const x = Number(p.x);
    const y = Number(p.y);
    const vx = Number(p.vx ?? p.speed ?? 0);
    const vy = Number(p.vy ?? 0);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(vx) || !Number.isFinite(vy)) return;
    out.push({ t, x, y, vx, vy });
  });

  return out;
}

function drawMetricChart(canvas, series, nowTime, windowSec, metricKey, unit) {
  const setup = setupChartCanvas(canvas);
  if (!setup) return;
  const { ctx, width, height } = setup;
  const m = { left: 42, right: 10, top: 8, bottom: 18 };
  const plotW = Math.max(1, width - m.left - m.right);
  const plotH = Math.max(1, height - m.top - m.bottom);
  const now = Number.isFinite(nowTime) ? nowTime : 0;

  let tMax = now;
  for (const s of series) {
    for (const p of (s.planPoints || [])) {
      if (Number.isFinite(p.t)) tMax = Math.max(tMax, p.t);
    }
  }
  tMax = Math.max(tMax, now + 1e-6);
  const tMin = now - windowSec;

  const samples = [];
  for (const s of series) {
    for (const p of (s.actualPoints || [])) {
      if (p.t >= tMin - 1e-9 && p.t <= tMax + 1e-9) samples.push(Number(p[metricKey]));
    }
    for (const p of (s.planPoints || [])) {
      if (p.t >= tMin - 1e-9 && p.t <= tMax + 1e-9) samples.push(Number(p[metricKey]));
    }
  }

  if (!samples.length) {
    ctx.fillStyle = 'rgba(148,163,184,0.9)';
    ctx.font = '12px system-ui';
    ctx.fillText('No data in current window', 12, 22);
    return;
  }

  let yMin = Math.min(...samples);
  let yMax = Math.max(...samples);
  if (Math.abs(yMax - yMin) < 1e-6) {
    yMin -= 1.0;
    yMax += 1.0;
  } else {
    const pad = (yMax - yMin) * 0.12;
    yMin -= pad;
    yMax += pad;
  }

  const toX = (t) => m.left + ((t - tMin) / Math.max(1e-6, (tMax - tMin))) * plotW;
  const toY = (v) => m.top + ((yMax - v) / Math.max(1e-6, (yMax - yMin))) * plotH;

  ctx.fillStyle = 'rgba(2, 6, 23, 0.35)';
  ctx.fillRect(m.left, m.top, plotW, plotH);

  ctx.strokeStyle = 'rgba(148,163,184,0.25)';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 3; i++) {
    const yy = m.top + (i / 3) * plotH;
    ctx.beginPath();
    ctx.moveTo(m.left, yy);
    ctx.lineTo(m.left + plotW, yy);
    ctx.stroke();
  }

  ctx.strokeStyle = 'rgba(203,213,225,0.7)';
  ctx.beginPath();
  ctx.moveTo(m.left, m.top + plotH);
  ctx.lineTo(m.left + plotW, m.top + plotH);
  ctx.stroke();

  const xNow = toX(now);
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = 'rgba(226,232,240,0.75)';
  ctx.beginPath();
  ctx.moveTo(xNow, m.top);
  ctx.lineTo(xNow, m.top + plotH);
  ctx.stroke();
  ctx.setLineDash([]);

  for (const s of series) {
    const actualPts = (s.actualPoints || []).filter((p) => p.t >= tMin - 1e-9 && p.t <= tMax + 1e-9);
    if (actualPts.length >= 2) {
      ctx.beginPath();
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 2;
      ctx.moveTo(toX(actualPts[0].t), toY(actualPts[0][metricKey]));
      for (let i = 1; i < actualPts.length; i++) {
        ctx.lineTo(toX(actualPts[i].t), toY(actualPts[i][metricKey]));
      }
      ctx.stroke();
    }

    const planPts = (s.planPoints || []).filter((p) => p.t >= now - 1e-6 && p.t <= tMax + 1e-9);
    if (planPts.length >= 2) {
      ctx.beginPath();
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.8;
      ctx.setLineDash([6, 4]);
      ctx.moveTo(toX(planPts[0].t), toY(planPts[0][metricKey]));
      for (let i = 1; i < planPts.length; i++) {
        ctx.lineTo(toX(planPts[i].t), toY(planPts[i][metricKey]));
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  ctx.fillStyle = 'rgba(203,213,225,0.9)';
  ctx.font = '11px system-ui';
  ctx.fillText(`${yMax.toFixed(1)} ${unit}`, 4, m.top + 8);
  ctx.fillText(`${yMin.toFixed(1)} ${unit}`, 4, m.top + plotH);
  ctx.fillText(`-${windowSec}s`, m.left, height - 3);
  const futureSec = Math.max(0, tMax - now);
  const rightLabel = futureSec > 0.05 ? `+${futureSec.toFixed(1)}s` : 'now';
  ctx.fillText(rightLabel, width - 38, height - 3);
}

function updateTrajectoryPanel(snapshot) {
  updateTrajectorySelector(snapshot);

  const selectedIds = [...state.selectedTrajectoryEgoIds].sort((a, b) => a.localeCompare(b));
  const now = Number(snapshot.time || 0);
  const currentMap = new Map((snapshot.vehicles || []).map((v) => [v.id, v]));

  const series = selectedIds.map((id) => {
    const veh = currentMap.get(id);
    return {
      id,
      color: seriesColorHex(id),
      actualPoints: state.trajectoryHistory.get(id) || [],
      planPoints: extractCsacPlanPoints(veh, now),
    };
  });

  drawMetricChart(els.trajX, series, now, state.trajectoryWindowSec, 'x', 'm');
  drawMetricChart(els.trajY, series, now, state.trajectoryWindowSec, 'y', 'm');
  drawMetricChart(els.trajVx, series, now, state.trajectoryWindowSec, 'vx', 'm/s');
  drawMetricChart(els.trajVy, series, now, state.trajectoryWindowSec, 'vy', 'm/s');

  if (!selectedIds.length) {
    els.trajValues.innerHTML = '<div class="hint">No ego selected.</div>';
    return;
  }

  const rows = selectedIds.map((id) => {
    const veh = currentMap.get(id);
    if (!veh) {
      return `<div class="traj-value-row"><span class="traj-id">${id}</span><span class="hint">not active</span></div>`;
    }
    const swatch = `<span style="display:inline-block;width:10px;height:10px;border-radius:999px;background:${seriesColorHex(id)};margin-right:6px;vertical-align:middle;"></span>`;
    return `<div class="traj-value-row">
      <span class="traj-id">${swatch}${id}</span>
      <span>x=${Number(veh.x).toFixed(1)} m, y=${Number(veh.y).toFixed(1)} m, vx=${Number(veh.vx ?? veh.speed ?? 0).toFixed(2)} m/s, vy=${Number(veh.vy ?? 0).toFixed(2)} m/s</span>
    </div>`;
  }).join('');

  els.trajValues.innerHTML = rows;
}

async function pollState() {
  try {
    const snapshot = await api('/api/state');
    state.latest = snapshot;

    if (!state.initialized && snapshot.bounds) {
      buildNetwork(snapshot);
      state.initialized = true;
    }

    updateTrajectoryHistory(snapshot);
    syncVehicles(snapshot);
    syncTrafficLights(snapshot);
    syncTrails();
    syncCsacPlans(snapshot);

    const nowMs = performance.now();
    const forceUi = !!snapshot?.meta?.error || !!snapshot?.meta?.finished;
    if (forceUi || (nowMs - state.lastUiRefreshMs) >= state.uiRefreshIntervalMs) {
      updateSidebar(snapshot);
      updateTrajectoryPanel(snapshot);
      state.lastUiRefreshMs = nowMs;
    }
  } catch (err) {
    els.runState.textContent = `Error: ${err.message}`;
    els.runState.className = 'badge stopped';
  } finally {
    const nextPollMs = document.hidden ? 350 : 100;
    state.pollHandle = setTimeout(pollState, nextPollMs);
  }
}

async function sendControl(action) {
  await api('/api/control', {
    method: 'POST',
    body: JSON.stringify({ action }),
  });
}

function setupEvents() {
  els.playPauseBtn.addEventListener('click', async () => {
    await sendControl(state.playing ? 'pause' : 'resume');
  });

  els.stepBtn.addEventListener('click', async () => {
    await sendControl('step');
  });

  els.followBtn.addEventListener('click', () => {
    state.followSelected = !state.followSelected;
    els.followBtn.textContent = state.followSelected ? 'Unfollow selected' : 'Follow selected';
    if (state.followSelected) setChaseMode();
    else if (state.latest) setIsoView(state.latest);
    updateSidebar(state.latest || { vehicles: [], traffic_lights: [], stats: {}, meta: {}, time: 0 });
  });

  els.resetViewBtn.addEventListener('click', () => {
    if (!state.latest) return;
    state.followSelected = false;
    els.followBtn.textContent = 'Follow selected';
    setIsoView(state.latest);
    updateSidebar(state.latest);
  });

  els.colorMode.addEventListener('change', () => {
    state.colorMode = els.colorMode.value;
    for (const [vehId, mesh] of state.vehicleMeshes.entries()) {
      const veh = (state.latest?.vehicles || []).find((v) => v.id === vehId);
      if (veh) updateVehicleMesh(mesh, veh);
    }
  });

  els.vehicleFilter.addEventListener('input', () => {
    updateSidebar(state.latest || { vehicles: [], traffic_lights: [], stats: {}, meta: {}, time: 0 });
  });

  els.trajEgoSelect.addEventListener('change', () => {
    state.selectedTrajectoryEgoIds = new Set(
      [...els.trajEgoSelect.selectedOptions].map((o) => o.value).filter(Boolean)
    );
    if (state.latest) updateTrajectoryPanel(state.latest);
  });

  els.trajWindowSec.addEventListener('change', () => {
    const val = Number(els.trajWindowSec.value || 40);
    state.trajectoryWindowSec = Number.isFinite(val) && val > 0 ? val : 40;
    if (state.latest) updateTrajectoryPanel(state.latest);
  });

  els.vehicleList.addEventListener('click', (event) => {
    const row = event.target.closest('[data-vehicle-id]');
    if (!row) return;
    selectVehicle(row.dataset.vehicleId);
  });

  renderer.domElement.addEventListener('click', (event) => {
    const rect = renderer.domElement.getBoundingClientRect();
    mouseNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouseNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouseNdc, camera);
    const meshes = [...state.vehicleMeshes.values()];
    const intersects = raycaster.intersectObjects(meshes, true);
    if (!intersects.length) return;

    let obj = intersects[0].object;
    while (obj && !obj.userData.vehicleId && obj.parent) obj = obj.parent;
    if (obj?.userData?.vehicleId) selectVehicle(obj.userData.vehicleId);
  });

  window.addEventListener('keydown', (event) => {
    if (!state.latest) return;

    if (event.key === '1') {
      state.followSelected = false;
      els.followBtn.textContent = 'Follow selected';
      setTopView(state.latest);
      updateSidebar(state.latest);
    } else if (event.key === '2') {
      state.followSelected = false;
      els.followBtn.textContent = 'Follow selected';
      setIsoView(state.latest);
      updateSidebar(state.latest);
    } else if (event.key === '3') {
      if (state.selectedVehicleId) {
        state.followSelected = true;
        els.followBtn.textContent = 'Unfollow selected';
        setChaseMode();
        updateSidebar(state.latest);
      }
    }
  });

  window.addEventListener('resize', onResize);
  window.addEventListener('resize', () => {
    if (state.latest) updateTrajectoryPanel(state.latest);
  });
}

setupThree();
setupEvents();
onResize();
renderFrame();
pollState();
