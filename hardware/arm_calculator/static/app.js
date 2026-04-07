'use strict';

const DEFAULTS = {
  link1_length: 200,
  link2_length: 180,
  link3_length: 80,
  tube_od: 22,
  tube_wall: 2,
  shaft_dia: 8,
  bearing_od: 22,
  bearing_width: 7,
  hub_od: 40,
  motor_mass: 300,
  hub_mass: 80,
  gripper_mass: 400,
  payload_mass: 500,
  spool_radius: 11,
  elbow_pulley_radius: 15,
  wrist_pulley_radius: 15,
  shoulder_gear_ratio: 1.0,
  elbow_gear_ratio: 1.0,
  wrist_gear_ratio: 1.0,
  shoulder_angle: 45,
  elbow_angle: 90,
  wrist_angle: 0,
  line_break_strength: 133,
  line_safety_factor: 3.0,
  torque_safety_factor: 1.5,
  motor_torque_target: 1.5,
};

// Current state
let state = { ...DEFAULTS };
let debounceTimer = null;

// --- Init controls ---
function initControls() {
  for (const [key, val] of Object.entries(DEFAULTS)) {
    const slider = document.getElementById(key);
    const num    = document.getElementById(key + '_n');
    if (!slider || !num) continue;

    slider.value = val;
    num.value    = val;

    slider.addEventListener('input', () => {
      num.value = slider.value;
      state[key] = parseFloat(slider.value);
      scheduleUpdate();
    });
    num.addEventListener('input', () => {
      slider.value = num.value;
      state[key] = parseFloat(num.value);
      scheduleUpdate();
    });
  }
}

function resetDefaults() {
  state = { ...DEFAULTS };
  for (const [key, val] of Object.entries(DEFAULTS)) {
    const s = document.getElementById(key);
    const n = document.getElementById(key + '_n');
    if (s) s.value = val;
    if (n) n.value = val;
  }
  scheduleUpdate();
}

function scheduleUpdate() {
  clearTimeout(debounceTimer);
  debounceTimer = setTimeout(fetchResults, 40);
}

// --- Fetch ---
async function fetchResults() {
  try {
    const res = await fetch('/calculate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(state),
    });
    if (!res.ok) throw new Error(await res.text());
    const data = await res.json();
    renderResults(data);
    drawArm(data.pose);
    document.getElementById('status-msg').textContent = '';
  } catch (e) {
    document.getElementById('status-msg').textContent = 'Error: ' + e.message;
  }
}

// --- Render results ---
function set(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function dot(id, status) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = 'dot ' + status;
}

function renderResults(d) {
  set('r-ts', d.torques.shoulder);
  set('r-te', d.torques.elbow);
  set('r-tw', d.torques.wrist);

  set('r-ms', d.motor_torques.shoulder);
  set('r-me', d.motor_torques.elbow);
  set('r-mw', d.motor_torques.wrist);
  dot('dot-ms', d.motor_torques.shoulder_status);
  dot('dot-me', d.motor_torques.elbow_status);
  dot('dot-mw', d.motor_torques.wrist_status);

  set('r-cl', d.cable.limit_N);
  set('r-ce', d.cable.elbow_tension_N);
  set('r-cw', d.cable.wrist_tension_N);
  dot('dot-ce', d.cable.elbow_status);
  dot('dot-cw', d.cable.wrist_status);

  set('r-grs', d.gear_recs.shoulder);
  set('r-gre', d.gear_recs.elbow);
  set('r-grw', d.gear_recs.wrist);
  set('r-rpms', d.gear_recs.rpm_shoulder);
  set('r-rpme', d.gear_recs.rpm_elbow);
  set('r-rpmw', d.gear_recs.rpm_wrist);

  // Mass table
  const tbody = document.getElementById('mass-tbody');
  tbody.innerHTML = '';
  for (const row of d.mass_table) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${row.component}</td><td>${row.mass_g}</td>`;
    tbody.appendChild(tr);
  }
  const tr = document.createElement('tr');
  tr.className = 'total';
  tr.innerHTML = `<td>Total</td><td>${d.total_mass_g}</td>`;
  tbody.appendChild(tr);

  // Pose readout
  set('pr-s', d.pose.shoulder_deg);
  set('pr-e', d.pose.elbow_deg);
  set('pr-w', d.pose.wrist_deg);
  // Reach = horizontal distance to tip
  const reach = Math.round(d.pose.x_tip_m * 1000);
  set('pr-r', reach);
}

// --- SVG arm draw ---
function drawArm(pose) {
  const svg = document.getElementById('arm-svg');
  const W = 380, H = 400;
  // viewBox: -200 -380 400 420 → origin at (0,0) in SVG coords = bottom center
  // SVG y goes down, so we invert: arm goes up

  const L1 = pose.L1;  // mm
  const L2 = pose.L2;
  const L3 = pose.L3;
  const sa = toRad(pose.shoulder_deg);
  const ea = toRad(pose.elbow_deg);
  const wa = toRad(pose.wrist_deg);

  // scale: fit arm in ~350px height, max arm length ~460mm
  const scale = 0.75; // px per mm

  // Pivot = (0, 0) in our coordinate system, SVG y inverted
  const p0 = { x: 0, y: 0 };
  const p1 = {
    x: p0.x + L1 * scale * Math.sin(sa),
    y: p0.y - L1 * scale * Math.cos(sa),
  };
  const ang2 = sa + ea;
  const p2 = {
    x: p1.x + L2 * scale * Math.sin(ang2),
    y: p1.y - L2 * scale * Math.cos(ang2),
  };
  const ang3 = ang2 + wa;
  const p3 = {
    x: p2.x + L3 * scale * Math.sin(ang3),
    y: p2.y - L3 * scale * Math.cos(ang3),
  };

  const ns = 'http://www.w3.org/2000/svg';

  // Clear
  while (svg.firstChild) svg.removeChild(svg.firstChild);

  // Ground line
  addLine(svg, -180, 10, 180, 10, '#1e2235', 1.5);

  // Ground hatch
  for (let i = -170; i <= 170; i += 20) {
    addLine(svg, i, 10, i - 12, 22, '#1e2235', 1);
  }

  // Base circle
  addCircle(svg, 0, 0, 12, '#2d3147', '#1a1d2e');

  // Link lines
  addLine(svg, p0.x, p0.y, p1.x, p1.y, '#6366f1', 5);
  addLine(svg, p1.x, p1.y, p2.x, p2.y, '#38bdf8', 4);
  addLine(svg, p2.x, p2.y, p3.x, p3.y, '#22c55e', 3);

  // Joint circles
  addCircle(svg, p1.x, p1.y, 7, '#6366f1', '#0f1117');
  addCircle(svg, p2.x, p2.y, 6, '#38bdf8', '#0f1117');
  addCircle(svg, p3.x, p3.y, 5, '#22c55e', '#0f1117');

  // Gripper (two small lines)
  const gAngle = ang3;
  const perpX = Math.cos(gAngle) * 10;
  const perpY = -Math.sin(gAngle) * 10;  // SVG y inverted
  addLine(svg, p3.x + perpX, p3.y - perpY - 8 * Math.cos(gAngle),
               p3.x + perpX + 12 * Math.sin(gAngle), p3.y - perpY + 12 * Math.cos(gAngle), '#22c55e', 2);
  addLine(svg, p3.x - perpX, p3.y + perpY - 8 * Math.cos(gAngle),
               p3.x - perpX + 12 * Math.sin(gAngle), p3.y + perpY + 12 * Math.cos(gAngle), '#22c55e', 2);

  // Labels
  addText(svg, (p0.x + p1.x) / 2 - 8, (p0.y + p1.y) / 2, `L1`, '#a5b4fc', 9);
  addText(svg, (p1.x + p2.x) / 2 - 8, (p1.y + p2.y) / 2, `L2`, '#7dd3fc', 9);
  addText(svg, (p2.x + p3.x) / 2 - 8, (p2.y + p3.y) / 2, `L3`, '#86efac', 9);
}

function toRad(deg) { return deg * Math.PI / 180; }

function addLine(svg, x1, y1, x2, y2, stroke, width) {
  const el = document.createElementNS('http://www.w3.org/2000/svg', 'line');
  el.setAttribute('x1', x1); el.setAttribute('y1', y1);
  el.setAttribute('x2', x2); el.setAttribute('y2', y2);
  el.setAttribute('stroke', stroke);
  el.setAttribute('stroke-width', width);
  el.setAttribute('stroke-linecap', 'round');
  svg.appendChild(el);
}

function addCircle(svg, cx, cy, r, stroke, fill) {
  const el = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
  el.setAttribute('cx', cx); el.setAttribute('cy', cy); el.setAttribute('r', r);
  el.setAttribute('stroke', stroke); el.setAttribute('stroke-width', 2);
  el.setAttribute('fill', fill);
  svg.appendChild(el);
}

function addText(svg, x, y, text, fill, size) {
  const el = document.createElementNS('http://www.w3.org/2000/svg', 'text');
  el.setAttribute('x', x); el.setAttribute('y', y);
  el.setAttribute('fill', fill); el.setAttribute('font-size', size);
  el.setAttribute('font-family', 'monospace');
  el.textContent = text;
  svg.appendChild(el);
}

// --- Export ---
async function exportCSV() {
  try {
    const res = await fetch('/export', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(state),
    });
    if (!res.ok) throw new Error(await res.text());
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'arm_params_fusion360.csv';
    a.click();
    URL.revokeObjectURL(url);
  } catch (e) {
    document.getElementById('status-msg').textContent = 'Export error: ' + e.message;
  }
}

// --- Boot ---
initControls();
fetchResults();
