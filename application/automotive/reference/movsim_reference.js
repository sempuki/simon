// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Records the Intelligent Driver Model and MOBIL as their authors implement
// them in movsim's traffic-simulation.de (Treiber and Kesting,
// https://github.com/movsim/traffic-simulation-de, GPL-3.0), for
// traffic_test. movsim is not part of simon; run this beside it:
//
//   git clone https://github.com/movsim/traffic-simulation-de
//   node application/automotive/reference/movsim_reference.js \
//       traffic-simulation-de/js/models.js application/automotive/reference
//
// movsim's IDM agrees with the published model below the desired speed and
// above the minimum gap, braking less than its 18 m/s^2 cap; its MOBIL agrees
// with the published symmetric criterion with a right bias when politeness is
// zero, its safe deceleration does not change with speed, and its bias is
// within the safe deceleration. The cases here keep to those, and movsim
// draws no noise (QnoiseAccel is zero). It writes:
//
//   movsim_idm.csv    accelerations at 1,000 gaps, speeds and leader speeds,
//                     for four drivers
//   movsim_mobil.csv  decisions at 1,000 sets of accelerations, to either side

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const [models, out] = process.argv.slice(2);
const context = {Math, console, QnoiseAccel: 0, dt: 0.1};
vm.createContext(context);
vm.runInContext(fs.readFileSync(models, 'utf8') +
                '\nthis.IDM = IDM; this.MOBIL = MOBIL;', context);

// A deterministic generator, so the tables repeat: 32-bit xorshift.
let state = 20261003;
function unit() {
  state ^= state << 13; state >>>= 0;
  state ^= state >>> 17;
  state ^= state << 5; state >>>= 0;
  return state / 4294967296;
}
const uniform = (low, high) => low + (high - low) * unit();
const g = (x) => x.toPrecision(17);

const drivers = [[33.33, 1.0, 2.0, 1.0, 1.5], [30.0, 1.5, 2.0, 1.2, 2.0],
                 [15.0, 1.2, 1.5, 1.5, 2.5], [40.0, 0.8, 3.0, 0.8, 1.0]];
let rows = ['v0,T,s0,a,b,gap,speed,leader_speed,acceleration'];
while (rows.length < 1001) {
  const [v0, T, s0, a, b] = drivers[rows.length % drivers.length];
  const gap = uniform(s0 + 0.01, 150.0);
  const speed = uniform(0.0, 0.999 * v0);
  const leader = uniform(0.0, 1.2 * v0);
  const idm = new context.IDM(v0, T, s0, a, b);
  const acc = idm.calcAccDet(gap, speed, leader, 0);
  if (acc <= -idm.bmax) {
    continue;  // Past movsim's cap.
  }
  rows.push([v0, T, s0, a, b, gap, speed, leader, acc].map(g).join(','));
}
fs.writeFileSync(path.join(out, 'movsim_idm.csv'), rows.join('\n') + '\n');

rows = ['safe_deceleration,threshold,right_bias,to_right,self_now,self_after,' +
        'new_follower_after,change'];
for (let i = 0; i < 1000; ++i) {
  const safe = uniform(2.0, 6.0);
  const threshold = uniform(0.0, 0.5);
  const bias = uniform(0.0, 0.5);
  const toRight = unit() < 0.5;
  const now = uniform(-3.0, 1.5);
  const after = now + uniform(-1.0, 1.0);
  const follower = uniform(-8.0, 1.0);
  const mobil = new context.MOBIL(safe, safe, 0.0, threshold, bias);
  const change = mobil.realizeLaneChange(0.5, now, after, follower, toRight);
  rows.push([safe, threshold, bias].map(g).join(',') + ',' + (toRight ? 1 : 0) +
            ',' + [now, after, follower].map(g).join(',') + ',' + (change ? 1 : 0));
}
fs.writeFileSync(path.join(out, 'movsim_mobil.csv'), rows.join('\n') + '\n');
console.log('wrote movsim_idm.csv and movsim_mobil.csv');
