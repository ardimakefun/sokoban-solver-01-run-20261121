// Sokoban solver — main.ts (ditulis dari nol di sesi ini untuk bounty make.fun
// sokoban-solver-01-run-20261121; Node 22, erasable TypeScript saja, tanpa dependensi).
//
// Cara kerja: tiap level dipecah jadi ruangan-ruangan yang independen (box tidak
// pernah pindah ruangan: box yang terdorong ke lorong tidak bisa kembali), lalu
// tiap ruangan diselesaikan push-optimal dengan A* atas push. Heuristiknya
// admissible: biaya matching minimum box->goal (Hungarian) dengan jarak grid
// terpendek abaikan box lain. Pruning: petak mati (reverse-pull BFS dari goal)
// dan blok 2x2 beku. Semua deterministik: tanpa jam, tanpa random, tie-break
// tetap; budget dibatasi oleh kerja (jumlah ekspansi), bukan oleh waktu.
//
// Interface: baca /input/<id>/level.txt (atau MAKEFUN_INPUT_DIR), tulis
// /output/<id>/solution.txt (atau MAKEFUN_OUTPUT_DIR) berisi langkah LURD.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

/** Arah: [dx, dy, huruf jalan, huruf push]. Urutan tetap: u, d, l, r. */
const DIRS: ReadonlyArray<readonly [number, number, string, string]> = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  // Bingkai terluar selalu tembok: tidak ada yang keluar dari papan.
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  goals.sort((a, b) => a - b);
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/**
 * Petak hidup: petak dari mana satu box (abaikan box lain) masih bisa didorong
 * ke suatu goal. Dihitung mundur dari goal: box di x bisa ditarik dari y = x - d
 * bila y dan y - d lantai terbuka. Box di petak mati tidak pernah bisa diselamatkan.
 */
function liveSquares(lv: Level): Uint8Array {
  const { w, wall, goals } = lv;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/**
 * Region yang bisa dijangkau player dari `from` (tembok dan box jadi penghalang),
 * mengembalikan petak terendah di region itu. Region ditentukan oleh petak
 * terendahnya, jadi petak itu cukup untuk mengidentifikasi region dalam kunci state.
 */
function regionLow(lv: Level, occupied: Uint8Array, from: number, seen: Int32Array, stamp: number, stack: Int32Array): number {
  const { w, wall } = lv;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    // Tetangga: atas, bawah, kiri, kanan (urutan tetap, hasil tidak tergantung urutan).
    const q0 = p - w;
    const q1 = p + w;
    const q2 = p - 1;
    const q3 = p + 1;
    if (wall[q0] === 0 && occupied[q0] === 0 && seen[q0] !== stamp) { seen[q0] = stamp; stack[n++] = q0; }
    if (wall[q1] === 0 && occupied[q1] === 0 && seen[q1] !== stamp) { seen[q1] = stamp; stack[n++] = q1; }
    if (wall[q2] === 0 && occupied[q2] === 0 && seen[q2] !== stamp) { seen[q2] = stamp; stack[n++] = q2; }
    if (wall[q3] === 0 && occupied[q3] === 0 && seen[q3] !== stamp) { seen[q3] = stamp; stack[n++] = q3; }
  }
  return low;
}

/** Apakah box yang baru didorong ke `t` terjebak dalam blok 2x2 tembok/box berisi box di luar goal. */
function frozenBlock(lv: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = lv;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** Jarak grid terpendek dari tiap goal ke semua petak (abaikan box), untuk heuristik. */
function goalDistances(lv: Level): Int32Array[] {
  const { w, wall, goals } = lv;
  const out: Int32Array[] = [];
  for (const g of goals) {
    const dist = new Int32Array(wall.length).fill(-1);
    dist[g] = 0;
    const queue = [g];
    for (let qi = 0; qi < queue.length; qi++) {
      const p = queue[qi]!;
      for (const [dx, dy] of DIRS) {
        const q = p + dy * w + dx;
        if (!wall[q] && dist[q] < 0) {
          dist[q] = dist[p]! + 1;
          queue.push(q);
        }
      }
    }
    out.push(dist);
  }
  return out;
}

/**
 * Biaya matching minimum box->goal (algoritma Hungarian O(n^3), biaya = jarak
 * grid). Mengembalikan Infinity bila tidak ada matching sempurna berbobot
 * hingga (level tidak mungkin solved). Implementasi potensial standar.
 */
function hungarian(cost: number[][]): number {
  const n = cost.length;
  if (n === 0) return 0;
  const INF = 1e9;
  // u, v potensial; p[j] = baris yang dipasangkan ke kolom j; way[j] = kolom sebelumnya.
  const u = new Array<number>(n + 1).fill(0);
  const v = new Array<number>(n + 1).fill(0);
  const p = new Array<number>(n + 1).fill(0);
  const way = new Array<number>(n + 1).fill(0);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Array<number>(n + 1).fill(INF);
    const used = new Array<boolean>(n + 1).fill(false);
    do {
      used[j0] = true;
      const i0 = p[j0]!;
      let delta = INF;
      let j1 = 0;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 - 1]![j - 1]! - u[i0]! - v[j]!;
        if (cur < minv[j]!) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j]! < delta) {
          delta = minv[j]!;
          j1 = j;
        }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) {
          u[p[j]!]! += delta;
          v[j]! -= delta;
        } else {
          minv[j]! -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do {
      const j1 = way[j0]!;
      p[j0] = p[j1]!;
      j0 = j1;
    } while (j0 !== 0);
  }
  let total = 0;
  for (let j = 1; j <= n; j++) {
    const c = cost[p[j]! - 1]![j - 1]!;
    if (c >= INF / 2) return Infinity;
    total += c;
  }
  return total;
}

/**
 * Heuristik admissible: matching minimum box->goal atas jarak grid (abaikan box lain).
 * Bila box lebih banyak dari goal, level tidak mungkin solved (ada box yang selalu
 * di luar goal) → kembalikan BIG. Bila box lebih sedikit, baris dummy berbiaya 0
 * melengkapi matriks jadi bujur sangkar, jadi matching memilih subset goal terbaik.
 */
function heuristic(boxes: number[], dists: Int32Array[], big: number): number {
  const nB = boxes.length;
  const nG = dists.length;
  if (nB === 0) return 0;
  if (nB > nG) return big;
  const m = nG;
  const cost: number[][] = [];
  for (let i = 0; i < m; i++) {
    const row: number[] = [];
    for (let j = 0; j < m; j++) {
      if (i < nB) {
        const d = dists[j]![boxes[i]!]!;
        row.push(d < 0 ? big : d);
      } else {
        row.push(0);
      }
    }
    cost.push(row);
  }
  const h = hungarian(cost);
  return h === Infinity ? big : h;
}

/** Jalan terpendek dari `from` ke `to` menghindari tembok dan box (BFS, arah u,d,l,r). */
function walkTo(lv: Level, occupied: Uint8Array, from: number, to: number): string | null {
  const { w, wall } = lv;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return null;
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

/** Posisi player setelah memainkan `moves` (abaikan case) dari `from`. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) {
    p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  }
  return p;
}

interface SearchResult {
  pushes: number;
  moves: string;
}

/** Anggaran kerja global per level (jumlah ekspansi), dibagi semua pencarian. */
interface Budget {
  used: number;
  limit: number;
}

/**
 * Pencarian push-optimal (A*) atas satu level (atau satu ruangan sebagai sub-level).
 * State = box terurut + region player. Kunci state = regionLow + box.
 * Mengembalikan null bila menyerah (lewat cap ekspansi / anggaran habis) atau terbukti tidak ada solusi.
 */
function searchPushOptimal(lv: Level, maxExpansions: number, greedy: boolean, budget: Budget): SearchResult | null {
  const { w, wall, goals } = lv;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(lv);
  const dists = goalDistances(lv);
  const BIG = 1e9;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  if (lv.boxes.some((b) => !live[b])) return null;
  if (solved(lv.boxes)) return { pushes: 0, moves: '' };

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  // Heap biner: entri [kunci-urut, stateIdx]. Urutan: f naik, lalu g turun (lebih dalam dulu), lalu seq naik.
  const heapKey: number[] = [];
  const heapIdx: number[] = [];
  const heapSeq: number[] = [];
  let seqCounter = 0;
  const heapLess = (a: number, b: number): boolean => {
    if (heapKey[a] !== heapKey[b]) return heapKey[a]! < heapKey[b]!;
    if (heapG[a] !== heapG[b]) return heapG[a]! > heapG[b]!;
    return heapSeq[a]! < heapSeq[b]!;
  };
  const heapG: number[] = [];
  const heapPush = (key: number, g: number, idx: number): void => {
    let i = heapKey.length;
    heapKey.push(key);
    heapG.push(g);
    heapIdx.push(idx);
    heapSeq.push(seqCounter++);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(i, p)) break;
      for (const arr of [heapKey, heapG, heapIdx, heapSeq]) {
        const t = arr[i]!;
        arr[i] = arr[p]!;
        arr[p] = t;
      }
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heapIdx[0]!;
    const last = heapKey.length - 1;
    heapKey[0] = heapKey[last]!;
    heapG[0] = heapG[last]!;
    heapIdx[0] = heapIdx[last]!;
    heapSeq[0] = heapSeq[last]!;
    heapKey.pop();
    heapG.pop();
    heapIdx.pop();
    heapSeq.pop();
    for (let i = 0; ; ) {
      let c = 2 * i + 1;
      if (c >= heapKey.length) break;
      if (c + 1 < heapKey.length && heapLess(c + 1, c)) c++;
      if (!heapLess(c, i)) break;
      for (const arr of [heapKey, heapG, heapIdx, heapSeq]) {
        const t = arr[i]!;
        arr[i] = arr[c]!;
        arr[c] = t;
      }
      i = c;
    }
    return top;
  };

  const keyOf = (boxes: number[], low: number): string => low + '|' + boxes.join(',');
  const statesBoxes: number[][] = [lv.boxes.slice()];
  const statesPlayer: number[] = [lv.player];
  const parent: number[] = [-1];
  // push yang membuat state ini dari parent-nya: [from, to, huruf push].
  const pushInfo: Array<[number, number, string] | null> = [null];
  const gScore: number[] = [0];
  const best = new Map<string, number>();
  for (const b of lv.boxes) occupied[b] = 1;
  const startLow = regionLow(lv, occupied, lv.player, seen, ++stamp, stack);
  for (const b of lv.boxes) occupied[b] = 0;
  best.set(keyOf(lv.boxes, startLow), 0);
  const h0 = heuristic(lv.boxes, dists, BIG);
  if (h0 >= BIG) return null;
  heapPush(greedy ? h0 : h0, 0, 0);

  let expanded = 0;
  let found = -1;
  while (heapKey.length > 0 && expanded < maxExpansions && budget.used < budget.limit) {
    const s = heapPop();
    const g = gScore[s]!;
    // Entri heap basi (state yang sama sudah dicapai dengan g lebih kecil)?
    const boxes = statesBoxes[s]!;
    for (const b of boxes) occupied[b] = 1;
    const low = regionLow(lv, occupied, statesPlayer[s]!, seen, ++stamp, stack);
    if (best.get(keyOf(boxes, low)) !== g) {
      for (const b of boxes) occupied[b] = 0;
      continue;
    }
    if (solved(boxes)) {
      found = s;
      for (const b of boxes) occupied[b] = 0;
      break;
    }
    expanded++;
    budget.used++;
    const here = stamp;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy, , pushLetter] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        // Player harus bisa berdiri di b - d (di region), dan t harus lantai kosong yang hidup.
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        if (frozenBlock(lv, occupied, goal, t)) {
          occupied[t] = 0;
          occupied[b] = 1;
          continue;
        }
        const next = boxes.slice();
        next[bi] = t;
        next.sort((x, y) => x - y);
        const nextLow = regionLow(lv, occupied, b, mark, ++stamp, stack);
        const key = keyOf(next, nextLow);
        const ng = g + 1;
        const prevBest = best.get(key);
        occupied[t] = 0;
        occupied[b] = 1;
        if (prevBest !== undefined && prevBest <= ng) continue;
        const h = heuristic(next, dists, BIG);
        if (h >= BIG) continue;
        if (statesBoxes.length >= maxExpansions) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        best.set(key, ng);
        const idx = statesBoxes.length;
        statesBoxes.push(next);
        statesPlayer.push(b);
        parent.push(s);
        pushInfo.push([b, t, pushLetter]);
        gScore.push(ng);
        heapPush(greedy ? h : ng + h, ng, idx);
        if (solved(next)) {
          found = idx;
          break;
        }
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;
  // Rekonstruksi: jalan ke tiap push, lalu push-nya.
  const path: number[] = [];
  for (let s = found; s > 0; s = parent[s]!) path.push(s);
  path.reverse();
  let moves = '';
  let player = lv.player;
  let boxes = lv.boxes.slice();
  for (const s of path) {
    const nextBoxes = statesBoxes[s]!;
    const [from, to, pushLetter] = pushInfo[s]!;
    const d = to - from;
    for (const b of boxes) occupied[b] = 1;
    const walk = walkTo(lv, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    if (walk === null) return null;
    moves += walk + pushLetter;
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

interface Room {
  boxes: number[];
  goals: number[];
  minSquare: number;
}

/**
 * Pecah level jadi ruangan: potong petak mati di baris hallway (baris player),
 * lalu komponen terhubung dari lantai yang tersisa. Null bila level bukan
 * sekumpulan ruangan (box di lorong, atau komponen berisi box != goal).
 */
function roomsOf(lv: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = lv;
  const live = liveSquares(lv);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i]! + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [], minSquare: Math.min(...squares) });
  }
  for (const b of boxes) {
    if (part[b] < 0) return null;
    rooms[part[b]]!.boxes.push(b);
  }
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]]!.goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Replay langkah pada level: kembalikan {ok, pushes} (ok = semua box di goal, tanpa langkah ilegal). */
function replay(lv: Level, moves: string): { ok: boolean; pushes: number } {
  const { w, wall } = lv;
  const goal = new Uint8Array(wall.length);
  for (const g of lv.goals) goal[g] = 1;
  const box = new Uint8Array(wall.length);
  for (const b of lv.boxes) box[b] = 1;
  let player = lv.player;
  let pushes = 0;
  for (const ch of moves.toLowerCase()) {
    const d = ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
    const next = player + d;
    if (wall[next]) return { ok: false, pushes };
    if (box[next]) {
      const beyond = next + d;
      if (wall[beyond] || box[beyond]) return { ok: false, pushes };
      box[next] = 0;
      box[beyond] = 1;
      pushes++;
    }
    player = next;
  }
  for (let i = 0; i < box.length; i++) {
    if (box[i] && !goal[i]) return { ok: false, pushes };
  }
  return { ok: true, pushes };
}

const ROOM_EXPANSIONS = 300_000;
const WHOLE_EXPANSIONS = 500_000;
const GREEDY_EXPANSIONS = 200_000;
/** Anggaran kerja total per level (semua pencarian digabung): batas atas waktu. */
const LEVEL_BUDGET = 1_000_000;

/** Selesaikan satu level; kembalikan langkah LURD ('' bila menyerah — format tetap valid). */
function solveLevel(text: string): string {
  const lv = parseLevel(text);
  if (lv.boxes.length === 0) return '';
  if (lv.boxes.every((b) => lv.goals.includes(b))) return '';
  const budget: Budget = { used: 0, limit: LEVEL_BUDGET };

  // Coba urutan ruangan: urutan baku dulu, lalu semua permutasi bila perlu (maks 4 ruangan).
  const rooms = roomsOf(lv);
  if (rooms !== null) {
    rooms.sort((a, b) => a.minSquare - b.minSquare);
    const orders: Room[][] = [rooms];
    if (rooms.length > 1 && rooms.length <= 4) {
      const permute = (arr: Room[], start: number): void => {
        if (start === arr.length) {
          orders.push(arr.slice());
          return;
        }
        for (let i = start; i < arr.length; i++) {
          const tmp = arr[start]!;
          arr[start] = arr[i]!;
          arr[i] = tmp;
          permute(arr, start + 1);
          const tmp2 = arr[start]!;
          arr[start] = arr[i]!;
          arr[i] = tmp2;
        }
      };
      permute(rooms.slice(), 0);
    }
    for (const order of orders) {
      const result = tryRoomOrder(lv, order, budget);
      if (result !== null) {
        const check = replay(lv, result);
        if (check.ok) return result;
      }
      if (budget.used >= budget.limit) break;
    }
  }

  // Fallback: cari di seluruh level sekaligus (A* optimal, lalu greedy).
  const whole = searchPushOptimal(lv, WHOLE_EXPANSIONS, false, budget);
  if (whole !== null) {
    const check = replay(lv, whole.moves);
    if (check.ok) return whole.moves;
  }
  const greedy = searchPushOptimal(lv, GREEDY_EXPANSIONS, true, budget);
  if (greedy !== null) {
    const check = replay(lv, greedy.moves);
    if (check.ok) return greedy.moves;
  }
  return '';
}

/** Selesaikan ruangan-ruangan dalam satu urutan tertentu; null bila ada ruangan gagal. */
function tryRoomOrder(lv: Level, order: Room[], budget: Budget): string | null {
  const currentBoxes = new Map<number, number[]>();
  for (const r of order) currentBoxes.set(r.minSquare, r.boxes.slice());
  let player = lv.player;
  let moves = '';
  for (const room of order) {
    const wall = lv.wall.slice();
    for (const other of order) {
      if (other === room) continue;
      for (const b of currentBoxes.get(other.minSquare)!) wall[b] = 1;
    }
    const sub: Level = {
      w: lv.w,
      h: lv.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: currentBoxes.get(room.minSquare)!.slice().sort((a, b) => a - b),
      player,
    };
    const solved = searchPushOptimal(sub, ROOM_EXPANSIONS, false, budget)
      ?? searchPushOptimal(sub, GREEDY_EXPANSIONS, true, budget);
    if (solved === null) return null;
    moves += solved.moves;
    player = endOf(lv.w, player, solved.moves);
    currentBoxes.set(room.minSquare, room.goals.slice());
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  // Satu level yang gagal (apa pun sebabnya) tidak boleh menggugurkan run:
  // tulis jawaban kosong (format valid, 0 poin) lalu lanjut ke level berikut.
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
