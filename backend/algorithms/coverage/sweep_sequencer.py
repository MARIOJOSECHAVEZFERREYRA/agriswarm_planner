"""Two-level sweep ordering.

Treats each decomposed cell as atomic: picks the order of cells and
whether each is traversed forward or reversed, but never interleaves
sweeps from different cells. This keeps the downstream mission cycles
inside contiguous regions of the field.

Modes:
    'fast' — euclidean distance (optimizer hot path).
    'full' — obstacle-aware geodesic via GeodesicSolver (final emit).

Falls back to a flat NN+2opt over individual sweeps when sweeps lack a
usable 'cell_id'.
"""

import math

from ...utils.geometry import dist as _eu


_MAX_2OPT_ITERS = 20
_POINT_TOL = 1e-9


class SweepSequencer:
    def __init__(self, solver, mode='full', base_point=None,
                 cell_adjacency=None, ugv_polyline=None):
        if mode not in ('fast', 'full'):
            raise ValueError("mode must be 'fast' or 'full'")
        self._solver = solver
        self._mode = mode
        self._base = (
            (float(base_point[0]), float(base_point[1]))
            if base_point is not None else None
        )
        # Optional adjacency graph (cell_id -> {cell_ids}). Consecutive
        # cells in the tour must share a physical boundary when present.
        self._cell_adjacency = cell_adjacency
        # UGV polyline for dynamic mode: cells are ordered along polyline
        # progression instead of proximity to base_point.
        self._ugv_polyline = (
            [(float(p[0]), float(p[1])) for p in ugv_polyline]
            if ugv_polyline is not None and len(ugv_polyline) >= 2
            else None
        )

    # ------------------------------------------------------------------

    def _dist(self, a, b):
        if self._mode == 'fast':
            return _eu(a, b)
        _, d = self._solver.shortest_path(a, b)
        return d

    @staticmethod
    def _clone(sw):
        return {**sw, 'path': list(sw['path'])}

    @staticmethod
    def _flip_sweep(sw):
        """Return a new sweep dict with its path reversed."""
        return {**sw, 'path': list(reversed(sw['path']))}

    # ------------------------------------------------------------------

    def sequence(self, sweeps):
        """Return the sweeps reordered and possibly flipped.

        Cell-aware path is used when every sweep carries a non-null
        cell_id; otherwise falls back to flat NN+2opt.
        """
        if not sweeps:
            return []

        cloned = [self._clone(s) for s in sweeps
                  if s.get('path') and len(s['path']) >= 2]
        if len(cloned) <= 1:
            return cloned

        has_all_ids = all(s.get('cell_id') is not None for s in cloned)
        if has_all_ids:
            cells, cell_ids = self._group_by_cell(cloned)
            if len(cells) >= 2:
                return self._sequence_cells(cells, cell_ids)

        # Fallback: flat per-sweep ordering (legacy behavior).
        ordered = self._nearest_neighbor_flat(cloned)
        ordered = self._two_opt_flat(ordered)
        return ordered

    @staticmethod
    def _group_by_cell(cloned_sweeps):
        """Bucket sweeps by cell_id, preserving input order inside each bucket."""
        buckets = {}
        order = []
        for sw in cloned_sweeps:
            cid = sw['cell_id']
            if cid not in buckets:
                buckets[cid] = []
                order.append(cid)
            buckets[cid].append(sw)
        cells = [buckets[cid] for cid in order]
        return cells, order

    def _cell_endpoints(self, cell, reversed_state):
        """Entry and exit points of a cell in natural or reversed state."""
        if reversed_state:
            return cell[-1]['path'][-1], cell[0]['path'][0]
        return cell[0]['path'][0], cell[-1]['path'][-1]

    def _flatten_cell(self, cell, reversed_state):
        """Clone a cell's sweeps, flipping each one when reversed."""
        if not reversed_state:
            return [self._clone(s) for s in cell]
        return [self._flip_sweep(sw) for sw in reversed(cell)]

    # ------------------------------------------------------------------
    # Polyline projection helpers (dynamic mode)
    # ------------------------------------------------------------------

    def _project_onto_polyline(self, point):
        """Return distance-along-polyline of the closest point on polyline.

        O(V) where V is the number of polyline vertices.
        """
        poly = self._ugv_polyline
        px, py = float(point[0]), float(point[1])
        best_dist_sq = math.inf
        best_d_along = 0.0
        accumulated = 0.0

        for i in range(len(poly) - 1):
            ax, ay = poly[i]
            bx, by = poly[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-18:
                continue
            seg_len = math.sqrt(seg_len_sq)
            t = max(0.0, min(1.0,
                ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
            proj_x = ax + t * dx
            proj_y = ay + t * dy
            dist_sq = (px - proj_x) ** 2 + (py - proj_y) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_d_along = accumulated + t * seg_len
            accumulated += seg_len

        return best_d_along

    @staticmethod
    def _cell_centroid(cell):
        """Average of all sweep midpoints in a cell."""
        sx, sy, n = 0.0, 0.0, 0
        for sw in cell:
            path = sw['path']
            if len(path) >= 2:
                mx = (path[0][0] + path[-1][0]) / 2.0
                my = (path[0][1] + path[-1][1]) / 2.0
                sx += mx
                sy += my
                n += 1
        if n == 0:
            return (0.0, 0.0)
        return (sx / n, sy / n)

    # ------------------------------------------------------------------

    def _sequence_cells(self, cells, cell_ids):
        """Order cells and emit flat sweeps.

        Two modes:
          * UGV polyline provided (dynamic): order cells by their
            centroid's projection onto the polyline so the UAV works
            in the same direction the UGV travels.
          * Otherwise (static): base-anchored NN + 2-opt with
            adjacency preference.
        """
        if self._ugv_polyline is not None:
            return self._sequence_cells_along_polyline(cells, cell_ids)
        return self._sequence_cells_nn(cells, cell_ids)

    def _sequence_cells_along_polyline(self, cells, cell_ids):
        """Order cells by centroid projection onto the UGV polyline."""
        n = len(cells)

        # Project each cell centroid onto polyline.
        projections = []
        for idx in range(n):
            centroid = self._cell_centroid(cells[idx])
            d_along = self._project_onto_polyline(centroid)
            projections.append((d_along, idx))
        projections.sort()

        order = [idx for _, idx in projections]
        states = [False] * n

        # Choose forward/reverse for each cell to minimize transition
        # to the next cell (greedy scan).
        for k in range(len(order)):
            idx = order[k]
            if k + 1 < len(order):
                next_idx = order[k + 1]
                # Evaluate which orientation connects better to next cell.
                _, nat_exit = self._cell_endpoints(cells[idx], False)
                _, rev_exit = self._cell_endpoints(cells[idx], True)
                next_nat_entry, _ = self._cell_endpoints(cells[next_idx], False)
                next_rev_entry, _ = self._cell_endpoints(cells[next_idx], True)
                d_best_next = min(
                    self._dist(nat_exit, next_nat_entry),
                    self._dist(nat_exit, next_rev_entry),
                )
                d_best_next_rev = min(
                    self._dist(rev_exit, next_nat_entry),
                    self._dist(rev_exit, next_rev_entry),
                )
                states[idx] = d_best_next_rev < d_best_next
            else:
                # Last cell: orient toward base if available,
                # otherwise keep natural.
                if self._base is not None:
                    _, nat_exit = self._cell_endpoints(cells[idx], False)
                    _, rev_exit = self._cell_endpoints(cells[idx], True)
                    states[idx] = (
                        self._dist(rev_exit, self._base)
                        < self._dist(nat_exit, self._base)
                    )

        # 2-opt refinement (position 0 pinned).
        order, states = self._two_opt_cells(cells, order, states, cell_ids)

        flat = []
        for idx in order:
            flat.extend(self._flatten_cell(cells[idx], states[idx]))
        return flat

    def _sequence_cells_nn(self, cells, cell_ids):
        """Order cells via base-anchored NN + 2-opt, then emit flat sweeps.

        When a cell adjacency graph is provided, NN prefers neighbors
        of the current cell so the tour stays topologically connected;
        it falls back to the global nearest only when no adjacent
        candidate remains.
        """
        n = len(cells)

        remaining = list(range(n))
        states = [False] * n  # reversed?
        order = []

        if self._base is not None:
            cursor = self._base
            cur_cell_id = None
        else:
            ordered_first = 0
            cursor = cells[ordered_first][-1]['path'][-1]
            order.append(ordered_first)
            states[ordered_first] = False
            remaining.remove(ordered_first)
            cur_cell_id = cell_ids[ordered_first]

        while remaining:
            # Prefer adjacent cells when adjacency info is available;
            # otherwise consider all remaining.
            candidates = remaining
            if (self._cell_adjacency is not None
                    and cur_cell_id is not None):
                neighbors = self._cell_adjacency.get(cur_cell_id, set())
                adj_candidates = [
                    idx for idx in remaining
                    if cell_ids[idx] in neighbors
                ]
                if adj_candidates:
                    candidates = adj_candidates

            best_i = candidates[0]
            best_d = math.inf
            best_rev = False
            for idx in candidates:
                cell = cells[idx]
                nat_start, _ = self._cell_endpoints(cell, False)
                rev_start, _ = self._cell_endpoints(cell, True)
                d_nat = self._dist(cursor, nat_start)
                d_rev = self._dist(cursor, rev_start)
                if d_nat < best_d:
                    best_d, best_i, best_rev = d_nat, idx, False
                if d_rev < best_d:
                    best_d, best_i, best_rev = d_rev, idx, True

            remaining.remove(best_i)
            states[best_i] = best_rev
            order.append(best_i)
            _, exit_pt = self._cell_endpoints(cells[best_i], best_rev)
            cursor = exit_pt
            cur_cell_id = cell_ids[best_i]

        order, states = self._two_opt_cells(cells, order, states, cell_ids)

        flat = []
        for idx in order:
            flat.extend(self._flatten_cell(cells[idx], states[idx]))
        return flat

    def _count_non_adjacent(self, order, cell_ids):
        """Count consecutive cell pairs that break the adjacency graph."""
        if self._cell_adjacency is None:
            return 0
        n = len(order)
        count = 0
        for k in range(n - 1):
            a = cell_ids[order[k]]
            b = cell_ids[order[k + 1]]
            if b not in self._cell_adjacency.get(a, set()):
                count += 1
        return count

    def _two_opt_cells(self, cells, order, states, cell_ids):
        """Refine the cell order by reversing sub-ranges that lower cost.

        Reversing [i..j] reverses the cell order and flips the state of
        each cell in the range. Only internal edges are re-costed;
        position 0 stays pinned so NN's base-anchored entry is never
        displaced. When an adjacency graph is supplied, swaps that add
        non-adjacent transitions are rejected.
        """
        n = len(order)
        if n < 3:
            return order, states

        baseline_non_adj = self._count_non_adjacent(order, cell_ids)

        for _ in range(_MAX_2OPT_ITERS):
            improved = False
            for i in range(1, n - 1):
                for j in range(i + 1, n):
                    new_order = order[:i] + list(reversed(order[i:j + 1])) + order[j + 1:]
                    new_states = list(states)
                    for k in range(i, j + 1):
                        new_states[new_order[k]] = not states[new_order[k]]

                    # Only the two edges around the reversed range
                    # change; all internal edges are identical.
                    def _entry_of(pos, seq_order, seq_states):
                        cell = cells[seq_order[pos]]
                        return self._cell_endpoints(cell, seq_states[seq_order[pos]])[0]

                    def _exit_of(pos, seq_order, seq_states):
                        cell = cells[seq_order[pos]]
                        return self._cell_endpoints(cell, seq_states[seq_order[pos]])[1]

                    left_anchor = _exit_of(i - 1, order, states)
                    right_anchor = _entry_of(j + 1, order, states) if j + 1 < n else None

                    left_old = (self._dist(left_anchor, _entry_of(i, order, states))
                                if left_anchor is not None else 0.0)
                    left_new = (self._dist(left_anchor, _entry_of(i, new_order, new_states))
                                if left_anchor is not None else 0.0)

                    right_old = (self._dist(_exit_of(j, order, states), right_anchor)
                                 if right_anchor is not None else 0.0)
                    right_new = (self._dist(_exit_of(j, new_order, new_states), right_anchor)
                                 if right_anchor is not None else 0.0)

                    delta = (left_new + right_new) - (left_old + right_old)
                    if delta < -_POINT_TOL:
                        # Reject swaps that add non-adjacent transitions.
                        new_non_adj = self._count_non_adjacent(new_order, cell_ids)
                        if new_non_adj > baseline_non_adj:
                            continue
                        order = new_order
                        states = new_states
                        baseline_non_adj = new_non_adj
                        improved = True
                        break
                if improved:
                    break
            if not improved:
                break

        return order, states

    # Flat NN+2opt fallback (used when sweeps lack a usable cell_id).

    def _nearest_neighbor_flat(self, sweeps):
        remaining = list(sweeps)

        if self._base is not None:
            best_i = 0
            best_d = math.inf
            best_flip = False
            for i, sw in enumerate(remaining):
                d0 = self._dist(self._base, sw['path'][0])
                d1 = self._dist(self._base, sw['path'][-1])
                if d0 < best_d:
                    best_d, best_i, best_flip = d0, i, False
                if d1 < best_d:
                    best_d, best_i, best_flip = d1, i, True
            first = remaining.pop(best_i)
            if best_flip:
                first['path'] = list(reversed(first['path']))
            ordered = [first]
        else:
            ordered = [remaining.pop(0)]

        while remaining:
            cur_end = ordered[-1]['path'][-1]
            best_i = 0
            best_d = math.inf
            best_flip = False
            for i, cand in enumerate(remaining):
                d_nat = self._dist(cur_end, cand['path'][0])
                d_flip = self._dist(cur_end, cand['path'][-1])
                if d_nat < best_d:
                    best_d, best_i, best_flip = d_nat, i, False
                if d_flip < best_d:
                    best_d, best_i, best_flip = d_flip, i, True
            nxt = remaining.pop(best_i)
            if best_flip:
                nxt['path'] = list(reversed(nxt['path']))
            ordered.append(nxt)

        return ordered

    def _two_opt_flat(self, ordered):
        """Refine sweep order by reversing sub-ranges that lower ferry cost.

        The virtual base→first and last→base edges are excluded from
        the delta so a base-close sweep at the tour ends cannot drag
        other sweeps into long internal jumps. Position 0 stays pinned
        (loop starts at i=1) to preserve the NN base anchor.
        """
        n = len(ordered)
        if n < 3:
            return ordered

        for _ in range(_MAX_2OPT_ITERS):
            improved = False
            for i in range(1, n - 1):
                for j in range(i + 1, n):
                    a_end = ordered[i - 1]['path'][-1]
                    d_start = ordered[j + 1]['path'][0] if j + 1 < n else None

                    b_start = ordered[i]['path'][0]
                    c_end = ordered[j]['path'][-1]

                    left_old = self._dist(a_end, b_start) if a_end is not None else 0.0
                    left_new = self._dist(a_end, c_end) if a_end is not None else 0.0
                    right_old = self._dist(c_end, d_start) if d_start is not None else 0.0
                    right_new = self._dist(b_start, d_start) if d_start is not None else 0.0

                    delta = (left_new + right_new) - (left_old + right_old)
                    if delta < -_POINT_TOL:
                        ordered[i:j + 1] = list(reversed(ordered[i:j + 1]))
                        for k in range(i, j + 1):
                            ordered[k]['path'] = list(reversed(ordered[k]['path']))
                        improved = True
                        break
                if improved:
                    break
            if not improved:
                break

        return ordered
