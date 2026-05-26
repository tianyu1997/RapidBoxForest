# Exp.3 SBF Construction Mechanism Ablation

## Claim

The grow-stop policy, connector, protected consolidation, and targeted refinement produce reusable audited corridors with controlled build and repair cost.

## Hypotheses

- Moderate quality floors improve repair-free query success with diminishing returns.
- Connector and segment edges improve connectivity and path quality.
- Direct aggressive merger can create unsafe GCS behavior; protected handling preserves audited corridors.
- Targeted refinement should reduce final failure or solved-but-unsafe cases, but its repair cost must be reported.

## Existing Runners

- `experiments/paper_04_grower_tradeoff.py`
- `experiments/paper_07_merger_protected_study.py`
- `experiments/paper_04_marcucci_combined.py`

## Protocol

1. Sweep quality floor and post-connect growth budget on shelf+IIWA.
2. Run with and without corridor refinement and bridge-repaired queries.
3. Compare consolidation/merger modes using protected study outputs.
4. Record per-query repair fields rather than only aggregate repair counts.

## Comparison Groups

- Quality floor: 0, 32, 64, 128, 224, 512.
- Corridor refinement: on/off.
- Connector/bridge: default, no bridge-repaired queries, no connector if runner supports it.
- Merger: none, direct/aggressive, protected.

## Metrics

- Build time, boxes, components/islands, segment edges.
- Repair-free query SR.
- Final audit SR.
- Repair trigger rate, iterations, boxes added, repair time.
- Path length and length ratio to reference.

## Visualizations

- Repair-free SR vs box count.
- Path length vs build time Pareto.
- Stacked build/repair timing bars.
- Merger safety table.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_04_grower_tradeoff.py \
  --quality-grid 0,128 \
  --seeds 1 \
  --threads 2 \
  --max-boxes 400 \
  --ffb-depth 80 \
  --corridor-refine \
  --bridge-repaired-queries \
  --out-json outputs/paper/tro2026_exp03_grower_smoke.json \
  --out-csv outputs/paper/tro2026_exp03_grower_smoke.csv \
  --out-md outputs/paper/tro2026_exp03_grower_smoke.md \
  --out-plot outputs/paper/tro2026_exp03_grower_smoke.png
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_04_grower_tradeoff.py \
  --quality-grid 0,32,64,96,128,160,224,320,512 \
  --seeds 10 \
  --threads 8 \
  --max-boxes 5000 \
  --ffb-depth 120 \
  --corridor-refine \
  --bridge-repaired-queries \
  --out-json outputs/paper/tro2026_exp03_grower_full.json \
  --out-csv outputs/paper/tro2026_exp03_grower_full.csv \
  --out-md outputs/paper/tro2026_exp03_grower_full.md \
  --out-plot outputs/paper/tro2026_exp03_grower_full.png
```

## Acceptance Criteria

- Repair counts vary by query/condition or the paper explicitly reports the flat result as a limitation.
- No merged/protected row is counted successful unless final audit passes.
- The selected default quality floor is justified by a visible trade-off, not by a single anecdote.
