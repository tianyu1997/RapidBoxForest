# Exp.1 Link-Envelope Foundation

## Claim

Endpoint iAABB sources and link-envelope representations expose measurable tightness, runtime, and storage trade-offs. This supports the link interval envelope package as an independent foundation for SBF.

## Hypotheses

- IFK is certified and fast but looser.
- CritSample is tighter but advisory.
- Analytical is certified and tight but much slower.
- LinkIAABB subdivision and Hull/Grid representations improve tightness at measurable storage/runtime cost.

## Existing Runners

- `experiments/paper_01_epiaabb_pipeline.py`
- `experiments/paper_02_link_envelope_pipeline.py`

## Protocol

1. Generate paired joint interval boxes across fixed width bins.
2. Evaluate endpoint sources: IFK, CritSample, Analytical, MC/reference.
3. Save the fixed box table from the endpoint run.
4. Feed the same box table into the link-envelope runner.
5. Evaluate LinkIAABB S=1/2/4/8, HullGrid variants, and IFK controls when feasible.

## Comparison Groups

- Endpoint source: IFK, CritSample, Analytical, MC.
- Link representation: LinkIAABB subdivisions, HullGrid deltas, strict pad/no-pad diagnostics.
- Width bin: fixed widths from 0.02 to 0.50 rad.

## Metrics

- Endpoint runtime median/IQR.
- Envelope volume and gap to reference.
- Certified/advisory flag.
- Link-envelope volume, payload bytes, grid occupancy, materialization count.
- Optional: false-positive obstacle filtering if obstacle probes are added.

## Visualizations

- Time-tightness Pareto plot.
- Storage-tightness Pareto plot.
- Width-bin curves by endpoint source.

## Smoke Command

```bash
PYTHONPATH="$PYTHONPATH_LIE" $PYTHON_LIE experiments/paper_01_epiaabb_pipeline.py \
  --n-boxes 5 \
  --fixed-widths 0.02,0.1 \
  --min-samples 50 \
  --ref-samples 200 \
  --sources IFK,CritSample,MC \
  --out-json outputs/paper/tro2026_exp01_endpoint_smoke.json

PYTHONPATH="$PYTHONPATH_LIE" $PYTHON_LIE experiments/paper_02_link_envelope_pipeline.py \
  --boxes-json outputs/paper/tro2026_exp01_endpoint_smoke_fixed_boxes.json \
  --max-boxes-per-width 5 \
  --variants link_s1,link_s4,hull_d0.04 \
  --out-json outputs/paper/tro2026_exp01_link_smoke.json
```

## Full Command

```bash
PYTHONPATH="$PYTHONPATH_LIE" $PYTHON_LIE experiments/paper_01_epiaabb_pipeline.py \
  --n-boxes 400 \
  --fixed-widths default \
  --sources IFK,CritSample,Analytical,MC \
  --endpoint-threads 8 \
  --out-json outputs/paper/tro2026_exp01_endpoint_full.json

PYTHONPATH="$PYTHONPATH_LIE" $PYTHON_LIE experiments/paper_02_link_envelope_pipeline.py \
  --boxes-json outputs/paper/tro2026_exp01_endpoint_full_fixed_boxes.json \
  --variants link_s1,link_s2,link_s4,link_s8,hull_d0.02,hull_d0.04,hull_d0.06,hull_d0.08,ifk_link_s4,ifk_hull_d0.04 \
  --endpoint-threads 8 \
  --batch-threads 8 \
  --out-json outputs/paper/tro2026_exp01_link_full.json
```

## Acceptance Criteria

- Both artifacts parse and contain nonempty width/source/variant rows.
- Full result exposes a clear Pareto frontier, not only a single best method.
- Advisory methods are labeled advisory in tables.
