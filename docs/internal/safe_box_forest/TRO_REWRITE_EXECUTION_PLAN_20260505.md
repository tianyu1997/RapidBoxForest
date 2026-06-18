# TRO Rewrite And Experiment Execution Plan

Date: 2026-05-05; revised experiment framework: 2026-05-07

This plan drives the current execution batch: repair or correctly bound the GCS workflow, migrate the paper into a new TRO rewrite folder, redesign the experiment section around the standalone packages, run the available experiments, generate paper tables, and integrate the results into the manuscript.

## Stage 1: GCS Audit Diagnosis

Goal: explain why direct GCS over generated SBF boxes solves the five Marcucci queries but passes strict collision audit on zero of them.

Actions:
- Extend the direct-GCS analysis with per-query failure diagnostics.
- Record the failed segment index, failed interpolation sample, approximate collision configuration, distance to the SBF audited path, graph size, path length, and whether the direct-GCS result was only solved or also audit-passed.
- Preserve the rule that GCS success means strict audit pass, not merely solver success.

Artifacts:
- `outputs/paper/marcucci_merger_gcs_diagnostics.json`
- Negative/control result row for the paper: direct GCS over provisional boxes is solved-but-unsafe.

Gate:
- The diagnostic file must identify an audit result for every queried path.

## Stage 2: Audited Corridor GCS

Goal: make GCS consume only a query-local SBF corridor whose transitions have been audited or are inherited from an audited SBF path.

Actions:
- Add `experiments/paper_04_audited_corridor_gcs.py`.
- Build SBF with the current quality-aware Exp.4 configuration.
- For each query, obtain the strict SBF path with repaired-query bridge enabled.
- Create a corridor-filtered GCS graph from the query box sequence, one-hop corridor neighbors, and segment-edge endpoints rather than all geometric edges.
- Solve GCS, then run the same strict collision audit used by SBF.
- If GCS audit fails, fall back to the SBF audited path for the positive safety certificate and keep the GCS row marked as solved-but-unsafe.

Artifacts:
- `outputs/paper/marcucci_audited_corridor_gcs.json`
- `outputs/paper/marcucci_audited_corridor_gcs_paths.json`

Gate:
- `gcs_success_count` must equal strict audit pass count.
- Any fallback path must be explicitly labeled `sbf_audited_fallback`, not GCS success.

## Stage 3: TRO Rewrite Folder

Goal: create a clean manuscript workspace without touching the original v6 paper entry files.

Actions:
- Maintain `cpp/SBF/doc/paper/tro_rewrite_2026/` as the active TRO rewrite folder.
- Copy the active SBF paper source into `sbf_tro_2026.tex`.
- Copy references and available generated table/figure assets.
- Replace the old experiment section with a new experiment design and generated-table includes.

Artifacts:
- `doc/paper/tro_rewrite_2026/sbf_tro_2026.tex`
- `doc/paper/tro_rewrite_2026/references.bib`
- `doc/paper/tro_rewrite_2026/generated/*.tex`

Gate:
- The original files `cpp/v6/doc/box_aabb_v6_paper_en.tex` and `cpp/v6/doc/box_aabb_v6_paper_zh.tex` remain unchanged.

## Stage 4: Main Experiment Framework

Goal: make the main experimental evidence high density and focused on algorithm efficiency plus path-quality performance. Safety is handled by theory and strict final audit, with one protected-fallback experiment in the main text.

Main-text experiments:
- Exp.A Shelf+IIWA evidence-profile efficiency: five canonical Marcucci queries; report build time, box count, query p50, audit p50, repair count, audit SR, and validation boundary. Do not report path quality in this table: profile rows use different grown graphs, so path length is not a nested shortest-path comparison. This replaces the old low-density endpoint/link/envelope profile split.
- Exp.B Reuse and amortized throughput: IIWA cold/warm/cross-scene LECT reuse table plus one amortization figure. The numeric amortization table is not a main-text table.
- Exp.C Shelf+IIWA efficiency and path quality: selected SBF-SH configuration versus IRIS-NP+GCS, PRM, RRT-Connect, and BIT* on the five named Marcucci queries. The SBF row must use `outputs/paper/tro2026_exp04_marcucci_full.json` (quality floor 128 / optimized corridor refinement), not the older support-hull-only default artifact that regresses CS->LB.
- Exp.D Cross-robot random-scene quality and efficiency: UR5/Panda easy/medium random-obstacle scenes versus same-oracle RRT-Connect. Report only comparable non-empty metrics: SBF build, SBF query/path/audit SR, RRT query/path/audit SR, and baseline trial count.
- Exp.E Single safety fallback check: Shelf+IIWA GCS composition as a negative-control plus protected-fallback experiment. Direct solver success is not counted unless strict audit passes; fallback is explicitly reported in prose, not as a main table.

Appendix-only diagnostics:
- Endpoint-source microbenchmarks, link-envelope representation tables, FFB depth/grow-stop sweeps, detailed GCS/merger taxonomy, random-scene per-row details, dynamic obstacle rebuild, parallel scaling, soundness audit suite, mechanism diagnostics, and implementation roadmap tables.
- Weak-dependency grouped subtractive route: count-first collision concentration, weak subtractive update versus weak full rebuild, and query-overlay path quality. Keep this appendix-only until the route exposes explicit query/audit semantics compatible with the main-text tables.

Gate:
- Every JSON consumed by the paper must expose scene/robot context, parameters, result rows, audit metrics, and failure semantics.
- No main-text table may contain structural empty cells caused by mixing unrelated metrics. If two rows require different metrics, they belong in prose or appendix diagnostics.

## Stage 5: Execution Strategy

Actions:
- Run smoke versions first to validate scripts and schemas.
- Run full versions for the available current scripts.
- Keep long-running or unavailable baselines explicitly labeled rather than implying unexecuted results.
- For the weak LIE-only subtractive route, first run `split`, `sample`, and `sample_keep_colliding` smoke artifacts, then run the matched weak rebuild/update comparison before adding any path-quality row.

Default environment:
- Working directory: `cpp/SBF`
- Build directory: `SBF_BUILD_DIR=/home/tian/桌面/box_aabb/cpp/SBF/build_perf`
- Python: base Conda Python 3.13 ABI

Gate:
- Experiments that fail must produce a recorded failure row or an execution note.
- Weak-route path-quality rows must be labeled `weak conservative audit` unless an exact checker is actually used.

## Stage 6: Table Generation And Paper Integration

Goal: integrate experimental results into the TRO manuscript as academic evidence, not a run log.

Actions:
- Keep `experiments/tro2026_generate_tables.py` as the single table generator.
- Generate main-text files only for: `tab_tro_main_evidence_validation.tex`, `tab_tro_lect_reuse.tex`, `tab_tro_main_shelf_benchmark.tex`, `tab_tro_main_generalization.tex`, `text_tro_safety_fallback.tex`, and `tro_macros.tex`.
- Generate `fig_tro_query_amortization.pdf` from `experiments/tro2026_main_02_query_amortization.py` and cite the figure directly rather than duplicating it as a table.
- Retain appendix diagnostics under `--mode appendix`, but keep them out of the main evidence chain unless the text needs a specific diagnostic.

Artifacts:
- `experiments/tro2026_generate_tables.py`
- `doc/paper/tro_rewrite_2026/generated/*.tex`

Gate:
- No GCS row is counted as successful unless strict audit passed.
- Table I must avoid duplicating the boundary/evidence status in both the profile name and a separate evidence column.
- Table VI-style mixed systems summaries with empty cells are retired from the main text.

## Stage 7: Verification

Actions:
- Build/test the SBF extension and unit test target.
- Run Python experiment scripts used by the paper.
- Compile the new manuscript with XeLaTeX.
- Check undefined references/citations and user-specific float grouping if legacy labels survive.

Gate:
- Report any remaining failures or incomplete long-running experiments explicitly.

## Stage 8: TRO Major-Revision Evidence Hardening

Date added: 2026-05-18

Goal: make the claim boundary, baseline fairness, and audit credibility hard
enough for a TRO major revision. Textual scoping is not a substitute for these
experiments; if any item is not completed, the manuscript must keep the claim as
a narrow final-audited design-point result.

P0 experiments and diagnostics:

1. Shared-seed protocol audit and seed-count sensitivity.
	 - Verify that RBF, IRIS-NP+GCS, PRM, RRTConnect, and BIT* all consume the same
		 workload-level seed set: manually selected Marcucci anchors for Shelf+IIWA
		 and fixed random scene/query seeds for the random-scene study.
	 - Add a seed-count or seed-set sensitivity sweep if time allows, so the paper
		 can separate statistical robustness from full-pipeline runtime attribution.
	 - Gate: every IRIS comparison in the abstract/introduction/conclusion states
		 the common seed protocol and remains explicitly pipeline-level unless a
		 separate primitive-isolation experiment is added.

2. Audit credibility sweep and failure taxonomy.
	 - Re-audit reported paths at multiple joint-space segment steps around the
		 current 0.01 value.
	 - Classify the current five final-audit failures by likely source: repair,
		 smoothing, external composition, narrow clearance, insufficient forest
		 connectivity, or data/serialization issue.
	 - Gate: the paper can explain whether the final-audit failures are rare
		 implementation artifacts, systematic geometry failures, or audit-resolution
		 sensitivity.

3. Planner-level ablations.
	 - Disable unexplored-volume sampling, frontier cooling, component connectors,
		 post-corridor smoothing/repair, and LECT independently.
	 - Gate: the paper can state which layer contributes the measured time/path
		 trade-off instead of attributing the full result to the envelope primitive.

4. Reusable versus single-query presentation.
	 - Keep RBF/PRM/IRIS-NP+GCS in a reusable multi-query group and RRTConnect/BIT*
		 in a single-query reference group.
	 - Gate: no table or caption ranks all methods as one undifferentiated
		 superiority comparison.

P1 evidence if time allows:

- Add one more repeated-query manipulation workload that is task-level rather
	than synthetic random obstacles.
- Increase the random-scene seed count or add confidence intervals/Wilson
	intervals for success-rate rows.

Manuscript gates before submission:

- Claim-to-evidence audit: every strong sentence in the abstract, introduction,
	and conclusion maps to either a theorem with matching assumptions or a matching
	experiment.
- Baseline-fairness audit: every baseline comparison states the shared seed
	protocol and is labeled pipeline-level unless a primitive-isolation experiment
	is added.
- Semantics audit: every table/caption distinguishes candidate, corridor-certified,
	and final-audited success where those notions appear.
- Reviewer stress test: a two-paragraph skim must answer what is novel, what is
	guaranteed, and why the comparison is fair under its stated boundary.