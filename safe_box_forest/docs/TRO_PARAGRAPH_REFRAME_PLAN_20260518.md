# TRO Paragraph-Level Reframe Plan

Date: 2026-05-18
Target manuscript: `doc/paper/tro_rewrite_2026/sbf_tro_2026.tex`

## 0. New Paper Thesis

The paper should read as a practical multi-query planning paper, not as a certification-first paper. The central claim is:

> SafeBoxForest is a fast multi-query box-region planner for manipulators. It grows reusable volumetric C-space regions using a robot-to-envelope geometric pipeline, queries the resulting region graph efficiently, and obtains strong build/query/path-quality trade-offs on repeated-query manipulation workloads.

Certification should remain as an available mode and reporting boundary, but it should not be the lead contribution. LECT should be presented as a runtime/cache optimization, not as the conceptual center of the paper.

## 1. Global Terminology Rules

1. Replace headline uses of "conservative repeated-query planning framework" with "multi-query box-region planning framework" or "volumetric multi-query planner".
2. Use "validated", "audited", and "collision-filtered" in experiments and reporting. Use "certified" only for the optional conservative-envelope mode or for statements that explicitly depend on strict interval evidence.
3. Avoid making LECT the subject of topic sentences in the abstract, introduction, and conclusion. LECT should appear as an acceleration/cache layer.
4. Use RRT/PRM-region vocabulary in the method section: seed, frontier, grow, connect, graph, corridor, local refinement, repeated-query reuse.
5. Keep path quality and charged time together in the empirical claims. The core evidence is the efficiency/quality frontier, not the cache hit rate alone.
6. When discussing CritSample or SupportHull variants, describe them as practical envelope sources/filters with final audit, not as unconditional certificates.

## 2. Title and Abstract

### Title

Current issue: the title leads with "Lifelong Envelope Caching", which makes LECT look central.

Revision target:
- Lead with fast multi-query box-region planning.
- Keep envelope caching as a mechanism only if needed.

Candidate title:
- `SafeBoxForest: Fast Box-Region Planning for Multi-Query Manipulation`

Optional longer title:
- `SafeBoxForest: Fast Box-Region Covers for Multi-Query Manipulator Planning`

### Abstract Paragraph 1

Current issue: starts from reusable kinematic structure and immediately says "conservative".

Revision target:
- Start from repeated-query planning cost and the limitation of zero-volume reusable graphs.
- Introduce SBF as a volumetric box-region planner.
- Mention endpoint/link envelopes as the geometric engine.
- Mention certification as an option, not the first descriptor.

Suggested paragraph logic:
1. Repeated-query manipulation needs low build/query latency and acceptable path quality.
2. Roadmaps reuse zero-volume graphs, while region methods can be expensive to rebuild.
3. SBF grows C-space box regions using fast robot-to-envelope maps.
4. LECT caches envelope evidence when it is worth reusing.
5. Queries run on a connected box graph and may use local refinement.

### Abstract Paragraph 2

Current issue: conclusion emphasizes conservative reusable-planning and cross-scene reuse.

Revision target:
- Emphasize lower charged time and competitive audited path length.
- Say additional studies explain envelope trade-offs, cache optimization, and dynamic updates.
- End with planning-system claim, not certification claim.

## 3. Introduction

### Intro P1: Problem Setup

Current issue: asks what conservative evidence can be built once.

Revision target:
- Ask how to build reusable volumetric planning structure that supports fast repeated queries.
- Keep fixed robot/changing scene motivation.
- Mention efficiency and quality together.

### Intro P2: Baseline Landscape

Current issue: contrasts sampling zero-volume graphs and certified regions.

Revision target:
- Keep comparison, but phrase it around planning throughput.
- PRM/RRT: cheap, general, but zero-volume and repeated local validation.
- IRIS/GCS: region-level planning, often high-quality, but scene-local construction is expensive.
- SBF aims for a middle ground: lightweight volumetric regions.

### Intro P3: Key Design Move

Current issue: stores kinematic evidence at robot level.

Revision target:
- Say the key design move is a fast robot-to-envelope map that lets a planner grow boxes cheaply.
- LECT can memoize the map, but planning still works without making LECT the main idea.

### Intro P4: Method Overview

Current issue: LECT gets a full central paragraph.

Revision target:
- Combine envelope map, RRT-like box growth, connected graph query, and optional cache in one method overview.
- The topic sentence should be SBF, not LECT.

### Intro P5: Evaluation Axes

Current issue: includes certificate-update cost as a main axis.

Revision target:
- Main axes: build time, query time, audited success, path length, amortization with query count, and scene/robot variation.
- Dynamic update can be a secondary maintenance study.

### Contribution List

Rewrite to three contributions:

1. Robot-to-envelope geometric pipeline:
   - rounded-link/seed reduction;
   - endpoint envelopes;
   - link envelope representations;
   - optional conservative mode and practical tighter filters.

2. Multi-query box-region planning algorithm:
   - RRT-like frontier growth of validated C-space boxes;
   - connected box graph/corridor query;
   - local refinement and repeated-query reuse;
   - LECT as an acceleration/caching layer.

3. Efficiency/quality evaluation:
   - Shelf+IIWA and IIWA/UR5/Panda planning results;
   - path-quality/charged-time curves;
   - amortization, cache support, and dynamic updates as secondary evidence.

Remove LECT as a standalone main contribution.

### Evidence-Chain Paragraph

Current issue: section chain centers LECT.

Revision target:
- `Sec. III builds the robot-to-envelope map; Sec. IV turns it into a box-region planner; Sec. V describes cache/runtime optimizations; Sec. VI evaluates planning efficiency and quality.`
- If section order remains unchanged, phrase it as: envelope map -> cache optimization -> box-region planner -> experiments, but still state the planner/evaluation as the main evidence.

## 4. Related Work

### Convex Region Planning and GCS

Keep but shift contrast:
- Region methods optimize over strong region representations, often with expensive scene-coupled construction.
- SBF uses simpler axis-aligned boxes to reduce build cost and support repeated-query throughput.
- Do not frame the contrast primarily as certificate strength.

### Interval/Envelope Methods

Current issue: reads like certification lineage.

Revision target:
- Present interval/envelope methods as a fast geometry abstraction for planning.
- State that strict interval variants can provide conservative validation, while sampled/tighter filters are evaluated under final audit.

### Multi-Query Reuse

Strengthen this subsection:
- PRM and LazyPRM reuse connectivity but remain zero-volume.
- SBF reuses/grows regions that can answer repeated local queries quickly.
- Put LECT here as a cache optimization for region construction, not a new planning paradigm by itself.

### Sampling-Based Planning

Make the analogy explicit:
- SBF is closest in spirit to RRT/PRM expansion, but it grows boxes instead of points/edges.
- The paper should not claim asymptotic optimality or completeness; it claims practical efficiency on repeated-query workloads.

## 5. Theory Section: Robot-to-Envelope Pipeline

### Section Intro

Current issue: "core theoretical object" is a conservative map.

Revision target:
- "This section gives the geometric primitive used by the planner."
- The primitive maps a joint box to link envelopes used for fast collision filtering and optional conservative validation.

### Geometric Foundation

Keep structure, but adjust topic sentences:
1. Rounded-link/seed reduction is the first main theoretical piece.
2. Endpoint generator reduction is the second main piece.
3. The practical result is a robot-to-envelope map for box-region planning.

Potential theorem/proposition:
- If the endpoint envelopes contain endpoint trajectories, then the radius-lifted convex hull contains the capsule sweep.
- This is enough theory for the paper; avoid overselling broader certification.

### Endpoint Interval Envelopes

Rewrite source descriptions:
- IFK: strict/conservative option, reliable but loose.
- CritSample: practical tighter source, used for performance-oriented variants with final audit.
- Analytical: offline reference/tighter bounds.

Important: do not let CritSample appear as a strict certificate unless an external certificate is added.

### Link Envelope Representations

Frame AABB/KDOP/SupportHull as planning-efficiency trade-offs:
- AABB: fastest coarse filter.
- KDOP: directional mid-phase.
- SupportHull: tighter but more expensive bottom phase.
- SBF-SH is a cascade tuned for planning throughput.

## 6. LECT Section: Optimization Layer

### Section Title

Possible rename:
- `Envelope Cache and Runtime Optimizations`
- or keep `Lifelong Envelope Cache Tree` but open with "LECT is an optimization layer".

### Section Intro

Current issue: LECT is written as the central data structure.

Revision target:
- "The planner can run by materializing envelopes online; LECT improves repeated-build workloads by memoizing expensive envelope records."

### Design Principles

Shorten to:
- node key;
- cached evidence;
- scene-independent vs scene-specific split;
- when lookup is worth it.

### Split Policy

Keep only if it directly affects planning runtime:
- one paragraph on best-tighten split as a heuristic for reducing envelope slack;
- one equation can remain;
- remove or shorten claims about suppressing box proliferation unless supported experimentally.

### Parent Refinement / Storage / Symmetry

Move to compact implementation paragraphs or appendix:
- parent refinement: one paragraph;
- storage/persistence: one paragraph;
- symmetry compression: appendix or one sentence unless measured.

### Warm Replay Claims

Tone down:
- "supporting optimization study";
- "same-scene replay gives upper reference";
- "cross-scene warm prewarm gives modest but measurable acceleration in the current artifact."

## 7. Safe Box Forest Construction: Planning Algorithm

### Section Intro

Current issue: starts from scene-independent certificate structure.

Revision target:
- Start from planner behavior: build a region graph for repeated queries.
- The planner is RRT-like: sample, locate frontier, materialize a free box, connect, repeat.

### Construction Objective

Rewrite objective around planning:
- cover useful query-relevant free space;
- keep graph connected enough for corridor queries;
- control build budget and path quality.

### Connected-Box Rule

Keep, but phrase as graph-connectivity and corridor-search rule.
- Avoid making it sound like the main certificate claim.

### FindFreeBox

Reframe:
- FFB is the local region oracle called by the grower.
- It descends a split hierarchy and returns a usable box around a seed.
- LECT is one implementation of the oracle.

### Forest Growth

Strengthen RRT analogy:
- samples propose directions;
- nearest frontier box chooses parent;
- boundary projection plays the role of extension;
- accepted boxes form a region-valued tree/forest.

### Consolidation

Frame as graph simplification and query-speed improvement.
- Keep conservative merge statements as optional validity contract.
- Do not overemphasize certificate-preserving proof if the paper is not certification-first.

### Corridor Query

Frame as the product of the planner:
- graph search returns a box sequence;
- local refinement improves path length;
- final audit is used for all reported paths.

### Theorem

Options:
1. Keep the theorem but relabel as `Corridor validation statement` and make it a reporting/validation property.
2. Move it after the algorithm, not before the empirical discussion.
3. Avoid referencing it as a main contribution.

## 8. Experiments

### Experiment Preamble

Current issue: starts with evidence profiles and LECT reuse.

Revision target:
- Start with planning-performance questions:
  1. How fast does SBF build/query relative to reusable baselines?
  2. What path-quality gap does it pay?
  3. How does performance vary across robots/scenes?
  4. How much do envelope/cache components contribute?

### Main Evidence Order

Preferred order:
1. Shelf+IIWA planning trade-off.
2. Cross-robot random-scene planning trade-off.
3. Amortization/reuse and LECT optimization.
4. Envelope representation/supporting microbenchmarks.
5. Dynamic updates as secondary maintenance result.

If moving full sections is too disruptive in the first pass, keep section order but rewrite the preamble and subsection introductions so the planning trade-off figures are called the primary evidence.

### Endpoint/Link Microbenchmarks

Demote:
- "supporting mechanism study";
- explain why SBF-SH is chosen;
- move detailed width-wise tables to appendix if page budget tight.

### LECT Reuse

Demote:
- report as optimization not main evidence;
- explicitly say current cross-scene gain is modest;
- same-scene replay is an upper-reference optimization result.

### Shelf+IIWA and Random Scenes

Promote:
- these are the main tables/figures;
- highlight charged build, query time, audited path length, and success;
- avoid over-relying on a single selected row; point to full curves.

### Dynamic Updates

Keep short:
- secondary maintenance property;
- not a main contribution unless space permits.

## 9. Discussion and Conclusion

### Discussion Opening

Rewrite around planning:
- "The results suggest that a simple volumetric box-region representation can be a strong practical point in the repeated-query planning design space."

### Reuse Behavior

Tone down cache:
- LECT helps when expensive envelope records are revisited;
- current cross-scene acceleration is measurable but not decisive;
- planner efficiency does not depend entirely on LECT.

### Theory and Scope Paragraph

Add honest framing:
- This is an efficiency-oriented planning system, not a paper claiming new completeness/optimality theory.
- Its strengths are build/query throughput and competitive path quality.

### Conclusion

Final claim:
- SBF provides a fast multi-query box-region planning pipeline with robot-to-envelope geometry and optional cache acceleration.
- The experiments show favorable planning efficiency/quality trade-offs on the studied workloads.
- Future work: post-corridor optimization, broader scenes, sensitivity, cache transfer.

## 10. First Editing Pass Checklist

1. Edit title and abstract.
2. Rewrite introduction through contribution list.
3. Change framework figure caption to planner-first wording.
4. Rewrite Link Interval Envelopes opening paragraphs.
5. Rewrite LECT opening to optimization-layer wording.
6. Rewrite Safe Box Forest opening to RRT-like multi-query planner wording.
7. Rewrite experiment preamble to planning-first wording.
8. Rewrite conclusion opening and final empirical claim.
9. Run LaTeX diagnostics and fix introduced syntax issues.

## 11. Acceptance Criteria For This Reframe

A reviewer skimming only the title, abstract, contribution list, section openings, and conclusion should conclude:

1. The paper is about fast multi-query manipulator planning.
2. The main algorithm grows and queries volumetric C-space box regions.
3. The geometric theory is the robot-to-endpoint-to-link envelope pipeline.
4. LECT is a useful optimization, not the whole contribution.
5. Certification exists as an option/reporting boundary, but the paper does not oversell itself as a certification-first theoretical contribution.

## 12. Progress Log

### 2026-05-18: First Paragraph-Level Pass

- Reframed the title, abstract, introduction, contribution list, framework caption, section openings, discussion, and conclusion around multi-query box-region planning.
- Demoted LECT from a central contribution to an optional cache/runtime optimization.
- Recast the theory opening around the robot-to-envelope geometric primitive.
- Recast the Safe Box Forest construction opening as an RRT-like region-valued planning algorithm.

### 2026-05-18: Second Paragraph-Level Pass

- Reordered the experiment narrative so `Shelf+IIWA Efficiency and Path Quality` and `Cross-Robot Quality and Efficiency` appear before the mechanism studies.
- Moved endpoint-source, link-envelope, and LECT reuse material into one supporting subsection: `Supporting Envelope and Cache Studies`.
- Removed the duplicate mechanism figure block from the experiment preamble so the first experiment figures are the primary planning trade-off figures.
- Updated the manuscript figure captions to mark Shelf+IIWA and cross-robot figures as primary planning trade-offs.
- Updated generated table captions and `experiments/tro2026_generate_tables.py` so regenerated tables preserve the primary/supporting evidence hierarchy.

### 2026-05-18: Third Full-Paper Optimization Pass

- Kept `SafeBoxForest (SBF)` as the paper/project method name, but reframed the title, abstract, introduction, contribution list, related work, and construction section around `rapidly exploring box-region forests`.
- Added the adaptive cell-decomposition interpretation in the abstract, introduction, related work, construction objective, FindFreeBox description, discussion, and conclusion.
- Positioned the adaptive-cell link as an inherited planning idea extended to high-dimensional manipulator `C`-space through the robot-to-envelope oracle, not as a new completeness or optimality claim.
- Tightened the discussion and conclusion so SBF is presented as a practical RRT-like adaptive cell planner for repeated-query manipulation.

### 2026-05-18: Fourth Envelope-First/RapidBoxForest Pass

- Renamed the paper-facing method to `RapidBoxForest (RBF)` while keeping repository/internal artifact names unchanged.
- Reframed the title, abstract, introduction, contributions, related work, experiments, discussion, and conclusion around fast `C`-space free-region construction as an alternative to expensive IRIS-style convex inflation.
- Promoted endpoint interval envelope to link interval envelope construction as the main theory thread and added a proposition showing that endpoint enclosures induce rounded-link sweep enclosures.
- Repositioned LECT as an endpoint/link-envelope materialization optimization and the grower as the RRT-inspired mechanism that expands the RBF box forest.
- Updated generated table labels/captions and the table generator display strings from `SBF-SH` to `RBF-SH`; restored numeric table rows after a generator run with incomplete local artifacts.
- Verified the manuscript with two XeLaTeX passes; the PDF compiles to 17 pages with no unresolved references or citations.

### 2026-05-18: Final Name-Scrub and TRO-Style Polish

- Removed remaining visible `SafeBoxForest`, standalone `SBF`, `SBF-SH`, and `\sbf` occurrences from the compiled manuscript and generated table `.tex` files.
- Removed the old repository URL from the abstract and replaced it with an artifact statement so the paper text no longer displays the previous project name.
- Tightened abstract, contribution, experiment, discussion, and conclusion wording for a more formal TRO style, including replacing informal phrasing such as path-quality ``cost'' and ``supporting studies''.
- Updated paper-facing generator strings so future table generation does not reintroduce old labels, while preserving lowercase internal `sbf` JSON keys and legacy artifact input mappings.
- Re-ran XeLaTeX twice; the PDF compiles to 17 pages with no unresolved references or citations.

### 2026-05-18: Reviewer-Concern Alignment and Final-Audit Calibration

- Recalibrated the title, abstract, contribution list, theory opening, theorem reporting protocol, experiments, discussion, and conclusion around a final-audited multi-query planning claim rather than a certification-first free-region claim.
- Made the strict corridor theorem explicitly conditional on conservative endpoint/link envelopes and paths that remain inside the validated box union; the main curves now report final audited planning success.
- Added the IRIS-NP+GCS seed caveat used in that draft pass; this was later corrected to a shared workload-level seed protocol with seed-count sensitivity as future validation.
- Downgraded LECT to a selective runtime/cache layer and the dynamic-update result to a capability study, with empirical scope limited to the reported Shelf+IIWA and five-seed random-scene protocols.
- Updated generated table captions, generator strings, endpoint/source mechanism captions, and dynamic-update wording to match the final-audited evidence hierarchy.
- Created cleaned high-resolution figure assets with `RBF-SH` legends for the included anytime trade-off figures because the archived plot JSON artifacts are unavailable locally and the original PDFs still contained old embedded legend text.
- Re-ran XeLaTeX twice; the PDF compiles to 18 pages with no unresolved references, citations, or image errors. A final `pdftotext` audit found `RapidBoxForest`/`RBF-SH` and no visible `SafeBoxForest`, standalone `SBF`, `SBF-SH`, `strict audit`, `strict-audited`, `certified free-region`, `free-region alternative`, or `seed-independent lower bound`.

### 2026-05-18: Novelty Calibration and Grower Reproducibility Pass

- Reframed the contribution list so the main claim is a reproducible final-audited multi-query planning system, supported by endpoint-to-link envelopes, a fully specified box-forest planner, and scoped pipeline-level evaluation.
- Softened the abstract, introduction, experiment opening, discussion, and conclusion so comparative statements are explicitly pipeline-level under the reported seed protocol and not primitive-isolation claims.
- Expanded the Forest Growth subsection with source-backed grower details: connector/query/unexplored/intertree target sampling, face seed construction, squared target-distance face ranking, 128-candidate retention, cooled LECT leaves, frontier-seed caching, and the `BoxesConnected` acceptance rule.
- Renamed the random-scene experiment subsection from a broad cross-robot claim to a balanced random-scene pipeline study, and added runtime-gain attribution limits plus future strategy ablations for unexplored-volume sampling, cooling, connector targets, and post-corridor smoothing.
- Re-ran diagnostics and validation after the pass: Python generator has no errors, LaTeX diagnostics remain ChkTeX-style warnings only, XeLaTeX passed twice, the PDF remains 18 pages, and the strict `pdftotext` audit found the expected `RapidBoxForest`, `RBF-SH`, `pipeline-level`, `primitive-isolation`, `strategy ablations`, and `failure_cooling_threshold` terms with no old paper-facing names or black-box grower placeholders.

### 2026-05-18: Design-Point Claim Scope and Safety-Semantics Pass

- Rewrote the abstract, introduction, contribution list, experiment preamble, Shelf+IIWA/random-scene interpretations, discussion, and conclusion around a final-audited low-build/query design point rather than a comparative superiority claim.
- Made the time/path-quality trade-off explicit: RBF-SH has very low reusable build/query cost, while audited paths are close to but not uniformly shorter than IRIS-NP+GCS, PRM, or BIT* rows.
- Clarified validation semantics: main curves are final-audited system curves, strict corridor validity applies only inside conservatively validated box unions, and the 663/668 audit count means the remaining five paths are failures without a current repair/smoothing/external-composition/narrow-clearance/forest-connectivity decomposition.
- Further demoted LECT to a selective runtime optimization: warm prewarm gives only 1.06--1.14x target-build gains, while 2.23x replay is a same-scene upper reference rather than cross-scene transfer evidence.
- Updated included table captions and generator caption strings to use `design-point` wording and not `primary evidence` wording; did not run the full table generator.
- Re-ran validation: `tro2026_generate_tables.py` has no errors, XeLaTeX passed twice, PDF remains 18 pages, and a strict `pdftotext` audit found the expected design-point/failure-accounting terms with no old paper-facing names or old overclaim phrases.

### 2026-05-18: TRO Major-Revision Stress-Test Alignment

- Lowered the prominence of the conditional corridor guarantee in the abstract: the planning curves are now described first as empirical final-audited system results under a fixed 0.01 joint-space audit step, with the strict interval mode framed as optional and conditional.
- Added a planning-systems positioning paragraph in the introduction: the paper is a planning-systems contribution enabled by the endpoint-to-link geometry primitive, not a pure theory paper or a deployment-heavy robotics study.
- Compressed the LECT section from implementation-heavy cache/storage/symmetry detail into a secondary optimization interface: cache key and split policy, runtime validation boundary, and warm-build semantics.
- Rewrote the experiment reporting protocol as a three-success-notion summary covering candidate success, corridor-certified success, and final-audited success; final audit is explicitly a uniform empirical check rather than a continuous certificate.
- Clarified baseline classes in the experiment setup: RBF, PRM, and IRIS-NP+GCS are reusable multi-query pipelines, while RRTConnect and BIT* are no-build single-query references.
- Added major-revision evidence gates to `TRO_REWRITE_EXECUTION_PLAN_20260505.md` and `TRO_EXPERIMENT_COMPRESSION_AND_SCRIPT_PLAN.md`: seed-protocol audit or seed-count sensitivity, audit-resolution sweep plus five-failure taxonomy, planner-level ablations, and a stronger repeated-query workload if time permits.

### 2026-05-18: Unified-Seed and Corridor-Coverage Correction

- Corrected the IRIS/other-baseline fairness narrative: all reported planners use common workload-level seed sets; Shelf+IIWA uses manually selected Marcucci reference anchors, while random scenes use fixed random scene/query seeds.
- Replaced the older seed-mismatch limitation with a full-pipeline attribution limitation and a seed-count/seed-set sensitivity follow-up.
- Made the main table semantics explicit: Shelf+IIWA and random-scene rows are final-audited only and do not tabulate corridor-certified coverage for the optional conservative mode.
- Added numeric reported grower settings to the method text: target-source probabilities, build budget, FFB depth cap, worker count, component-connector candidate limit, staged radius, post-connect budget, quality floor, and hard-frontier cooling horizon.
