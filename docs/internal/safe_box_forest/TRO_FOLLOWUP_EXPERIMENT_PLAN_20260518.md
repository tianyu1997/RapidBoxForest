# TRO Follow-Up Experiment Plan 2026-05-18

本文档只规划下一轮最关键的补证据实验，目标是把当前 RapidBoxForest (RBF) 稿件从 promising planning-system result 推到更接近 TRO major-revision 标准。原则是少而硬：优先补验证闭环、统计稳健性和系统归因，不展开过多细枝末节的展示实验。

## Priority 0: Validation Closure

目的：补上论文当前最容易被审稿人抓住的安全语义缺口。主表已经说明是 final-audited rows，但还没有告诉读者有多少结果同时满足 optional conservative corridor mode，也没有解释 5 个 final-audit failures 的来源。

实验内容：

1. 对现有 paper artifacts 做统一 validation accounting。
2. 对每条候选/成功路径统计 final-audited success、solved-but-audit-failed、repair/fallback 事件、corridor-certified 字段是否存在。
3. 如果 artifact 有 corridor-certified 或 inside-validated-union 字段，直接给出 coverage rate；如果没有，明确标为 missing instrumentation，而不是假装有数据。
4. 对 audit failures 做最小 taxonomy：repair/smoothing、bridge/external composition、corridor coverage missing、collision/audit failure、timeout/no path、unknown。
5. 只在该项完成后，主文才能把 corridor-certified coverage 写成实测结果；否则继续保持“主表不报告 corridor-certified coverage”。

主要输出：

- `outputs/paper/tro2026_followup_validation_closure.json`
- `outputs/paper/tro2026_followup_validation_closure.csv`
- `outputs/paper/tro2026_followup_validation_closure.md`
- 论文中一张 compact validation-closure 表，优先放 appendix；若结果很强，再在主文 validation paragraph 中引用。

代码入口：

- `experiments/tro2026_followup_01_validation_closure.py`

验收标准：

- 能复现 paper-wide final-audit pass/fail accounting。
- 能明确说明 corridor-certified coverage 是已测、未测，还是字段缺失。
- 5 个 final-audit failures 至少能被归入一组最小 taxonomy；不能分类的失败必须列为 unknown 并保留 artifact locator。

## Priority 0: Shared-Seed Robustness And Sample Size

目的：回应统计力度不足和 seed-count 弱的问题。当前稿件已经说明所有 planner 使用统一 workload-level seeds；下一步不再做 matched-seed IRIS，而是做 seed-protocol audit 和 seed-count/seed-set sensitivity。

实验内容：

1. 审计 Shelf+IIWA 和 random-scene artifacts 的 seed metadata，确认 RBF、IRIS-NP+GCS、PRM、RRTConnect、BIT* 使用同一 workload seed set。
2. random scenes 将每个 robot/difficulty 的 scene seeds 从 5 扩到 10 或 20。优先 10；只有时间充足时做 20。
3. 输出 Wilson interval 或 bootstrap interval，只对 final-audited success 和 paired time/path difference 做统计，不新增很多展示曲线。
4. Shelf+IIWA 保持 Marcucci reference anchors；如果要增强统计，只增加 repeated build/query seeds，不改 anchor policy。

主要输出：

- `outputs/paper/tro2026_followup_seed10_random_anytime.json`
- `outputs/paper/tro2026_followup_seed_protocol_audit.json`
- 一张 random-scene robustness 表：method、scenario group、n、audit SR + CI、build/query median + IQR、path median + IQR。

代码入口：

- 命令矩阵由 `experiments/tro2026_followup_02_priority_run_matrix.py` 生成。
- 实际重实验复用 `paper_15_random_anytime_tradeoff.py`、`paper_16_random_iris_np_gcs_anytime.py` 和 OMPL baseline runners。

验收标准：

- 主文/appendix 能说明当前 5-seed 结论在 10-seed setting 下是否保持。
- 如果结论变化，主文只能保留更窄的 5-seed claim。

## Priority 1: Minimal Planner Attribution Ablation

目的：补 system-level novelty 的因果归因。不要展开大规模网格，只做最能解释 RBF build/query design point 的少数 ablation。

实验内容：

在 Shelf+IIWA 上跑同一 task/anchor seeds，比较：

1. Full RBF-SH reported configuration。
2. No unexplored-volume sampling (`unexplored_sample_prob=0`)。
3. No component-connector targets (`component_connect_prob=0`)。
4. No post-audit repair (`repair_on_audit_failure=false`)。
5. Optional: no LECT/warm cache only当 artifact 已经支持，不为它单独重构 runner。

主要输出：

- `outputs/paper/tro2026_followup_grower_ablation_shelf.json`
- 一张 ablation 表：build time、query time、audit SR、repair count、path length、box count。

代码入口：

- 命令矩阵由 `experiments/tro2026_followup_02_priority_run_matrix.py` 生成。
- 实际运行复用 `paper_04_marcucci_combined.py` 或在其基础上加最小参数开关。

验收标准：

- 能回答 runtime gain 主要来自 envelope/box representation、frontier growth、connectors 还是 repair。
- 如果某个机制对结果不可或缺，正文要把 RBF 明确表述为 integrated system，而非单一 primitive 的效果。

## Priority 1: Audit-Resolution Sensitivity

目的：让 fixed 0.01 joint-space audit step 更可信。该实验在 validation closure 之后运行，不作为首要主结果。

实验内容：

1. 对代表性 saved paths 用 0.02、0.01、0.005 三档 audit step 复审。
2. 统计 status change count、audit overhead、changed path locator。
3. 如果 0.005 相对 0.01 有状态变化，论文应改用更细档或把 0.01 明确降级为 pilot setting。

主要输出：

- `outputs/paper/tro2026_followup_audit_resolution_sweep.json`
- 一张 audit sensitivity 表。

验收标准：

- 0.01 到 0.005 的 changed-decision count 为 0，或主文明确承认 audit-resolution sensitivity。

## Deferred Experiments

以下暂不作为下一轮核心任务，避免实验设计过散：

- 大规模 worker-count sweep：除非 reviewer 专门追问 parallel scaling。
- 多种 row-selection utility sweep：保留为 appendix script check，不作为主补证据。
- 大量新 robot/scene family：优先先把 random-scene seeds 做扎实。
- LECT cross-scene 大矩阵：当前稿件已把 LECT 降级为 secondary optimization，不应抢主线资源。

## Immediate Coding Tasks

1. 新增 `tro2026_followup_01_validation_closure.py`：从 artifacts 汇总 final audit、corridor coverage field availability、failure taxonomy。
2. 新增 `tro2026_followup_02_priority_run_matrix.py`：生成 P0/P1 的最小命令矩阵和 markdown checklist。
3. 暂不接入 `tro2026_generate_tables.py`，等 artifacts 真实存在后再加表格 writer，避免 placeholder 污染主文数值表。