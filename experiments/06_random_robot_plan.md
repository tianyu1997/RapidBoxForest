# Experiment 6: Random Scenes And Robots

## 目的

复现旧 TABLE II 和 fig4 风格，评估 SBF 在 IIWA、UR5、Panda 与 Easy/Medium/Hard 难度下的泛化表现。

## 实验设置

1. Robots：IIWA、UR5、Panda。
2. Difficulty：Easy、Medium、Hard。
3. Methods：SBF-SH baseline、IRIS-NP+GCS、PRM、RRTConnect、BIT*。
4. Seeds：每个 robot x difficulty 固定 scene seeds 和 query seeds。
5. SBF 默认 warm + AAFK + SupportHull + 8 threads + AAFKVolumeMin。

## 如何进行

1. 使用 balanced random scene sampler 生成候选场景。
2. 先做 balanced probe，排除 trivially free 或 completely blocked 的场景，并记录替换原因。
3. 冻结 scene manifest：robot、difficulty、seed、obstacle list、start、goal、query id。
4. 对每个 method x robot x difficulty x seed 运行完整 anytime/checkpoint 输出。
5. 按旧 TABLE II 输出 readable slice，同时保留 fig4 风格完整 curve。

## 统计口径

1. 主表列：Scenario、Build(s)、Query(s)、Path。
2. RRTConnect/BIT* 无 build 时只报 Query/Path。
3. 每个 scenario 报 mean/median/p95；主表可选 representative point，但附录/CSV 保存全量 seeds。
4. Success-only path/query 和 all-run failure rate 同时报告。
5. 跨机器人不合并 path length。

## 预期结果

1. SBF 在不同机器人/难度下保持低 build 和低 query。
2. 难度升高时所有方法成本上升，SBF 增长应较平缓。
3. IRIS-NP+GCS 路径质量有竞争力但成本最高。
4. PRM 依赖 build budget；RRTConnect 快但稳定性弱；BIT* 依赖 timeout 改善路径。

## 初始脚本

`exp06_random_robot/run_random_robot.py` 调度 SBF、RRTConnect、PRM/BIT* 和 IRIS-NP+GCS random-scene runner，统一 robots/difficulties/seeds/out-json 参数。
