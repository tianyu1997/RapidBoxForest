# Experiment 1: Endpoint Envelope

## 目的

复现并扩展旧 TABLE III，比较不同 EndpointSource 在固定关节区间宽度下的 endpoint AABB 体积、计算耗时和相对参考 envelope 的 gap。该实验用于解释后续 planning 为什么采用 AAFK/IFK 类安全外包络，以及 CritSample 等经验源的成本和风险。

## 实验设置

1. 宽度集合：`0.02,0.05,0.10,0.20,0.30,0.50` rad。
2. Source：`IFK,AAFK,HIFK_3,HIFK_5,CritSample,Analytical,MC`。
3. 默认机器人：IIWA14；跨机器人扩展时 IIWA、UR5、Panda 分开报告。
4. 默认每个 width 采样 10000 个 joint boxes；smoke test 使用 1 到 5 个 boxes。
5. 所有 source 共享同一 box table、seed 和 joint limits。

## 如何进行

1. 生成 joint-box table，记录 robot、joint limits、width、seed、box id、intervals。
2. 对每个 `source x width x box` 计算 endpoint AABB。
3. 记录 volume、endpoint time、combo count、线程数、是否 cache reused。
4. 以 Analytical/CritSample/MC 的采样并集作为 gap 参考，计算每个 source 的最大 under-coverage gap。
5. 输出 TABLE III 风格汇总和 per-box 原始记录。

## 统计口径

1. `V (m^3)`：每个 box 的 endpoint AABB volume，按 width/source 求 mean，并保留 median/p95。
2. `Mean (us)`：单次 endpoint 计算耗时，不含 box table 生成和写文件。
3. `Max gap (m)`：source envelope 相对参考并集的最大未覆盖距离。
4. 失败和 timeout 不丢弃，单独计入 failure rate。

## 预期结果

1. IFK/AAFK 预期最快且安全，但宽 width 下 volume 较大。
2. HIFK_3/HIFK_5 预期更紧但更慢，HIFK_5 通常更紧。
3. CritSample 预期更紧、耗时中等，但不是认证外包络，可能出现非零 gap。
4. Analytical/MC 主要作为参考或 sanity check，成本最高。

## 初始脚本

`exp01_endpoint_envelope/run_endpoint_envelope.py` 复用旧 endpoint pipeline，并补充 AAFK alias、dry-run manifest 和 smoke 参数。
