# Experiment 2: Link Envelope, S=1

## 目的

参考旧 TABLE IV，但统一固定 `S=1`，比较 AABB、SH 和 `AABB->SH chain` 的 volume、payload 和 eval time。

## 实验设置

1. 宽度集合：`0.02,0.05,0.10,0.20,0.30,0.50` rad。
2. Short-link split 固定为 `S=1`。旧 TABLE IV 使用 `S=4`，只能趋势对照，不能直接数值比较。
3. Endpoint source 默认 `CritSample`；追加 AAFK/IFK 组用于和 shelf baseline 对齐。
4. Envelope 碰撞检测选项：`AABB`、`SH`、`AABB->SH chain`。
5. 默认读取 Exp.1 生成的固定 box table。

## 如何进行

1. 固定 `S=1` 构造每个 link envelope。
2. 对每个 option x width x box 记录 envelope construction time、eval time、payload bytes 和 volume proxy。
3. 对 `AABB->SH chain` 记录每一级 early-out rate、实际访问 payload bytes 和平均 eval time。
4. 输出 TABLE IV 风格汇总和 width-wise 明细。

## 统计口径

1. `V (m^3)`：同一 short-link split 下的 record-wise envelope volume，需注明是否包含 inflation。
2. `Payload`：serialized compact payload bytes；`AABB->SH chain` 同时报累计 stage bytes 和平均实际访问 bytes。
3. `Eval (us)`：单次 envelope evaluation，不包含 endpoint source 计算和文件 IO。
4. `Early-out rate`：`AABB->SH chain` 在 AABB、SH 阶段终止比例。

## 预期结果

1. AABB 最快、payload 最小，但最松。
2. SH 最紧，但 payload 和 eval time 高于 AABB。
3. `AABB->SH chain` 预期在平均耗时上接近 AABB，并在困难 width 上保留 SH 的 tighter witness。
4. `S=1` 预期比 `S=4` 更省存储和构建时间，但 envelope 更粗。

## 初始脚本

`exp02_link_envelope_s1/run_link_envelope_s1.py` 复用旧 link pipeline，强制 S=1，并在 wrapper 中补上 `AABB->SH chain` 的 S=1 staged variant 解析。
