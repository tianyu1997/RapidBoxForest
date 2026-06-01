# RapidBoxForest

This repository root is the RapidBoxForest workspace-style monorepo for three first-party modules:

- `link_interval_envelope/`: envelope and incremental-FK primitives.
- `lect_database/`: database, online cache, and the SBF adapter layer.
- `safe_box_forest/`: planner and query pipeline built on `LECTDatabase`.

Dependency direction is intentional and one-way:

`safe_box_forest -> lect_database -> link_interval_envelope`

The root does not own implementation code for any one module. It only provides a
single top-level CMake entry point that can configure the modules together in
dependency order.

## Layout

```text
.
|-- CMakeLists.txt
|-- .gitignore
|-- docs/
|-- link_interval_envelope/
|-- lect_database/
`-- safe_box_forest/
```

## Root Build

```bash
cmake -S . -B build \
  -DRBF_BUILD_ENVELOPE=ON \
  -DRBF_BUILD_LECT_DATABASE=ON \
  -DRBF_BUILD_SBF=ON \
  -DRBF_BUILD_TESTS=ON \
  -DRBF_WITH_PYTHON=OFF

cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Each module still keeps its own package-facing CMake entry and documentation.


当前AAFKVolumnMin split policy在确定最佳分割维度时，时只沿着左子树的 root interval 进行采样，而不考虑右子树的 root interval。这可能会导致在某些情况下，分割维度的选择过于局限，从而影响最终的分割效果。为了解决这个问题，我认为应该在特点深度确定分割维度时，在该深度采样多个节点，综合各个节点的信息来决定最佳分割维度。
这样做的好处是可以更全面地考虑整个树的结构和数据分布，从而选择出更合适的分割维度，提高分割的效果和效率。当然，这也会增加一些计算开销，但如果能够显著提升分割质量，可能是值得的。建议在实现时，可以先进行一些实验来验证这种方法的有效性，并根据实验结果来调整采样策略和参数设置。
当前还有一个很困扰我的问题是：通过birrt connector 产生的segment bridge为什么不能通过chain pave 完全用box进行完全覆盖？理论上来讲，当max depth足够深的话，box应该是能够完全覆盖segment bridge的，即使是非常狭窄的环境。请你设计一个实验来验证这个问题，并且分析可能的原因。分析chain pave的实现细节，看看是否存在某些边界条件或者数值误差导致了这个问题。同时，可以尝试在不同的环境和参数设置下运行这个实验，以观察是否存在特定的情况会导致这个问题的出现。