# tools/parity — 与原版 IVP 的逐位对照

本工具同时检查 libivp 的三个构建域：

- **reference-development（默认配置）：**使用
  `IVP_STABLE_INTERNAL_STATE=OFF` 和 `IVP_QHULL_MODE=REENTRANT`，
  Release/Debug 必须 48/48 逐位一致。
- **mainline：**使用 `IVP_STABLE_INTERNAL_STATE=ON` 和可重入 Qhull 2020.2，必须恰好只有
  `mainline_expected_differences.txt` 登记的 20 个场景不同；多一个或少一个都失败。
  Debug 主线另外只允许 `mainline_debug_text_differences.txt` 登记的场景在调试文本上
  不同（JSONL 仍须逐位一致），同样多一个或少一个都失败。
- **edit-locality：**主线加 `IVP_EDIT_LOCALITY=ON`，必须恰好只有
  `edit_locality_expected_differences.txt` 登记的 39 个场景不同。

三个域都对照同一个原版引擎（默认 `../ivp`）。Release 对照
`-ffp-contract=off` 原版；Debug 对照不定义 `NDEBUG`、定义 `DEBUG` 的原版。
Qhull 已不再是两端的差异变量。

本目录的脚本用来构建原版、批量运行场景，并比较输出。

## 准备

原版引擎和 runner 都在 `https://github.com/doyaGu/ivp` 的 `master` 上；
对照引擎固定为 `2b4965e`，包含 Qhull 8.0.2 可重入迁移。需要克隆到与
本仓库同级的 `../ivp`：

```bash
git clone https://github.com/doyaGu/ivp.git ../ivp
```

放在别处时用 `IVP_REF_SRC` 指定路径。构建原版需要 C++17 编译器，比较脚本需要 python3。

## 一键检查

```bash
tools/parity/check.sh                      # release + debug
tools/parity/check.sh --release
tools/parity/check.sh --debug
tools/parity/check.sh --sweep "0.016666666666666666:900 0.0125:1200"   # 另加步长扫描（release）
```

退出码为 0 表示三个域都满足各自要求。结果写在
`build/parity/out/{release,debug}_{reference,mainline,locality}.txt`；步长扫描只比较
reference-development 和主线，写入 `build/parity/out/sweep_*/{reference,mainline}.txt`。

各项检查比较的内容：

| 检查 | 比较内容 |
|---|---|
| release | 三个构建域各自的 JSONL 帧（pos、quat、lin_vel、ang_vel 逐位）、stdout 中的非 JSON 行、stderr、退出码 |
| debug | 三个构建域各自的 JSONL 帧、调试输出文本、stderr 中 `ASSERTION FAILURE: <expr>` 报告序列、退出码 |
| sweep | 两个构建域中每个场景在给定步长和步数下的 JSONL 帧；主线差异必须是登记集合的子集，因为不同步数可能尚未到达首个分叉 |

## 环境变量

| 变量 | 默认值 | 作用 |
|---|---|---|
| `IVP_REF_SRC` | `../ivp` | 原版源码 |
| `IVP_PARITY_DIR` | `build/parity` | 构建和输出目录 |
| `IVP_JOBS` | 8 | 并行数 |
| `SCENARIOS` | 全部 | 以空格分隔的场景子集 |

## 单独使用的脚本

| 脚本 | 用途 |
|---|---|
| `build_reference.sh [release\|debug\|fma]` | 构建原版库（CMake）和 runner，输出 `build/parity/ref_<flavor>/runner/legacy_runner` |
| `run_scenarios.sh <ref\|lib> <binary> <out_dir> [dt steps]` | 运行全部场景，写出 `<scenario>.{jsonl,out,err,rc}` |
| `compare.sh [--text] [--stderr] [--asserts] [--expect-differences file\|--allow-differences file] <ref_dir> <lib_dir>` | 逐场景比较；可要求差异集合等于或包含于清单 |
| `cmp_jsonl.py <ref.jsonl> <lib.jsonl>` | 逐位比较两个 JSONL，给出第一个不同的帧 |
| `build_override_runner.sh <release\|debug> <out_dir> <file.cxx>...` | 用插桩后的原版源文件替换库中的同名对象，链接新的 runner |

两种场景程序的运行方式：

- `run_scenarios.sh ref` 运行原版 runner 时忽略 SIGINT，所以失败的原版断言只打印、不终止。
- `run_scenarios.sh lib` 运行 libivp 的 `test_scenarios` 时加 `--assert-continue`，失败的断言以原版的格式打印并继续。

## 定位一个差异

1. **找第一个不同的帧：**用 `compare.sh` 或 `cmp_jsonl.py`。
2. **给原版插桩：**复制相关的原版 `.cxx`，加 `fprintf(stderr, "%a ...", value)` 探针，再运行
   `tools/parity/build_override_runner.sh release /tmp/ovr copy.cxx`。
   覆盖文件按原版库对象的编译选项编译，所以其余部分仍然逐位相同。
3. **给 libivp 插桩：**在对应位置加相同的探针，运行
   `build/parity/lib_reference_release/test_scenarios --scenario X --steps N --dt D --output x.jsonl`
   （或用 `lib_mainline_release` 检查主线路径）。
4. **逐值比较：**比较两边的 stderr，找出第一个不同的值。用完后删除所有探针。

最常见的原因是表达式的求值类型（float 还是 double）与原版不一致：每个表达式
都要按原版的类型求值，float 与 float 运算仍是 float，`IVP_FLOAT op= IVP_DOUBLE`
先按 double 计算再舍入一次。

## 注意

- **runner 的位置：**原版 runner 不能和 libivp 的 `test_scenarios` 放在同一目录，否则它会改为调用后者
  （`main.cxx` 中的 `run_c17_test_scenarios`）。本目录的脚本把两者分开放置。
- **与 regen_reference.sh 的关系：**`tools/regen_reference.sh` 生成的是仓库里的参考数据（`tools/testdata`，strict release）。
  这里的脚本只在 `build/parity` 下生成临时输出，不改动仓库内容。
- **平台：**逐位一致在 macOS arm64 上验证过。覆盖文件的编译选项（`-DAPPLE -DUNIX`）定义在 `common.sh` 中，
  其它平台需要按原版 CMake 生成的选项调整。
