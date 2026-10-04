# tools/parity — 与原版 IVP 的逐位对照

libivp 与原版引擎（默认 `../ivp`）在 `tools/scenarios.txt` 的全部场景中逐位一致：

- **Release 构建：**对照以 `-ffp-contract=off` 构建的原版。
- **Debug 构建：**对照不定义 `NDEBUG`、定义 `DEBUG` 的原版。

本目录的脚本用来构建原版、批量运行场景，并比较输出。

## 准备

原版引擎和 runner 都在 `https://github.com/doyaGu/ivp` 的 `master` 上（对照的引擎版本为 `8b81109`，其后只有 runner 的提交），需要克隆到与本仓库同级的 `../ivp`：

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

退出码为 0 表示全部一致。每个场景的结果写在 `build/parity/out/{release,debug,sweep_*}.txt`。

各项检查比较的内容：

| 检查 | 比较内容 |
|---|---|
| release | JSONL 帧（pos、quat、lin_vel、ang_vel 逐位）、stdout 中的非 JSON 行（引擎输出）、stderr、退出码 |
| debug | JSONL 帧、调试输出文本（统计行、`linear_constraint_failed` 等）、stderr 中 `ASSERTION FAILURE: <expr>` 报告的序列、退出码 |
| sweep | 每个场景在给定步长和步数下的 JSONL 帧 |

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
| `compare.sh [--text] [--stderr] [--asserts] <ref_dir> <lib_dir>` | 逐场景比较两次运行的输出 |
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
   `build/parity/lib_release/test_scenarios --scenario X --steps N --dt D --output x.jsonl`。
4. **逐值比较：**比较两边的 stderr，找出第一个不同的值。用完后删除所有探针。

最常见的原因是表达式的求值类型（float 还是 double）与原版不一致，规则见
[docs/PORT_WORK_REPORT.md](../../docs/PORT_WORK_REPORT.md) 第 5.1 节。

## 注意

- **runner 的位置：**原版 runner 不能和 libivp 的 `test_scenarios` 放在同一目录，否则它会改为调用后者
  （`main.cxx` 中的 `run_c17_test_scenarios`）。本目录的脚本把两者分开放置。
- **与 regen_reference.sh 的关系：**`tools/regen_reference.sh` 生成的是仓库里的参考数据（`tools/testdata`，strict release）。
  这里的脚本只在 `build/parity` 下生成临时输出，不改动仓库内容。
- **平台：**逐位一致在 macOS arm64 上验证过。覆盖文件的编译选项（`-DAPPLE -DUNIX`）定义在 `common.sh` 中，
  其它平台需要按原版 CMake 生成的选项调整。
