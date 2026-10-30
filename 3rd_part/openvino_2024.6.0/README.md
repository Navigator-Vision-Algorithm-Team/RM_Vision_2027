# 内置 OpenVINO 2024.6.0 运行库

本目录随仓库提供两套官方预编译运行库，`cmake/platform.cmake` 按目标架构自动选择对应的一套，
业务代码与配置文件无需关心。版本统一钉在 2024.6.0，避免不同机器的 apt / pip 版本差异。

| 目标架构 | 目录 | 官方包 | lib 子目录 |
| --- | --- | --- | --- |
| aarch64 | `aarch64/runtime/` | `l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64.tgz` | `lib/aarch64` |
| x86_64 | `x86_64/runtime/` | `l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz` | `lib/intel64` |

下载源：`https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/`

## 为什么按架构分目录

官方包内的 `runtime/cmake/OpenVINOConfig.cmake` 与 `OpenVINOTargets-release.cmake` 把库路径
**写死**为各自架构（`lib/aarch64` 或 `lib/intel64`），两份配置无法合并成一份，因此每个架构
各放一棵完整的 `runtime/` 树，互不干扰。

## 保留内容

只保留 `runtime/` 下的 `cmake/ include/ lib/ 3rdparty/ version.txt`，删除了包内的
`docs/ samples/ python/ tools/ install_dependencies setupvars.sh` 等与运行无关的内容。

在此之上按架构做了最小裁剪：

- aarch64：官方包本身即只含 ARM CPU 插件，未做额外裁剪。
- x86_64：删除 `lib/intel64/cache.json`（8.5MB GPU 预编译缓存，首次运行会自动重建）与
  `libopenvino_intel_npu_plugin.so*`（本车不使用 NPU）。

## 校验值

| 包 | SHA256 |
| --- | --- |
| arm64 (ubuntu20) | `e094e7c3e3a3931213aeeb6d6abb6cc4d654f5cb4df8da2914158ba62e25e33a` |
| x86_64 (ubuntu22) | `e6d9d9d32e98ffe329c7dc3c1688ba0f9466308be118117f8896049366ab39c0` |

## 重新获取 / 升级

```bash
cd 3rd_part
URL="https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux"

# aarch64
curl -fSL -o ov_arm64.tgz "$URL/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64.tgz"
echo "e094e7c3e3a3931213aeeb6d6abb6cc4d654f5cb4df8da2914158ba62e25e33a  ov_arm64.tgz" | sha256sum -c -
mkdir -p tmp_arm64 && tar -xzf ov_arm64.tgz -C tmp_arm64
mv tmp_arm64/*/runtime openvino_2024.6.0/aarch64/runtime

# x86_64
curl -fSL -o ov_x86_64.tgz "$URL/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz"
echo "e6d9d9d32e98ffe329c7dc3c1688ba0f9466308be118117f8896049366ab39c0  ov_x86_64.tgz" | sha256sum -c -
mkdir -p tmp_x86_64 && tar -xzf ov_x86_64.tgz -C tmp_x86_64
mv tmp_x86_64/*/runtime openvino_2024.6.0/x86_64/runtime
rm -f openvino_2024.6.0/x86_64/runtime/lib/intel64/cache.json \
      openvino_2024.6.0/x86_64/runtime/lib/intel64/libopenvino_intel_npu_plugin.so*
```

## 不使用内置库

任意平台都可以用系统安装的 OpenVINO 覆盖内置版本：

```bash
cmake -B build -DOpenVINO_DIR=/path/to/openvino/runtime/cmake
```

如果连内置目录都删掉，`platform.cmake` 也会自动回退到 `/opt/intel/openvino_2024.6.0`，
再退回 `find_package(OpenVINO)` 搜索系统路径（apt 安装）。
