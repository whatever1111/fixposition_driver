# 部署指南(仅驱动)| Deployment Guide (driver only)

本文讲**只部署 fixposition 驱动**:进程连 FP 设备(VRTK2 等),发 fixposition 话题,配置由你自己给。
需要 M20 的 OEM 三话题契约(`/ODOM` + `/LOC_BODY_POINTS` + `/LOCATION_STATUS`)请看 fslam 仓库的
`DEPLOYMENT.md` —— 那一套在本驱动之上加了产品层。

This covers deploying **the fixposition driver alone**: it connects to the FP sensor (VRTK2 and
friends), publishes fixposition topics, and takes a configuration you supply. If you need the M20
OEM three-topic contract (`/ODOM` + `/LOC_BODY_POINTS` + `/LOCATION_STATUS`), see `DEPLOYMENT.md`
in the fslam repo — that adds the product layer on top of this driver.

发布物不含源码,两种形态:**原生二进制包**和 **docker 镜像**。二者用同一批二进制。
Releases are binary-only and come in two forms — a **native tarball** and a **docker image** —
built from the same binaries.

---

## 0. 先选发行版 | Pick the distro first

| 目标机 ROS 2 | 用哪条发布线 | 分支 |
|---|---|---|
| Foxy | `foxy-v*` | `m20-foxy` |
| Humble | `humble-v*` | `m20-humble` |

> **M20 机器狗必须用 Foxy。** OEM 雷达的点云 writer 只绑 127.0.0.1;Humble 的 Fast DDS 2.6 不再为
> 多网卡参与者通告 loopback locator,**能在 graph 里看到该话题(类型、publisher 数都对),但永远收不到
> 数据**。实测(2026-08-08,已排除 `/dev/shm` 变量):Humble 容器 4 次探测 0 数据,同机 Foxy 首次探测即
> 有数据。详见 `DISTRO.md`。
>
> **The M20 robot must run Foxy.** The OEM cloud writer is bound to 127.0.0.1 only; Humble's
> Fast DDS 2.6 no longer announces a loopback locator for multi-NIC participants, so it **sees the
> topic in the graph — correct type and publisher count — and never receives a single message**.
> Measured 2026-08-08 with the `/dev/shm` variable ruled out: Humble container, 0 data in 4 probes;
> Foxy on the same host, data on the first probe. See `DISTRO.md`.

其它平台(不依赖那个 loopback-only writer)两条发布线都能用。
On any other platform — one that does not depend on that loopback-only writer — either line works.

---

## 1. 方式 A:原生二进制包 | Native tarball

目标机需已装对应发行版的 ROS 2。包内**不含 ROS 本体**。
The target must already have the matching ROS 2 distro installed; the tarball does not ship ROS.

```bash
# 校验后解开 | verify then unpack
sha256sum -c SHA256SUMS.txt
tar -xzf fixposition-driver-m20_<ver>_<distro>_arm64.tar.gz
cd fixposition-driver-m20

# env.sh 会 source ROS + overlay 并设好 LD_LIBRARY_PATH(可在 set -u 下安全 source)
# env.sh sources ROS + the overlay and sets LD_LIBRARY_PATH (safe under set -u)
source env.sh

ros2 run fixposition_driver_ros2 fixposition_driver_ros2_exec \
    --ros-args --params-file /path/to/your.yaml
```

目录结构 | layout:

```
fixposition-driver-m20/
├── env.sh          # 环境入口 | environment entry point
├── entrypoint.sh   # docker 用 | used by the image
├── .ros_distro     # 该包对应的发行版 | the distro this build targets
├── fpsdk/lib/      # Fixposition SDK 动态库 | SDK shared libraries
└── install/        # colcon 安装树(二进制 + msg + launch/config)
```

> **Foxy 包不含 `drdds`** —— 狗上它是系统包,再带一份会遮蔽掉,导致类型不一致。Humble 包自带。
> **The foxy tarball ships no `drdds`** — it is a system package on the robot and a bundled copy
> would shadow it. The humble tarball includes one.

> 发布物无 python typesupport,所以 **fixposition 自定义消息不能 `ros2 topic echo`**(标准消息和
> OEM 的 drdds 话题不受影响)。
> Releases carry no python typesupport, so **fixposition custom messages cannot be `ros2 topic
> echo`-ed** (standard messages and the OEM's drdds topics are unaffected).

### 1.1 包里的脚本和 launch 文件 | Scripts and launch files in the tarball

| 文件 file | 用途 purpose | 备注 notes |
|---|---|---|
| `env.sh` | 环境入口:source ROS + overlay,设 `LD_LIBRARY_PATH` | 可在 `set -u` 下安全 source safe to source under `set -u` |
| `entrypoint.sh` | 镜像入口;原生也能当包装脚本用 image entry point, also usable natively | 先 source `env.sh`,再查配置在不在,最后 `exec "$@"` |
| `install/.../launch/node.launch` | 上游 launch 文件 upstream launch file | **Foxy 用不了** — 见下 |
| `install/.../launch/dev.launch` | 开发用 development | 同上 same |
| `install/.../launch/config*.yaml` | 自带默认配置 bundled defaults | 见 §3.1 |

> **`node.launch` 在 Foxy 上直接报错,别用。** 它带 `respawn` / `respawn_delay` / `ros_args`,这些是
> Galactic 以后才有的 XML 属性,Foxy 的 `launch_xml` 会抛 `InvalidLaunchFileError`。放进 systemd 里更坑:
> unit 看着是 `active`,进程其实压根没起来,话题一个都没有。**Foxy 一律用 §1 的 `ros2 run ...
> --params-file`**;Humble 上 `ros2 launch` 是好的。
> **`node.launch` fails outright on Foxy — do not use it.** It carries `respawn`, `respawn_delay`
> and `ros_args`, all Galactic-and-later XML attributes, so Foxy's `launch_xml` throws
> `InvalidLaunchFileError`. Inside systemd this is worse: the unit reads `active` while the process
> never started and no topics appear. **On Foxy always use the `ros2 run ... --params-file` form from
> §1**; on Humble `ros2 launch` works fine.

仓库里另有 `setup_ros_ws.sh` / `create_ros_ws.sh` / `precommit.sh` —— 那些是**从源码开发**用的,不参与
二进制部署,发布包里也没有。
The repo also carries `setup_ros_ws.sh`, `create_ros_ws.sh` and `precommit.sh`: those are for
**building from source**, play no part in binary deployment, and are not in the release.

### 1.2 最小启动脚本 | A minimal launcher script

本仓库不像 fslam 那样自带 `run_*.sh`(驱动是通用件,启动策略归产品层)。要一个能直接跑的脚本,把下面这段
放到解开后的目录里存成 `run.sh` 就行,Foxy / Humble 通用:
This repo ships no `run_*.sh` the way fslam does — the driver is a generic component and launch
policy belongs to the product layer. If you want one, drop this into the unpacked directory as
`run.sh`; it works on both Foxy and Humble:

```bash
#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${1:-${HERE}/my.yaml}"
[[ -f "${CONFIG}" ]] || { echo "config not found: ${CONFIG}" >&2; exit 2; }

source "${HERE}/env.sh"          # env.sh 自己处理 set -u | env.sh handles set -u itself

exec ros2 run fixposition_driver_ros2 fixposition_driver_ros2_exec \
    --ros-args -r __node:=fixposition_driver_ros2 \
    --params-file "${CONFIG}"
```

```bash
chmod +x run.sh
./run.sh /etc/fixposition/my.yaml       # 前台,Ctrl-C 停 | foreground, Ctrl-C stops it
```

开机自启:`exec` + `Type=simple` + `Restart=on-failure`,systemd 负责保活,不需要 launch 的 respawn。
For boot start, `exec` + `Type=simple` + `Restart=on-failure` lets systemd supervise it; launch-level
respawn is not needed.

```ini
# /etc/systemd/system/fixposition_driver.service
[Unit]
Description=Fixposition driver
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
Restart=on-failure
RestartSec=5
ExecStart=/opt/fixposition/run.sh /etc/fixposition/my.yaml

[Install]
WantedBy=multi-user.target
```

M20 那套完整链路(三话题契约 + 启动脚本 + unit)在 fslam 仓库,别在这里重造。
The full M20 chain (three-topic contract, launchers, unit) lives in the fslam repo — don't rebuild it
here.

---

## 2. 方式 B:Docker 镜像 | Docker image

```bash
# 目标机没有公网就用 release 里的镜像包 | no internet on the target: load the asset
docker load < fixposition-driver-m20_<ver>_<distro>_arm64_docker-image.tar.gz
# 有公网也可以直接拉 | with internet you can pull instead
# docker pull ghcr.io/whatever1111/fixposition-driver-m20:<distro>-<ver>

docker run --rm --network host -v /dev/shm:/dev/shm \
    -v /path/to/your.yaml:/config/fixposition.yaml \
    ghcr.io/whatever1111/fixposition-driver-m20:<distro>-<ver>
```

### 两个 flag 都是必须的,少任何一个都是"静默失效" | Both flags are mandatory; either one missing fails silently

- `--network host` —— 传感器和 DDS 流量都在宿主网络上。
  The sensor and DDS traffic live on the host's interfaces.
- `-v /dev/shm:/dev/shm` —— Fast DDS 的共享内存传输就是 `/dev/shm` 里的 POSIX shm 文件。docker 默认给容器
  一个**私有 64 MB `/dev/shm`**,于是容器**照常启动、照常发布、日志一切正常,但一条数据都收不到**。
  Fast DDS's shared-memory transport *is* POSIX shm files under `/dev/shm`. Docker gives a container
  a **private 64 MB `/dev/shm`** by default, so the container **starts, publishes, and logs nothing
  wrong — while receiving nothing at all**.

  实测(M20,2026-08-08)| measured on the M20:

  | 配置 configuration | 收到数据? received? |
  |---|---|
  | 宿主命名空间 host namespace | 是 yes(首次探测 first probe) |
  | 容器,仅 `--network host` container, `--network host` only | **否 no**(4 次探测 4 probes) |
  | 容器,加 `-v /dev/shm:/dev/shm` container, plus the mount | 是 yes(首次探测 first probe) |

  `--ipc host` 也可以(是前者的超集);IPC 命名空间本身无关紧要,**起作用的是 `/dev/shm`**。
  `--ipc host` also works (a superset); the IPC namespace itself is irrelevant — **`/dev/shm` is
  the load-bearing part**.

### `/config` 要自己挂 | `/config` must be mounted

`CMD` 指向 `/config/fixposition.yaml`,镜像里没有这个路径,所以按上面那条命令跑就必须挂配置。忘了挂,
`v1.0.1` 起会给出明确报错并退出 2;`v1.0.0` 只会甩一句 `Error opening YAML file` 加一屏 C++ 栈,看不出
是缺挂载。镜像**确实自带**默认配置,只是不在 `/config` —— 见 §3.1,可以直接 `--params-file` 指过去。
`CMD` points at `/config/fixposition.yaml`, a path the image does not contain, so the command above
requires a mount. If you forget, `v1.0.1`+ exits 2 with a clear message; `v1.0.0` only printed
`Error opening YAML file` plus a C++ stack trace that never mentioned the missing mount. The image
**does** bundle default configs, just not at `/config` — see §3.1, where you can point
`--params-file` straight at them.

> **M20 狗上永远只用 `--network host` 或 `--network none`。** 默认 bridge 网络创建 veth 会触发内核
> RTNL 死锁,ssh 直接断、dockerd 全局卡死,只能整机断电重启。
> **On the M20 robot use only `--network host` or `--network none`.** Creating a bridge veth
> triggers a kernel RTNL deadlock: ssh dies, dockerd wedges globally, and only a power cycle recovers.

---

## 3. 配置文件 | Configuration

参数以 `/**` 通配符打头,这样节点名怎么改都能吃到 | Key the file on `/**` so it applies regardless of
the node name:

```yaml
/**:
  ros__parameters:
    stream: "tcpcli://10.21.31.66:21000"
    # ...
```

镜像 `CMD` 没有做节点重映射,所以容器里节点名是 `fixposition_driver`;原生方式想固定名字自己加
`-r __node:=...`。
The image's `CMD` does no node remapping, so the container's node is named `fixposition_driver`.
Add `-r __node:=...` yourself in the native path if you want a fixed name.

### 3.1 自带的默认配置 | The bundled default configs

发布物**自带两份配置**,原生包和镜像里都有,路径一样:
Releases **bundle two configs**, present in both the tarball and the image at the same path:

```
install/fixposition_driver_ros2/share/fixposition_driver_ros2/launch/
├── config.yaml       # 通用默认(纯 fixposition)| generic fp-only defaults
└── config_m20.yaml   # M20 定位接口 | the M20 localization interface
```

镜像里的绝对路径 | absolute path inside the image:
`/opt/fixposition/install/fixposition_driver_ros2/share/fixposition_driver_ros2/launch/config.yaml`

拿它当起点最省事 | the easiest way to start from it:

```bash
# 原生 | native
cp install/fixposition_driver_ros2/share/fixposition_driver_ros2/launch/config.yaml my.yaml
# 至少要改 stream(默认是示例地址 10.0.2.1)| at minimum change stream (the default is the example 10.0.2.1)

# 镜像:直接用自带的那份跑(不挂任何配置)| image: run the bundled one directly, no mount
docker run --rm --network host -v /dev/shm:/dev/shm <image> \
    ros2 run fixposition_driver_ros2 fixposition_driver_ros2_exec --ros-args --params-file \
    /opt/fixposition/install/fixposition_driver_ros2/share/fixposition_driver_ros2/launch/config.yaml
```

> 镜像默认的 `CMD` 读的是 `/config/fixposition.yaml`,**那个路径**镜像里没有,所以按 §2 的方式跑必须
> 挂配置进去。自带的默认配置在上面那个 share 路径下,不在 `/config`。
> The image's default `CMD` reads `/config/fixposition.yaml`, and **that path** does not exist in the
> image, so the §2 invocation requires a mount. The bundled defaults live under the share path above,
> not under `/config`.

### 3.2 话题名可以配吗?可以 | Are topic names configurable? Yes

**输入 | inputs**

| 用途 purpose | 参数 parameter | 默认 default |
|---|---|---|
| 轮速直接输入 wheelspeed input | `speed_topic` | `/fixposition/speed`(留空禁用 empty disables) |
| RTCM 差分输入 correction input | `corr_topic` | `/rtcm`(留空禁用 empty disables) |
| 从现有运动话题转换出轮速 convert wheelspeed from an existing motion topic | `converter.enabled` | `false` |
| ↳ 订阅哪个话题 which topic to subscribe | `converter.input_topic` | `/robot/odom` |
| ↳ 该话题的类型 its type | `converter.topic_type` | `Twist`(可选 `TwistWithCov` / `Odometry`) |
| ↳ 单位换算到 mm/s scale to mm/s | `converter.scale_factor` | `1000.0`(必须是浮点 must be a float) |
| ↳ 用哪几个轴 which axes | `converter.use_x` / `use_y` / `use_z` | `true` / `false` / `false` |

所以"运动速度话题名"有两条路:自己往 `speed_topic` 发 fixposition 的轮速消息,或者打开 `converter`
让驱动直接订阅你已有的 `Twist`/`Odometry` 话题并自动换算。
So there are two routes for the motion/speed topic: publish fixposition wheelspeed messages to
`speed_topic` yourself, or enable `converter` and let the driver subscribe to your existing
`Twist`/`Odometry` topic and do the conversion.

**输出 | outputs**

| 参数 parameter | 默认 default | 作用 effect |
|---|---|---|
| `output_ns` | `/fixposition` | 所有输出话题的命名空间前缀;留空则用节点命名空间 namespace prefix for every output topic; empty falls back to the node namespace |

**注意:单个输出话题名不是参数。** 它们是 `output_ns` 加固定后缀拼出来的,例如
`<ns>/fpa/odometry`、`<ns>/odometry_ecef`、`<ns>/odometry_llh`、`<ns>/poiimu`、`<ns>/odometry_smooth`。
改 `output_ns` 能整棵搬走;想单独改某一个,用 ROS 2 的 remap:
**Note: individual output topic names are not parameters.** They are `output_ns` plus a fixed
suffix — `<ns>/fpa/odometry`, `<ns>/odometry_ecef`, `<ns>/odometry_llh`, `<ns>/poiimu`,
`<ns>/odometry_smooth` and so on. `output_ns` moves the whole tree; to rename just one, use ROS 2
remapping:

```bash
ros2 run fixposition_driver_ros2 fixposition_driver_ros2_exec --ros-args \
    --params-file my.yaml \
    -r /fixposition/odometry_enu:=/my/odom
```

M20 定位接口那套话题名则是**逐个可配**的(`m20.odom_topic` / `m20.lidar_topic` /
`m20.motion_info_topic` / `m20.matching_odom_topic`),见 fslam 仓库的 `DEPLOYMENT.md`。
The M20 localization interface, by contrast, makes its topic names **individually configurable**
(`m20.odom_topic`, `m20.lidar_topic`, `m20.motion_info_topic`, `m20.matching_odom_topic`) — see
`DEPLOYMENT.md` in the fslam repo.

---

## 4. 验证 | Verify

```bash
# 进程活着、连上设备了吗 | is it alive and connected to the sensor?
#   看日志里的 DriverParams: stream=...  | look for DriverParams: stream=... in the log
ros2 node list | grep fixposition
ros2 topic list | grep -i fixposition
```

FP 设备本身状态(GNSS 没定位时驱动会如实扣着数据不发,这是对的,不是 bug):
Sensor-side status — with no GNSS fix the driver correctly withholds data rather than publishing
zeros, which is not a bug:

```bash
ros2 topic echo /fixposition/fpa/odomstatus     # init_status / gnss1_status / corr_status
```

**Foxy 的 `ros2 topic echo` 没有 `--once` 和 `--field`**(Galactic 才加);另外本机上探针会阶段性"聋"
—— 任何"收不到"的结论都要拿同一条命令的宿主对照组来兜底,否则就是自欺欺人。
**Foxy's `ros2 topic echo` has no `--once` or `--field`** (those arrived in Galactic). Probes here
also go deaf intermittently, so never conclude "not receiving" without a host-namespace control run
with the same command.

不想惊动生产环境又要试容器:加 `-e ROS_DOMAIN_ID=42`。它和 domain 0 的活跃栈物理上不可能匹配,但仍能
连上 FP 设备(TCP,不走 DDS),启动路径照样验完。
To exercise a container against real hardware without disturbing production, add
`-e ROS_DOMAIN_ID=42`: it cannot match the live domain-0 stack, yet still reaches the FP sensor
(TCP, not DDS), so the whole startup path still gets verified.

---

## 5. 常见故障 | Troubleshooting

| 现象 symptom | 原因 cause | 处理 fix |
|---|---|---|
| 容器跑着、发得出、就是收不到 container runs and publishes but receives nothing | 缺 `-v /dev/shm:/dev/shm` | 加上该挂载 add the mount |
| `Error opening YAML file` + C++ 栈 | 配置没挂进去 config not mounted | 挂配置;升到 `v1.0.1`+ 会直接说清楚 mount it; `v1.0.1`+ says so plainly |
| Humble 上 graph 有话题但零数据 topic visible, zero data on Humble | Fast DDS 2.6 loopback locator 墙 | 换 Foxy 发布线 switch to the foxy line |
| 自定义消息 `ros2 topic echo` 报没有 typesupport | 发布物不含 python typesupport by design | 用日志或 C++ 订阅者 use logs or a C++ subscriber |
| `estimated delay ~7.7 s` 告警 | FP 设备钟与宿主钟的差 sensor-vs-host clock offset | 看时间同步链路 check time sync |

---

## 6. 升级与回滚 | Upgrade and rollback

原生包换目录、重启服务即可;镜像换 tag 重启即可。两种形态都不改宿主系统,回滚就是切回上一个版本。
For the tarball, swap the directory and restart the service; for the image, change the tag and
restart. Neither touches the host system, so rolling back is just going back to the previous version.

发版流程见 `release/README.md`。| Cutting a release: see `release/README.md`.
