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

开机自启就写一个 systemd unit 指向上面的命令;`Restart=on-failure` 即可,不需要 launch 的 respawn。
For boot start, write a systemd unit around the command above; `Restart=on-failure` is enough and
launch-level respawn is not needed.

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

### 镜像不带配置 | The image ships no config

`CMD` 指向 `/config/fixposition.yaml`,必须自己挂进去。忘了挂,`v1.0.1` 起会给出明确报错并退出 2;
`v1.0.0` 只会甩一句 `Error opening YAML file` 加一屏 C++ 栈,看不出是缺挂载。
`CMD` points at `/config/fixposition.yaml` and you must mount one. If you forget, `v1.0.1`+ exits 2
with a clear message; `v1.0.0` only printed `Error opening YAML file` plus a C++ stack trace that
never mentioned the missing mount.

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
