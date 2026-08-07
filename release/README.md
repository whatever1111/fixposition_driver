# Binary-only releases

Every ROS 2 distro branch (`m20-foxy`, `m20-humble`, later `m20-jazzy`) gets its
own release line. A release ships **no source code** — the workflow audits every
artifact and fails if a header, C/C++ source or python file survives the prune.

## Cutting a release

Tag the branch head with `<distro>-v<version>` and push the tag:

```bash
git tag foxy-v1.0.1 m20-foxy   && git push origin foxy-v1.0.1
git tag humble-v1.0.1 m20-humble && git push origin humble-v1.0.1
```

or run the `release` workflow manually from the branch (Actions →
release → Run workflow) — it creates the tag for you at the dispatched commit.

## What a release contains

| asset | contents |
|---|---|
| `fixposition-driver-m20_<ver>_<distro>_arm64.tar.gz` | native install tree: `fpsdk/` (SDK libs), `install/` (colcon tree), `env.sh` |
| `fixposition-driver-m20_<ver>_<distro>_arm64_docker-image.tar.gz` | `docker load`-able runtime image |
| `SHA256SUMS.txt` | checksums |

The image is additionally pushed to
`ghcr.io/whatever1111/fixposition-driver-m20:<distro>-<version>` (and
`<distro>-latest`) — the tarball asset exists because deployment targets have no
internet access.

Kept in the artifacts: shared libraries, executables, `.msg` interface
definitions, launch/config share files, colcon environment scripts.
Stripped: `include/` trees, all C/C++ sources and headers, static libs,
python bindings, cmake/pkg-config dev files.

GitHub auto-attaches "Source code (zip / tar.gz)" links to every release; that
is a GitHub feature that cannot be disabled and is independent of the uploaded
assets.

## Per-distro notes

- **foxy** — the native tarball does **not** include `drdds`: on the robot it is
  a system package and a vendored copy would shadow it. The docker image keeps
  its own vendored drdds build and is self-contained. The SDK is built without
  `BagWriter` (Foxy's rosbag2 header layout — see `DISTRO.md` on `m20-foxy`).
- **humble** — vendored `drdds` is included in both artifacts.

## Build machinery

`release/Dockerfile` is one multi-stage file for all distros
(`--build-arg ROS_DISTRO=...`): a `builder` stage compiles the SDK
(cmake, PROJ off) and the driver workspace (colcon), prunes the staged
`/opt/fixposition` tree down to binaries; the published `runtime` stage carries
only that tree plus runtime apt dependencies, and re-runs the no-source audit +
an `ldd` completeness check as build steps.
