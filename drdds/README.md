# drdds (vendored subset)

This is **not** a package of ours. It is a byte-for-byte copy of the message definitions that the
Deep Robotics M20 ships in its own `drdds` interface package, reduced to the closure of types the
M20 localization module actually exchanges.

## Why it exists

The M20's on-board software runs ROS 2 **Foxy** and owns the real `drdds` package. The Fixposition
driver runs in a **Humble** container, where that package does not exist. DDS matches endpoints by
*topic name* and *type name*, and the type name is derived from the package name — so a Humble-side
package named `drdds` with identical field layouts talks to the robot's Foxy nodes directly.

That is why the package name must stay `drdds`, and why the `.msg` files must not be "cleaned up".
Renaming the package, reordering fields, or changing a type breaks the wire contract silently: the
endpoints simply never match, or worse, decode garbage.

## Provenance

Copied on 2026-08-04 from `/opt/ros/foxy/share/drdds/msg/` on the ChongQing M20 (CPU unit 103).

The robot's root filesystem also carries `/root-ro/opt/ros/humble/share/drdds`. **Do not use it.**
It is an older, incompatible generation of the same package: `MetaType` has a different layout and
`MotionInfo` does not exist in it at all. Only the Foxy tree matches what is on the wire today.

## Contents

| Message | Direction | Used for |
|---|---|---|
| `MotionInfo` (+ `MotionInfoValue`, `MotionStateValue`, `GaitValue`) | input | leg odometry from the robot; body velocity is forwarded to the VRTK2 as wheelspeed aiding |
| `LocationStatus` (+ `LocationStatusValue`, `ExecStatusValue`, `LossStatusValue`, `InputStatusValue`) | output | localization health reported to the OEM navigation stack at 2 Hz |
| `MetaType` | both | OEM stamp/sequence header |

Note the OEM's inconsistent naming, which the definitions preserve: `LocationStatus` names its
header field `meta`, while `MotionInfo` names its own `header`.

## Adding messages

Copy the `.msg` verbatim from the robot's Foxy tree (along with everything it references) and add it
to `CMakeLists.txt`. Never hand-write a definition from documentation.
