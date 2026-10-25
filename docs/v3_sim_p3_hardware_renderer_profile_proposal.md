# P3 hardware-renderer profile — proposal `p3-acceptance-v1.2` (D3D12 + `libd3d12core` lifetime fix)

**Status: PROPOSAL ONLY — NOT APPLIED, NOT APPROVED.** P3 remains INCOMPLETE, qualification
0/3. Existing Gate A (accepted, `764fcfd7`) and Gate B (failed, `764fcfd7`) results and their
consumed budgets are unchanged. The llvmpipe candidate `764fcfd7` is not modified.

Date: 2026-10-25. Host: WSL 3.0.1.0, kernel 6.18.40.1-1, WSLg 1.0.79, Direct3D 1.611.1-81528511,
DXCore 10.0.26100.1, Windows 10.0.22631.5335, NVIDIA GeForce RTX 4060 Laptop GPU driver 571.96,
Mesa 25.2.8-0ubuntu0.24.04.2.

## 1. Confirmed crash cause

Gazebo-only diagnostic `20261025T093020Z-gz-d3d12-trace-baseline`. The gdb trace logged every
`dlclose` and unload event and captured `/proc/<pid>/maps` immediately before SIGINT; gdb
disables ASLR, so addresses are stable.

1. The faulting PC `0x7fff94641b10` lay, before SIGINT, in the executable text of
   **`/usr/lib/wsl/lib/libd3d12core.so`** (mapping `7fff94636000-7fff94991000 r-xp`, file
   offset `0x2edb10`). At the crash it was unmapped ("Cannot access memory").
2. During shutdown, on the sensors **render thread** (thread 8), Mesa/EGL teardown closes
   `libd3d12.so`. Its static destructor (`__cxa_finalize`) calls `dlclose(libd3d12core.so)`,
   and the loader reports "Inferior unloaded `/usr/lib/wsl/lib/libd3d12core.so`".
3. The render thread then exits. glibc's thread-specific-data cleanup
   (`__nptl_deallocate_tsd`, the caller frame recorded in section M) calls a destructor
   located in the now-unmapped `libd3d12core.so` text, causing SIGSEGV.

**Confirmed:** module (`libd3d12core.so`, Microsoft's WSL D3D12 runtime, not Mesa or the
NVIDIA user-mode driver), unload path (`libd3d12.so` destructor on the render thread), and
the faulting address inside that module. **Inferred, consistent with every observation but
not read directly:** that the entry is a TSD key destructor registered by
`libd3d12core.so`. The TSD table dump did not run because the gdb script stopped at the
unmapped PC.

## 2. Repair tested (exactly one variant)

`20261025T093153Z-gz-d3d12-trace-preload-d3d12core`: identical run with
`LD_PRELOAD=/usr/lib/wsl/lib/libd3d12core.so` for the Gazebo server process only.

* `libd3d12.so`'s destructor still calls `dlclose(libd3d12core.so)`, but the module is **not
  unloaded** (the preload holds a reference); 0 unload events, against 1 in the baseline.
* `SensorsPrivate::RenderThread stopped` → `Shutting down gz-sim-server` →
  `[Inferior 1 exited normally]`: **exit 0**, no residual processes.
* Renderer: **D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)**.
* One sample. A Gate A run would add 8 Gazebo shutdowns of evidence.

Pinned identities (SHA-256): `libd3d12core.so` `a4104a20…894d`, `libd3d12.so` `b3d78d40…a8c3`,
`libdxcore.so` `83d1671a…190c`, `libnvwgf2umx.so` `36c79d26…400f`,
`libgallium-25.2.8-0ubuntu0.24.04.2.so` `e3748642…cca8b`.

## 3. Proposed profile (requires approval; nothing applied)

| Item | v1.1 (current) | v1.2 (proposed) |
|---|---|---|
| Renderer requirement | `software:llvmpipe` | `hardware:d3d12` with the recorded RTX 4060 renderer string |
| Owned-process environment | no D3D12 variables | `GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` |
| Lifetime fix | — | `LD_PRELOAD=/usr/lib/wsl/lib/libd3d12core.so` for **Gazebo server processes only**, through a P3-owned `gz` shim first on PX4's `PATH` that adds it only for `gz sim`; never in PX4, ROS, QGC or bridges |
| Manifest binding | — | the five library hashes above plus the shim's hash; any change blocks the run |
| Every sensor, freshness (including the 2 s `vehicle_land_detected` wall limit), timing, recorder, policy A, MC and graceful-cleanup rule | — | **unchanged** |

The shim is needed because PX4's `px4-rc.gzsim` starts `gz sim` itself with PX4's
environment. Setting `LD_PRELOAD` on PX4 would inject the library into the pinned PX4
process, which is rejected.

## 4. Environment updates (not applied)

No public report or fix matches this failure. A newer Ubuntu Mesa build
(`25.2.8-0ubuntu0.24.04.4`) is available, but the faulting module is Microsoft's
`libd3d12core.so`, shipped with the WSL Direct3D component (1.611.1, file dated
2023-10-20), so a Mesa update is not expected to address it. Any WSL, Windows or NVIDIA driver
update needs separate approval and would invalidate the pinned hashes.

## 5. Approvals required before any change

1. Approve contract revision `p3-acceptance-v1.2` and the Gazebo-only shim design above.
2. After implementation and regression: Policy A (unchanged) re-approval for the new commit.
3. A fresh Gate A authorization for the new frozen candidate, then B, then C separately.

A clean Gazebo shutdown is not evidence that flight timing passes. D3D12's readiness RTF of
0.82 has not yet been exercised in flight.
