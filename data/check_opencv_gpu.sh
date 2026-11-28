#!/usr/bin/env bash
# Check whether OpenCV in the container is built with / can use a GPU
# (CUDA and/or OpenCL backends), and whether the project's nodes use it.
#
# Run from the host:
#   docker compose -f compose.sim.yaml exec -it ros2_gazebo bash /root/data/check_opencv_gpu.sh

set -u

section() { printf '\n== %s ==\n' "$*"; }

section "Installed OpenCV (system)"
if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists opencv4; then
    echo "pkg-config opencv4: $(pkg-config --modversion opencv4)"
elif command -v pkg-config >/dev/null 2>&1 && pkg-config --exists opencv; then
    echo "pkg-config opencv: $(pkg-config --modversion opencv)"
else
    echo "no pkg-config opencv module found"
fi

section "Python cv2 (used by the project nodes) + GPU backends"
python3 - <<'PY'
import importlib.util, re, sys, time

if importlib.util.find_spec("cv2") is None:
    print("python3 cv2 module NOT installed")
    sys.exit(0)

import cv2
print("cv2 version : %s" % cv2.__version__)
print("cv2 location: %s" % cv2.__file__)

build = cv2.getBuildInformation()
def build_line(label, prefix):
    for line in build.splitlines():
        s = line.strip()
        if re.match(r"^\s*" + re.escape(prefix) + r"\b", s, re.IGNORECASE):
            val = s.split(":", 1)[1].strip() if ":" in s else s
            print("%-12s: %s" % (label, val))
            return True
    print("%-12s: (not in build info)" % label)
    return False

print("\n-- build flags --")
cuda_built = build_line("CUDA", "CUDA")
build_line("CUBLAS", "CUBLAS")
build_line("cuDNN", "CUDNN")
build_line("OpenCL", "OpenCL")
build_line("NVIDIA", "NVIDIA")

print("\n-- runtime backends --")
# CUDA
if hasattr(cv2, "cuda"):
    n = cv2.cuda.getCudaEnabledDeviceCount()
    if n > 0:
        print("cv2.cuda      : YES, %d CUDA device(s)" % n)
        for i in range(n):
            p = cv2.cuda.getCudaEnabledDeviceProperties(i)
            print("  device %d: %s (arch %d.%d)" % (i, p["name"], p["major"], p["minor"]))
        import numpy as np
        try:
            g = cv2.cuda_GpuMat(np.uint8(np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8)))
            cv2.cuda.cvtColor(g, cv2.COLOR_BGR2GRAY)
            print("  live test   : cv2.cuda.cvtColor OK -> CUDA features work")
        except Exception as e:
            print("  live test   : FAILED (%s)" % e)
    else:
        print("cv2.cuda      : module present but 0 CUDA devices -> CUDA features NOT usable")
        if not cuda_built:
            print("                (build info has no CUDA section: OpenCV is built WITHOUT CUDA)")
else:
    print("cv2.cuda      : NO (module missing)")

# OpenCL
ocl_ok = False
if hasattr(cv2, "ocl") and cv2.ocl.haveOpenCL():
    ocl_ok = True
    try:
        d = cv2.ocl.Device.getDefault()
        t = d.type()
        tname = {1: "GPU", 2: "CPU", 4: "Accelerator(GPU)"}.get(t, "type %d" % t)
        print("OpenCL        : available, default device: %s (%s)" % (d.name(), tname))
        print("  OpenCL ver  : %s, compute units: %d" % (d.version(), d.maxComputeUnits()))
    except Exception as e:
        print("OpenCL        : available, device query failed (%s)" % e)
    import numpy as np
    try:
        u = cv2.UMat(np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8))
        cv2.cvtColor(u, cv2.COLOR_BGR2GRAY)
        print("  live test   : UMat cvtColor OK -> OpenCL code path works")
    except Exception as e:
        print("  live test   : FAILED (%s)" % e)
        ocl_ok = False
else:
    print("OpenCL        : not available (CPU-only paths)")

print("\n-- performance comparison (4K image, 30x GaussianBlur 101x101) --")
if ocl_ok:
    import numpy as np
    a = np.random.randint(0, 255, (2160, 3840, 3), dtype=np.uint8)
    u = cv2.cvtColor(cv2.UMat(a), cv2.COLOR_BGR2GRAY)
    cv2.ocl.setUseOpenCL(True)
    t0 = time.time()
    for _ in range(30):
        u = cv2.GaussianBlur(u, (101, 101), 31)
    ocl_t = time.time() - t0
    m = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
    t0 = time.time()
    for _ in range(30):
        m = cv2.GaussianBlur(m, (101, 101), 31)
    cpu_t = time.time() - t0
    print("OpenCL (UMat): %.2fs   CPU (Mat): %.2fs   -> GPU %s on this workload"
          % (ocl_t, cpu_t, "faster" if ocl_t < cpu_t else "not faster (overhead dominates for small ops)"))
else:
    print("skipped (OpenCL not available)")
PY

section "Does the project actually use the GPU?"
echo "The project's Python nodes call the standard Mat API (cv2.resize, cvtColor, ...)."
echo "With this OpenCV build (OpenCL yes / CUDA no) that is CPU-only by default."
echo "A node would use the GPU only if it switched to UMat and called cv2.ocl.setUseOpenCL(True)."
