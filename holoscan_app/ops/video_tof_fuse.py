import os, json
from holoscan.decorator import create_op, Input, Output

try:
    import cupy as cp
    HAS_CUPY = True
except Exception:
    import numpy as cp  # type: ignore
    HAS_CUPY = False

def _load_calib(path: str):
    if path and os.path.exists(path):
        with open(path, 'r') as f: return json.load(f)
    return None

_CUDA_SRC = r"""
extern "C" __global__
void project_overlay(
    const float* depth, int Hd, int Wd,
    unsigned char* rgb, int Hr, int Wr,
    const float* Kd, const float* Kr,
    const float* T,
    unsigned int* zbuf,
    int splat_radius,
    float z_min, float z_max, float alpha)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (x >= Wd || y >= Hd) return;
    int idx = y * Wd + x;
    float z = depth[idx];
    if (!(z > 0.0f) || z < z_min || z > z_max) return;

    float fx_d = Kd[0], fy_d = Kd[4], cx_d = Kd[2], cy_d = Kd[5];
    float Xd = ((float)x - cx_d) * z / fx_d;
    float Yd = ((float)y - cy_d) * z / fy_d;
    float Zd = z;

    float Xr = T[0]*Xd + T[1]*Yd + T[2]*Zd + T[3];
    float Yr = T[4]*Xd + T[5]*Yd + T[6]*Zd + T[7];
    float Zr = T[8]*Xd + T[9]*Yd + T[10]*Zd + T[11];
    if (Zr <= 1e-6f) return;

    float fx_r = Kr[0], fy_r = Kr[4], cx_r = Kr[2], cy_r = Kr[5];
    int u0 = (int)roundf(fx_r * (Xr / Zr) + cx_r);
    int v0 = (int)roundf(fy_r * (Yr / Zr) + cy_r);

    float t = (z - z_min) / (z_max - z_min);
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    unsigned char rc = (unsigned char)(255.0f * t);
    unsigned char gc = (unsigned char)(255.0f * (1.0f - fabsf(t - 0.5f) * 2.0f));
    unsigned char bc = (unsigned char)(255.0f * (1.0f - t));

    int r = splat_radius;
    for (int dv = -r; dv <= r; ++dv) {
        for (int du = -r; du <= r; ++du) {
            int u = u0 + du, v = v0 + dv;
            if (u < 0 || u >= Wr || v < 0 || v >= Hr) continue;
            int pi = v * Wr + u;
            int rgb_idx = pi * 3;

            float z_prev = __uint_as_float(zbuf[pi]);
            if (Zr <= z_prev) {
                zbuf[pi] = __float_as_uint(Zr);
                unsigned char r0 = rgb[rgb_idx + 0];
                unsigned char g0 = rgb[rgb_idx + 1];
                unsigned char b0 = rgb[rgb_idx + 2];
                float a = alpha;
                rgb[rgb_idx + 0] = (unsigned char)(a * rc + (1.0f - a) * r0);
                rgb[rgb_idx + 1] = (unsigned char)(a * gc + (1.0f - a) * g0);
                rgb[rgb_idx + 2] = (unsigned char)(a * bc + (1.0f - a) * b0);
            }
        }
    }
}
""";

_raw = cp.RawKernel(_CUDA_SRC, "project_overlay") if HAS_CUPY else None

@create_op("VideoTofFuseOp")
def video_tof_fuse(bundle: Input,
                   calibration_path: str = "holoscan_app/config/calibration.json",
                   z_min: float = 0.3, z_max: float = 2.0, alpha: float = 0.6,
                   splat_radius: int = 1, conf_min: int = 0) -> Output:
    if bundle is None or 'img' not in bundle:
        return None

    img = bundle['img']; depth = bundle.get('depth', None); meta = dict(bundle.get('meta', {}))

    K_rgb = meta.get('K_rgb'); K_tof = meta.get('K_tof'); T = meta.get('T_rgb_tof')
    if K_rgb is None or K_tof is None or T is None:
        calib = _load_calib(calibration_path)
        if calib: K_rgb, K_tof, T = calib['K_rgb'], calib['K_tof'], calib['T_rgb_tof']

    if depth is None or K_rgb is None or K_tof is None or T is None or not HAS_CUPY:
        return {'ts': bundle.get('ts', 0), 'img': img, 'meta': meta}

    if not isinstance(img, cp.ndarray):   img   = cp.asarray(img, dtype=cp.uint8)
    if not isinstance(depth, cp.ndarray): depth = cp.asarray(depth, dtype=cp.float32)
    Kr = cp.asarray(K_rgb, dtype=cp.float32); Kd = cp.asarray(K_tof, dtype=cp.float32); Tm = cp.asarray(T, dtype=cp.float32)

    # Optional confidence gating passed via meta['conf'] (uint8)
    if 'conf' in meta and conf_min > 0:
        C = cp.asarray(meta['conf'], dtype=cp.uint8)
        depth = cp.where(C >= int(conf_min), depth, cp.zeros_like(depth))

    Hr, Wr = img.shape[:2]; Hd, Wd = depth.shape[:2]
    zbuf = cp.full((Hr*Wr,), cp.float32(cp.inf)).view(cp.uint32)
    block = (16,16,1); grid = ((Wd + 15)//16, (Hd + 15)//16, 1)

    _raw(grid, block, (depth, Hd, Wd, img, Hr, Wr, Kd, Kr, Tm, zbuf,
                       int(splat_radius), float(z_min), float(z_max), float(alpha)))
    meta['depth_attached'] = True
    return {'ts': bundle.get('ts', 0), 'img': img, 'meta': meta}
