from holoscan.decorator import create_op, Input, Output

@create_op("TimeAlignMuxOp")
def time_align_mux(video: Input, tof: Input = None, tolerance_ns: int = 10_000_000) -> Output:
    """
    Align a video frame with the nearest ToF frame by timestamp (ns).
    Emits None if no match within tolerance. Pass-through if ToF not present.
    Expects {'ts': int, 'img': <gpu>, 'meta': {...}} and {'ts': int, 'depth': <gpu>, 'meta': {...}}
    """
    if tof is None:
        return {'ts': video.get('ts', 0), 'img': video.get('img', None), 'meta': video.get('meta', {})}
    ts_v = video.get('ts', 0); ts_t = tof.get('ts', 0)
    if abs(ts_v - ts_t) <= tolerance_ns:
        out = {'ts': max(ts_v, ts_t), 'img': video.get('img', None), 'meta': {}}
        if 'meta' in video: out['meta']['video'] = video['meta']
        if 'meta' in tof: out['meta']['tof'] = tof['meta']
        if 'depth' in tof: out['depth'] = tof['depth']
        return out
    return None
