from holoscan.decorator import create_op, Input, Output
@create_op("BiomarkerScoreOp")
def biomarker_score(frame: Input, thresholds=(0.2,0.5,0.8)) -> Output:
    meta = frame.get('meta', {}).copy(); meta['scores']={'biomarker_x':0.0,'qa_ok':True}
    return {'ts': frame.get('ts',0), 'img': frame.get('img'), 'meta': meta}
