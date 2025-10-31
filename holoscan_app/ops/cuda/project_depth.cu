
// project_depth.cu — reference signature
extern "C" __global__
void project_overlay(
    const float* depth, int Hd, int Wd,
    unsigned char* rgb, int Hr, int Wr,
    const float* Kd, const float* Kr,
    const float* T,
    unsigned int* zbuf, int splat_radius,
    float z_min, float z_max, float alpha);
