const uint SOLID = 0;
const uint FLUID = 1;
const uint AIR = 2;

struct ComputeUBO {
    uint size;
    uint numParticles;
    float overCellSize;
    float cellSize;
    float dt;
    float flipRatio;
    ivec3 dim;
};

struct IdxWeight {
    ivec3 gIdx;
    vec3 weight;
};

ivec3 particleToGrid(vec3 pos, vec3 shift, float overCellSize) {
    return ivec3((pos - shift)*overCellSize);
}

uint gPosToIdx(ivec3 gPos, ivec3 dim) {
    return gPos.x + dim.x * (gPos.z + gPos.y * dim.z);
}

uint particleIdx(vec3 pos, vec3 shift, float overCellSize, ivec3 dim) {
    return gPosToIdx(particleToGrid(pos,shift,overCellSize), dim);
}

IdxWeight particleIdxWeight(vec3 pos, vec3 shift, float overCellSize) {
    vec3 p = (pos - shift)*overCellSize;
    IdxWeight idxWeight;
    idxWeight.gIdx = ivec3(p);
    idxWeight.weight = p - floor(p);
    return idxWeight;
}