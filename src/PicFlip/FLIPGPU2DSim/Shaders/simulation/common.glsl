#ifndef CORE_SHADER_H
#define CORE_SHADER_H

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
    ivec2 dim;
};

struct IdxWeight {
    ivec2 gIdx;
    vec2 weight;
};

ivec2 particleToGrid(vec2 pos,vec2 shift, float overCellSize) {
    return ivec2((pos - shift)*overCellSize);
}

uint gPosToIdx(ivec2 gPos, int dimX) {
    return gPos.x + gPos.y * dimX;
}

uint particleIdx(vec2 pos,vec2 shift, float overCellSize, int dimX) {
    return gPosToIdx(particleToGrid(pos,shift,overCellSize),dimX);
}

IdxWeight particleIdxWeight(vec2 pos,vec2 shift, float overCellSize, int dimX) {
    vec2 p = (pos - shift)*overCellSize;
    IdxWeight idxWeight;
    idxWeight.gIdx = ivec2(p);
    idxWeight.weight = p - floor(p);
    return idxWeight;
}

#endif //CORE_SHADER_H