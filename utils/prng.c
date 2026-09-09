
#include "prng.h"

void prng_seed(prng* rng, u64 init_state, u64 init_seq) {
    if (rng == NULL) {
        return;
    }

    rng->state = 0;
    rng->increment = (init_seq << 1) | 1;

    prng_rand(rng);

    rng->state += init_state;

    prng_rand(rng);
}

u32 prng_rand(prng* rng) {
    if (rng == NULL) {
        return 0;
    }

    u64 old_state = rng->state;

    rng->state = old_state * 6364136223846793005ULL + rng->increment;

    u32 xorshifted = (u32)(((old_state >> 18u) ^ old_state) >> 27u);
    u32 rot = old_state >> 59u;

    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

f32 prng_rand_f32(prng* rng) {
    return ldexpf((f32)prng_rand(rng), -32);
}

// Box-Muller Transform
// https://en.wikipedia.org/wiki/Box–Muller_transform
f32 prng_std_norm(prng* rng){
    if (rng == NULL) {
        return 0;
    }

    static const f32 epsilon = 1e-6f;

    f32 u1 = epsilon;
    f32 u2 = 0.0f;

    do {
        u1 = (prng_rand_f32(rng)) * 2.0f - 1.0f;
    } while (u1 <= epsilon);
    u2 = (prng_rand_f32(rng)) * 2.0f - 1.0f;

    f32 mag = sqrtf(-2.0f * logf(u1));
    f32 z0 = mag * cosf(2.0f * 3.141592653f * u2);

    // I am ignoring the second value here
    // It might be worth trying to use it
    //f32 z1 = mag * sin(two_pi * u2);

    return z0;
}

