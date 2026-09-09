/*

File prng.h
-----------

This file defines a permuted congruential generator for generating random 
numbers in C. This rng is better than the libc rand(). This should only be used
for code thats intended to run on a host platform (i.e. not an MCU)

Based on https://www.pcg-random.org
Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)

*/

#ifndef PRNG_H
#define PRNG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "base/base.h"

typedef struct {
    u64 state;
    u64 increment;
} prng;

void prng_seed(prng* rng, u64 init_state, u64 init_seq);
u32 prng_rand(prng* rng);
f32 prng_rand_f32(prng* rng);
f32 prng_std_norm(prng* rng);

#ifdef __cplusplus
}
#endif

#endif // PRNG_H
