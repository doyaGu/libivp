/* Optional, intrusive work counters. Never link into timing binaries. */
#ifndef BENCH_WORK_H
#define BENCH_WORK_H
#ifdef __cplusplus
extern "C" {
#endif
void bench_work_reset(void);
void bench_work_register(const void *queue, int kind); /* 1=hull, 2=time */
void bench_work_start(void);
void bench_work_add(const void *queue, unsigned population);
void bench_work_remove(const void *queue);
void bench_work_lcs(int dimension, int initial_actives);
void bench_work_step(void);
void bench_work_finish(void); /* JSON to stderr after stopping collection */
#ifdef __cplusplus
}
#endif
#endif
