#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>  // For benchmark
#include "max30102.h"

// Analysis: Thread creation race - Use barrier to sync creation, avoid race on shared fd by mutex.
// Benchmark: Measure creation time.

#define NUM_MAP_THREADS 4  // For map-reduce sample

pthread_barrier_t creation_barrier;
pthread_mutex_t race_mutex = PTHREAD_MUTEX_INITIALIZER;

struct thread_arg {
    int fd;
    int type;  // 0: red, 1: ir, 2: temp
    uint32_t *data;  // For map-reduce
    int len;
};

void *read_red_thread(void *arg) {
    struct thread_arg *targ = (struct thread_arg *)arg;
    struct max30102_fifo_data data;
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);  // Benchmark start

    pthread_barrier_wait(&creation_barrier);  // Sync to avoid creation race

    while (1) {
        pthread_mutex_lock(&race_mutex);  // Prevent race on ioctl
        ioctl(targ->fd, MAX30102_IOC_READ_FIFO, &data);
        pthread_mutex_unlock(&race_mutex);
        printf("Red thread: %d samples\n", data.len);
        usleep(500000);
    }

    clock_gettime(CLOCK_MONOTONIC, &end);  // Benchmark end
    printf("Red thread time: %ld ns\n", (end.tv_nsec - start.tv_nsec));
    return NULL;
}

void *read_ir_thread(void *arg) {
    struct thread_arg *targ = (struct thread_arg *)arg;
    struct max30102_fifo_data data;
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    pthread_barrier_wait(&creation_barrier);

    while (1) {
        pthread_mutex_lock(&race_mutex);
        ioctl(targ->fd, MAX30102_IOC_READ_FIFO, &data);
        pthread_mutex_unlock(&race_mutex);
        printf("IR thread: %d samples\n", data.len);
        usleep(500000);
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("IR thread time: %ld ns\n", (end.tv_nsec - start.tv_nsec));
    return NULL;
}

void *read_temp_detached(void *arg) {
    struct thread_arg *targ = (struct thread_arg *)arg;
    float temp;
    while (1) {
        pthread_mutex_lock(&race_mutex);
        ioctl(targ->fd, MAX30102_IOC_READ_TEMP, &temp);
        pthread_mutex_unlock(&race_mutex);
        printf("Temp detached: %.2f\n", temp);
        sleep(5);
    }
    return NULL;
}

// Sample Map-Reduce: Map threads process FIFO data chunks, reduce computes mean
void *map_thread(void *arg) {
    struct thread_arg *targ = (struct thread_arg *)arg;
    // Process chunk: e.g., sum data
    uint32_t local_sum = 0;
    for (int i = 0; i < targ->len; i++) local_sum += targ->data[i];
    return (void*)(intptr_t)local_sum;
}

uint32_t map_reduce(uint32_t *data, int total_len) {
    pthread_t threads[NUM_MAP_THREADS];
    struct thread_arg args[NUM_MAP_THREADS];
    int chunk = total_len / NUM_MAP_THREADS;
    for (int i = 0; i < NUM_MAP_THREADS; i++) {
        args[i].data = data + i * chunk;
        args[i].len = (i == NUM_MAP_THREADS - 1) ? total_len - i * chunk : chunk;
        pthread_create(&threads[i], NULL, map_thread, &args[i]);
    }
    uint32_t total = 0;
    for (int i = 0; i < NUM_MAP_THREADS; i++) {
        void *res;
        pthread_join(threads[i], &res);
        total += (intptr_t)res;
    }
    return total / total_len;  // Reduce: mean
    // Theory: Map parallel process, reduce aggregate. Scalable on multi-core.
}

int main() {
    int fd = open("/dev/max30102", O_RDWR);
    if (fd < 0) return 1;

    pthread_barrier_init(&creation_barrier, NULL, 3);  // 3 threads

    pthread_t red_tid, ir_tid, temp_tid;
    struct thread_arg arg = {fd, 0};

    // Custom stack: Analysis - Prevent overflow for large data
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 1024 * 1024);  // 1MB

    // Scheduling: Real-time FIFO, high prio for sensor data
    struct sched_param param;
    param.sched_priority = 50;
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);

    pthread_create(&red_tid, &attr, read_red_thread, &arg);
    pthread_create(&ir_tid, &attr, read_ir_thread, &arg);

    // Detached: No join needed, self-terminate
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&temp_tid, &attr, read_temp_detached, &arg);

    // Map-reduce example with FIFO data
    struct max30102_fifo_data fifo;
    ioctl(fd, MAX30102_IOC_READ_FIFO, &fifo);
    uint32_t mean = map_reduce(fifo.red, fifo.len);
    printf("Map-reduce mean: %u\n", mean);

    pthread_join(red_tid, NULL);
    pthread_join(ir_tid, NULL);

    pthread_barrier_destroy(&creation_barrier);
    close(fd);
    return 0;
}