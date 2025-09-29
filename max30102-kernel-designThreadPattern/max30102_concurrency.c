#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>
#include "max30102.h"

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
sem_t alt_sem1, alt_sem2;  // For strict alternation

void *producer_thread(void *arg) {
    int fd = *(int*)arg;
    struct max30102_fifo_data data;
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        sem_wait(&alt_sem1);  // Strict alternation: producer first
        ioctl(fd, MAX30102_IOC_READ_FIFO, &data);
        pthread_mutex_lock(&mutex);
        printf("Producer: Read %d samples\n", data.len);  // Overlapping: Read while consumer processes
        pthread_mutex_unlock(&mutex);
        sem_post(&alt_sem2);
        usleep(100000);
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Producer throughput: %ld ops/sec\n", 1000000 / (end.tv_nsec - start.tv_nsec));  // Benchmark
    // Analysis: Concurrency (switching) vs Parallelism (multi-core exec). Threads light-weight than processes (less context switch).
    return NULL;
}

void *consumer_thread(void *arg) {
    while (1) {
        sem_wait(&alt_sem2);  // Alternate with producer
        pthread_mutex_lock(&mutex);
        printf("Consumer: Processing data\n");  // Non-overlapping: Locked section
        pthread_mutex_unlock(&mutex);
        sem_post(&alt_sem1);
        sleep(1);
    }
    // Theory: Singularism (single thread) slow for sensor data; concurrency overlaps I/O and compute.
    return NULL;
}

int main() {
    int fd = open("/dev/max30102", O_RDWR);
    if (fd < 0) return 1;

    sem_init(&alt_sem1, 0, 1);  // Start with producer
    sem_init(&alt_sem2, 0, 0);

    pthread_t prod, cons;
    pthread_create(&prod, NULL, producer_thread, &fd);
    pthread_create(&cons, NULL, consumer_thread, &fd);

    // Example concurrent design: Producer reads sensor, consumer computes HR (overlapping)
    // Another: Parallel threads for red/ir processing.

    pthread_join(prod, NULL);
    pthread_join(cons, NULL);

    sem_destroy(&alt_sem1);
    sem_destroy(&alt_sem2);
    close(fd);
    return 0;
}