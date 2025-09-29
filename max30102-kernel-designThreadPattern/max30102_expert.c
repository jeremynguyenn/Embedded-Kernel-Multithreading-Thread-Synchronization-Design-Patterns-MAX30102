#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include "max30102.h"

pthread_cond_t pause_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t pause_mutex = PTHREAD_MUTEX_INITIALIZER;
int paused = 0;

// Event pair: Two condvars for sync
pthread_cond_t event1 = PTHREAD_COND_INITIALIZER;
pthread_cond_t event2 = PTHREAD_COND_INITIALIZER;

// Fork handler
void prepare_fork(void) { /* Lock all */ pthread_mutex_lock(&pause_mutex); }
void parent_fork(void) { /* Unlock */ pthread_mutex_unlock(&pause_mutex); }
void child_fork(void) { close(0); /* Close fds */ }  // Error-prone: Avoid fd leak

void *paused_thread(void *arg) {
    pthread_mutex_lock(&pause_mutex);
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    ts.tv_sec += 5;  // Bounded waiting
    while (paused) {
        if (pthread_cond_timedwait(&pause_cond, &pause_mutex, &ts) == ETIMEDOUT) {
            printf("Bounded wait timeout\n");
            break;
        }
    }
    pthread_mutex_unlock(&pause_mutex);
    printf("Thread resumed\n");
    return NULL;
}

int main() {
    int fd = open("/dev/max30102", O_RDWR);
    if (fd < 0) return 1;

    pthread_t t;
    pthread_create(&t, NULL, paused_thread, NULL);

    paused = 1;
    sleep(2);
    paused = 0;
    pthread_cond_signal(&pause_cond);

    // Event pair: Signal event1, wait event2
    pthread_cond_signal(&event1);
    pthread_cond_wait(&event2, &pause_mutex);

    // Fork multi-threaded
    pthread_atfork(prepare_fork, parent_fork, child_fork);  // Handle
    pid_t pid = fork();
    if (pid == 0) {
        // Child: Threads continue, but fds closed to avoid errors
    }

    // Analysis: Fork in multi-threaded error-prone (e.g., locked mutex in child) - atfork mitigates.
    // Benchmark: Measure fork time in threaded env.
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    fork();  // Simulate
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Fork benchmark: %ld ns\n", end.tv_nsec - start.tv_nsec);

    pthread_join(t, NULL);
    close(fd);
    return 0;
}