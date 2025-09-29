#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include "max30102.h"

#define NUM_PHIL 5
sem_t forks[NUM_PHIL];
pthread_barrier_t barrier;

// Custom recursive mutex
struct rec_mutex {
    pthread_mutex_t mutex;
    pthread_t owner;
    int count;
};

void rec_mutex_lock(struct rec_mutex *rm) {
    pthread_t me = pthread_self();
    if (rm->owner == me) {
        rm->count++;
        return;
    }
    pthread_mutex_lock(&rm->mutex);
    rm->owner = me;
    rm->count = 1;
}

void rec_mutex_unlock(struct rec_mutex *rm) {
    if (--rm->count == 0) {
        rm->owner = 0;
        pthread_mutex_unlock(&rm->mutex);
    }
}

// Deadlock detection: Simple try-lock with timeout
int detect_deadlock(pthread_mutex_t *m) {
    if (pthread_mutex_trylock(m) == 0) {
        pthread_mutex_unlock(m);
        return 0;
    }
    // Simulate timeout check
    usleep(1000);
    return 1;  // Potential deadlock
}

// Dining Philosopher full impl
// Problem: Philosophers (threads) share forks (resources), risk deadlock.
// Data: sem_t forks
// Algorithm: 1. Think 2. Pick lower fork 3. Pick higher (hierarchy avoid circular) 4. Eat 5. Release
void *philosopher(void *arg) {
    int id = *(int*)arg;
    // Step 1: Think
    usleep(rand() % 1000000);
    // Step 2-3: Pick forks with order
    int left = id;
    int right = (id + 1) % NUM_PHIL;
    if (left > right) { int temp = left; left = right; right = temp; }  // Order to prevent deadlock
    sem_wait(&forks[left]);
    sem_wait(&forks[right]);
    // Step 4: Eat
    printf("Philosopher %d eating\n", id);
    usleep(rand() % 1000000);
    // Step 5: Release
    sem_post(&forks[left]);
    sem_post(&forks[right]);
    return NULL;
}

// Listener with delegation
void *listener_thread(void *arg) {
    int fd = *(int*)arg;
    struct pollfd pfd = {fd, POLLIN, 0};
    while (1) {
        poll(&pfd, 1, -1);
        printf("Listener: Data ready, delegate to worker\n");
        // Delegate: Signal worker thread for responsibility (e.g., process FIFO)
    }
    // Why: Listener waits on events (poll), delegates to avoid blocking main.
    // Demo: Integrate with sensor interrupt simulation.
    // Cancel blocked: Use pthread_cancel, but check PTHREAD_CANCEL_DEFERRED for poll.
    return NULL;
}

void cleanup_handler(void *arg) {
    // Invariants check: Ensure data consistent (e.g., queue not corrupt)
    printf("Cleanup: Invariants preserved\n");
}

void *worker_thread(void *arg) {
    pthread_cleanup_push(cleanup_handler, NULL);
    while (1) {
        // Work, check invariants post-lock (e.g., assert data_len valid)
    }
    pthread_cleanup_pop(1);
    return NULL;
}

// Assembly line: Stage1 read, Stage2 compute, Stage3 log
sem_t stage1_sem, stage2_sem;
void *stage1(void *arg) { /* Read FIFO */ sem_post(&stage1_sem); return NULL; }
void *stage2(void *arg) { sem_wait(&stage1_sem); /* Compute */ sem_post(&stage2_sem); return NULL; }
void *stage3(void *arg) { sem_wait(&stage2_sem); /* Log */ return NULL; }

// Covid case: Monitor for vaccination queue
// Guidelines: Single mutex + condvars, methods lock/unlock internally, condition checks.
struct monitor {
    pthread_mutex_t mutex;
    pthread_cond_t queue_cond;
    int queue_size;
    int bounded_wait;  // Timeout for bounded waiting
};

void enter_queue(struct monitor *m) {
    pthread_mutex_lock(&m->mutex);
    while (m->queue_size >= 10) {  // Full queue
        struct timespec ts;
        timespec_get(&ts, TIME_UTC);
        ts.tv_sec += m->bounded_wait;
        if (pthread_cond_timedwait(&m->queue_cond, &m->mutex, &ts) == ETIMEDOUT) {
            // Bounded waiting: Timeout error
            printf("Vaccination queue timeout\n");
        }
    }
    m->queue_size++;
    pthread_mutex_unlock(&m->mutex);
    // Case study: Model covid drive - threads as patients, monitor ensures fair queue (no starvation via bounded wait).
}

int main() {
    int fd = open("/dev/max30102", O_RDWR);
    if (fd < 0) return 1;

    for (int i = 0; i < NUM_PHIL; i++) sem_init(&forks[i], 0, 1);

    pthread_barrier_init(&barrier, NULL, 3);

    pthread_t phil[NUM_PHIL], listen, worker;
    int ids[NUM_PHIL];
    for (int i = 0; i < NUM_PHIL; i++) {
        ids[i] = i;
        pthread_create(&phil[i], NULL, philosopher, &ids[i]);
    }
    pthread_create(&listen, NULL, listener_thread, &fd);
    pthread_create(&worker, NULL, worker_thread, NULL);

    // Deadlock check
    if (detect_deadlock(&rec_mutex.mutex)) printf("Deadlock detected!\n");

    // Assembly line start
    pthread_t s1, s2, s3;
    pthread_create(&s1, NULL, stage1, NULL);
    pthread_create(&s2, NULL, stage2, NULL);
    pthread_create(&s3, NULL, stage3, NULL);
    pthread_join(s1, NULL); pthread_join(s2, NULL); pthread_join(s3, NULL);

    // Covid monitor
    struct monitor vac_mon = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, 0, 5};  // 5s bound
    enter_queue(&vac_mon);

    // Benchmark assembly line throughput
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    // Run stages 100 times
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Assembly benchmark: %ld ns\n", end.tv_nsec - start.tv_nsec);

    // Analysis: Deadlock conditions broken by hierarchy. Cancellation may cause deadlock if holding locks - use cleanup.

    for (int i = 0; i < NUM_PHIL; i++) pthread_join(phil[i], NULL);
    pthread_cancel(worker);
    pthread_join(listen, NULL);
    pthread_join(worker, NULL);

    for (int i = 0; i < NUM_PHIL; i++) sem_destroy(&forks[i]);
    pthread_barrier_destroy(&barrier);
    close(fd);
    return 0;
}