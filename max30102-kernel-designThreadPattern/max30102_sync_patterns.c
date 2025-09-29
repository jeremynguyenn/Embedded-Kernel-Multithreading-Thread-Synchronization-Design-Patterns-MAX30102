#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include "max30102.h"

pthread_rwlock_t rwlock = PTHREAD_RWLOCK_INITIALIZER;
sem_t *inter_sem;  // Inter-process sem (named)
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mon_mutex = PTHREAD_MUTEX_INITIALIZER;

// Custom strong semaphore (FIFO): Use queue for waiting threads
struct strong_sem {
    int value;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    // Queue for FIFO (strong)
    struct queue_node *queue_head;  // Linked list for FIFO
    struct queue_node *queue_tail;
};

struct queue_node {
    pthread_t thread;
    struct queue_node *next;
};

void strong_sem_wait(struct strong_sem *s) {
    pthread_mutex_lock(&s->mutex);
    if (s->value > 0) {
        s->value--;
    } else {
        // Enqueue current thread
        struct queue_node *node = malloc(sizeof(*node));
        node->thread = pthread_self();
        node->next = NULL;
        if (s->queue_tail) s->queue_tail->next = node;
        else s->queue_head = node;
        s->queue_tail = node;
        pthread_cond_wait(&s->cond, &s->mutex);
    }
    pthread_mutex_unlock(&s->mutex);
}

void strong_sem_post(struct strong_sem *s) {
    pthread_mutex_lock(&s->mutex);
    s->value++;
    if (s->queue_head) {
        // Dequeue first (FIFO)
        struct queue_node *node = s->queue_head;
        s->queue_head = node->next;
        if (!s->queue_head) s->queue_tail = NULL;
        free(node);
        pthread_cond_signal(&s->cond);
    }
    pthread_mutex_unlock(&s->mutex);
}

// Weak sem: Standard sem_t

// Dynamic pub-sub
typedef void (*callback_t)(int);
struct subscriber {
    callback_t cb;
    struct subscriber *next;
};
struct subscriber *subs = NULL;
pthread_mutex_t sub_mutex = PTHREAD_MUTEX_INITIALIZER;

void subscribe(callback_t cb) {
    pthread_mutex_lock(&sub_mutex);
    struct subscriber *new_sub = malloc(sizeof(*new_sub));
    new_sub->cb = cb;
    new_sub->next = subs;
    subs = new_sub;
    pthread_mutex_unlock(&sub_mutex);
}

void unsubscribe(callback_t cb) {
    pthread_mutex_lock(&sub_mutex);
    struct subscriber **curr = &subs;
    while (*curr) {
        if ((*curr)->cb == cb) {
            struct subscriber *temp = *curr;
            *curr = (*curr)->next;
            free(temp);
            break;
        }
        curr = &(*curr)->next;
    }
    pthread_mutex_unlock(&sub_mutex);
}

void notify_subscribers(int value) {
    pthread_mutex_lock(&sub_mutex);
    struct subscriber *curr = subs;
    while (curr) {
        curr->cb(value);
        curr = curr->next;
    }
    pthread_mutex_unlock(&sub_mutex);
}

void subscriber_cb(int value) {
    printf("Subscriber: Received %d\n", value);
}

void *publisher_thread(void *arg) {
    notify_subscribers(42);
    return NULL;
}

void *producer(void *arg) {
    int fd = *(int*)arg;
    struct max30102_fifo_data data;
    while (1) {
        ioctl(fd, MAX30102_IOC_READ_FIFO, &data);
        pthread_mutex_lock(&mon_mutex);
        while (!data.len) {  // Spurious wake handling: Loop check condition
            pthread_cond_wait(&cond, &mon_mutex);
        }
        pthread_mutex_unlock(&mon_mutex);
        sem_post(inter_sem);  // Inter-process notify
    }
    return NULL;
}

void *consumer(void *arg) {
    while (1) {
        sem_wait(inter_sem);
        pthread_rwlock_rdlock(&rwlock);
        printf("Consumer: Data processed\n");
        pthread_rwlock_unlock(&rwlock);
    }
    return NULL;
}

int main() {
    int fd = open("/dev/max30102", O_RDWR);
    if (fd < 0) return 1;

    inter_sem = sem_open("/max_sem", O_CREAT, 0666, 0);  // Inter-process

    subscribe(subscriber_cb);

    pthread_t pub, prod, cons;
    pthread_create(&pub, NULL, publisher_thread, NULL);
    pthread_create(&prod, NULL, producer, &fd);
    pthread_create(&cons, NULL, consumer, &fd);

    // Benchmark: Measure sync overhead
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    // Simulate 1000 syncs
    for (int i = 0; i < 1000; i++) sem_post(inter_sem); sem_wait(inter_sem);
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Sync benchmark: %ld ns per op\n", (end.tv_nsec - start.tv_nsec) / 1000);

    pthread_join(pub, NULL);
    pthread_join(prod, NULL);
    pthread_join(cons, NULL);

    sem_close(inter_sem);
    sem_unlink("/max_sem");
    close(fd);
    return 0;
}