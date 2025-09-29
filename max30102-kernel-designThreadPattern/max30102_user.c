#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <pthread.h>
#include <mqueue.h>
#include <sys/wait.h>
#include <string.h>
#include <poll.h>  // Added for poll()
#include <sys/mman.h>  // For shared memory
#include <sys/stat.h>  // For FIFO
#include <sys/types.h>  // For pipe
#include "max30102.h"

#define SHM_NAME "/max30102_shm"
#define SHM_SIZE 1024
#define FIFO_NAME "/tmp/max30102_fifo"

static int fd = -1;
static volatile sig_atomic_t running = 1;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
mqd_t mq;  // Message queue for IPC
int pipe_fd[2];  // Pipe for IPC
int shm_fd;  // Shared memory FD
void *shm_ptr;  // Shared memory pointer

// Signal handler with default/ignore demonstration
static void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        running = 0;
        printf("Signal %d received, stopping...\n", sig);
    } else if (sig == SIGUSR1) {
        printf("SIGUSR1 received, custom action.\n");
    }
}

// Poll function
static unsigned int max30102_poll(struct file *file, struct pollfd *pfd)
{
    struct max30102_data *data = file->private_data;
    unsigned int revents = 0;

    if (!data) return -EINVAL;

    if (data->fifo_full)
        revents |= POLLIN | POLLRDNORM;

    return revents;
}

// Thread function to read FIFO continuously (joinable thread)
void *fifo_thread(void *arg) {
    struct max30102_fifo_data fifo_data;
    char buf[256];
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    pthread_t tid = pthread_self();  // Thread ID
    printf("FIFO thread ID: %lu\n", (unsigned long)tid);

    while (running) {
        int poll_ret = poll(&pfd, 1, 1000);
        if (poll_ret < 0) {
            perror("Poll failed");
            break;
        } else if (poll_ret > 0 && (pfd.revents & POLLIN)) {
            int ret = pthread_mutex_lock(&mutex);
            if (ret != 0) {
                perror("Mutex lock failed");
                break;
            }
            ret = ioctl(fd, MAX30102_IOC_READ_FIFO, &fifo_data);
            if (ret < 0) {
                perror("Failed to read FIFO data");
                pthread_mutex_unlock(&mutex);
                break;
            }
            sprintf(buf, "FIFO: %d samples", fifo_data.len);
            // IPC: Send to message queue
            ret = mq_send(mq, buf, strlen(buf) + 1, 0);
            if (ret < 0) {
                perror("mq_send failed");
            }
            // IPC: Write to pipe
            ret = write(pipe_fd[1], buf, strlen(buf) + 1);
            if (ret < 0) {
                perror("Pipe write failed");
            }
            // IPC: Write to shared memory
            memcpy(shm_ptr, buf, strlen(buf) + 1);
            pthread_cond_signal(&cond);
            pthread_mutex_unlock(&mutex);
        }
        usleep(100000);  // Reduced sleep for better responsiveness
    }
    return NULL;
}

// Detached thread function to read temperature
void *temp_thread(void *arg) {
    float temp;
    static int static_var = 0;
    int auto_var = 0;
    pthread_t tid = pthread_self();  // Thread ID
    printf("Temp thread ID: %lu\n", (unsigned long)tid);

    while (running) {
        int ret = pthread_mutex_lock(&mutex);
        if (ret != 0) {
            perror("Mutex lock failed");
            break;
        }
        ret = ioctl(fd, MAX30102_IOC_READ_TEMP, &temp);
        if (ret < 0) {
            perror("Failed to read temperature");
            pthread_mutex_unlock(&mutex);
            break;
        }
        auto_var++;
        static_var++;
        printf("Temp: %.4f°C, Auto: %d, Static: %d\n", temp, auto_var, static_var);
        pthread_mutex_unlock(&mutex);
        sleep(5);
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    // Command line arguments (Process management)
    if (argc > 1) {
        printf("Command line arg: %s\n", argv[1]);
    }

    // Open device with error check
    fd = open("/dev/max30102", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    // Signal setup (Signals)
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) < 0 || sigaction(SIGTERM, &sa, NULL) < 0 || sigaction(SIGUSR1, &sa, NULL) < 0) {
        perror("sigaction");
        close(fd);
        return 1;
    }

    // Message queue setup (IPC: POSIX Message Queue)
    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = 256 };
    mq = mq_open("/max30102_mq", O_CREAT | O_RDWR, 0666, &attr);
    if (mq == (mqd_t)-1) {
        perror("mq_open");
        close(fd);
        return 1;
    }

    // Pipe setup (IPC: Pipes)
    if (pipe(pipe_fd) < 0) {
        perror("pipe");
        mq_close(mq);
        mq_unlink("/max30102_mq");
        close(fd);
        return 1;
    }

    // FIFO setup (IPC: FIFO)
    mkfifo(FIFO_NAME, 0666);
    int fifo_fd = open(FIFO_NAME, O_WRONLY | O_NONBLOCK);
    if (fifo_fd < 0) {
        perror("open FIFO");
    }

    // Shared memory setup (IPC: POSIX Shared Memory)
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        perror("shm_open");
    } else {
        if (ftruncate(shm_fd, SHM_SIZE) < 0) {
            perror("ftruncate");
        }
        shm_ptr = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_ptr == MAP_FAILED) {
            perror("mmap");
            shm_ptr = NULL;
        }
    }

    // Config with error check
    uint8_t mode = MAX30102_MODE_SPO2;
    struct max30102_slot_config slot_config = { .slot = 1, .led = 2 };
    uint8_t fifo_config = MAX30102_FIFO_SMP_AVE_8;
    uint8_t spo2_config = MAX30102_SPO2_CONFIG_DEFAULT;

    if (ioctl(fd, MAX30102_IOC_SET_FIFO_CONFIG, &fifo_config) < 0 ||
        ioctl(fd, MAX30102_IOC_SET_SPO2_CONFIG, &spo2_config) < 0 ||
        ioctl(fd, MAX30102_IOC_SET_MODE, &mode) < 0 ||
        ioctl(fd, MAX30102_IOC_SET_SLOT, &slot_config) < 0) {
        perror("Config ioctl failed");
        goto cleanup;
    }

    // Threads (POSIX Threads: joinable and detachable)
    pthread_t fifo_tid, temp_tid;
    pthread_attr_t attr_detach;
    pthread_attr_init(&attr_detach);
    pthread_attr_setdetachstate(&attr_detach, PTHREAD_CREATE_DETACHED);

    int ret = pthread_create(&fifo_tid, NULL, fifo_thread, NULL);
    if (ret != 0) {
        perror("pthread_create fifo");
        goto cleanup;
    }
    ret = pthread_create(&temp_tid, &attr_detach, temp_thread, NULL);
    if (ret != 0) {
        perror("pthread_create temp");
        goto cleanup;
    }

    // Fork demo (Process Management: creation, termination, parent-child)
    pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        goto cleanup;
    } else if (pid == 0) {
        // Child process
        printf("Child PID: %d, Parent PID: %d\n", getpid(), getppid());
        char *args[] = {"echo", "Hello from exec", argv[1] ? argv[1] : "default", NULL};  // Command line args
        execvp("echo", args);
        perror("execvp");
        exit(1);
    } else {
        // Parent process
        printf("Parent PID: %d waiting for child %d\n", getpid(), pid);
        int status;
        if (wait(&status) < 0) {
            perror("wait");
        }
        if (WIFEXITED(status)) {
            printf("Child exited with status %d\n", WEXITSTATUS(status));
        }
    }

    // Main loop: Receive from queue (IPC)
    char buf[256];
    while (running) {
        int ret = pthread_mutex_lock(&mutex);
        if (ret != 0) {
            perror("Mutex lock failed");
            break;
        }
        ret = pthread_cond_wait(&cond, &mutex);
        if (ret != 0) {
            perror("Cond wait failed");
            pthread_mutex_unlock(&mutex);
            break;
        }
        // Read from message queue
        ssize_t mq_ret = mq_receive(mq, buf, 256, NULL);
        if (mq_ret < 0) {
            perror("mq_receive");
        } else {
            printf("Received from queue: %s\n", buf);
        }
        // Read from pipe
        ssize_t pipe_ret = read(pipe_fd[0], buf, 256);
        if (pipe_ret < 0) {
            perror("pipe read");
        } else {
            printf("Received from pipe: %s\n", buf);
        }
        // Read from shared memory
        if (shm_ptr) {
            printf("Received from shm: %s\n", (char *)shm_ptr);
        }
        // Write to FIFO
        if (fifo_fd > 0) {
            write(fifo_fd, buf, strlen(buf) + 1);
        }
        pthread_mutex_unlock(&mutex);
    }

cleanup:
    // Cleanup
    pthread_join(fifo_tid, NULL);  // Join joinable thread
    // Detached thread self-terminates
    if (fifo_fd > 0) close(fifo_fd);
    unlink(FIFO_NAME);
    if (shm_ptr) munmap(shm_ptr, SHM_SIZE);
    if (shm_fd > 0) close(shm_fd);
    shm_unlink(SHM_NAME);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    mq_close(mq);
    mq_unlink("/max30102_mq");
    close(fd);
    return 0;
}