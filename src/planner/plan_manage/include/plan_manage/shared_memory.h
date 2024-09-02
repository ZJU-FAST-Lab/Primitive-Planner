#include <sys/ipc.h>
#include <sys/shm.h>
#include <vector>
#include <utility>
#include <iostream>
#include <cstring> // 用于memset

#include <traj_utils/plan_container.hpp>

// 附加共享内存到进程地址空间
void* attach_shared_memory(int shm_id) {
    void* shared_memory = shmat(shm_id, nullptr, 0);
    if (shared_memory == (void*)-1) {
        perror("shmat");
        exit(1);
    }
    return shared_memory;
}

// 从进程地址空间分离共享内存
void detach_shared_memory(void* shared_memory) {
    if (shmdt(shared_memory) == -1) {
        perror("shmdt");
        exit(1);
    }
}

// 删除共享内存
void remove_shared_memory(int shm_id) {
    // 检查共享内存是否存在
    struct shmid_ds shminfo;
    if (shmctl(shm_id, IPC_STAT, &shminfo) == -1) {
        if (errno == EIDRM) {
            std::cerr << "Shared memory already removed or never existed." << std::endl;
        } else {
            std::cerr << "Error checking shared memory status: " << strerror(errno) << std::endl;
        }
        return;
    }

    // 确认共享内存存在后，执行删除操作
    if (shmctl(shm_id, IPC_RMID, nullptr) == -1) {
        std::cerr << "Failed to remove shared memory: " << strerror(errno) << std::endl;
    } else {
        std::cout << "Shared memory removed successfully." << std::endl;
    }
}

int get_shared_memory_key(const char* filename, char proj_id) {
    key_t key = ftok(filename, proj_id);
    if (key == -1) {
        perror("ftok");
        exit(1);
    }
    return key;
}

int get_or_create_shared_memory(size_t size) {
    int key = get_shared_memory_key("/etc/apt/sources.list", 'a');
    int shm_id = shmget(key, 0, 0666);
    if (shm_id == -1) {
        // 如果共享内存不存在，创建它
        shm_id = shmget(key, size, IPC_CREAT | 0666);
        if (shm_id < 0) {
            perror("shmget");
            exit(1);
        }
    }
    return shm_id;
}
