#pragma once

#include <stdint.h>
#include <stddef.h>

namespace RobofestDrone {

struct SchedulerTask {
    const char* name = nullptr;
    uint32_t period_ms = 0;
    uint32_t last_run_ms = 0;
    bool enabled = true;
    void (*task_function)(uint32_t now_ms) = nullptr;
};

class Scheduler {
public:
    static constexpr size_t MAX_TASKS = 16;

    Scheduler();

    void init();
    bool registerTask(const char* name, uint32_t period_ms, void (*task_function)(uint32_t now_ms), bool enabled = true);
    void run(uint32_t now_ms);
    bool isTaskDue(size_t index, uint32_t now_ms) const;
    bool enableTask(size_t index);
    bool disableTask(size_t index);
    bool enableTask(const char* name);
    bool disableTask(const char* name);

    size_t getTaskCount() const { return m_task_count; }
    const SchedulerTask* getTask(size_t index) const;

private:
    SchedulerTask m_tasks[MAX_TASKS];
    size_t m_task_count = 0;
};

} // namespace RobofestDrone
