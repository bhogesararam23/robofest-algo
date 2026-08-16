#include "scheduler.h"
#include <cstring>

namespace RobofestDrone {

Scheduler::Scheduler()
    : m_task_count(0) {
}

void Scheduler::init() {
    m_task_count = 0;
    for (size_t i = 0; i < MAX_TASKS; ++i) {
        m_tasks[i] = SchedulerTask();
    }
}

bool Scheduler::registerTask(const char* name, uint32_t period_ms, void (*task_function)(uint32_t now_ms), bool enabled) {
    if (m_task_count >= MAX_TASKS || task_function == nullptr) {
        return false;
    }

    m_tasks[m_task_count].name = name;
    m_tasks[m_task_count].period_ms = period_ms;
    m_tasks[m_task_count].last_run_ms = 0;
    m_tasks[m_task_count].enabled = enabled;
    m_tasks[m_task_count].task_function = task_function;
    m_task_count++;
    return true;
}

bool Scheduler::isTaskDue(size_t index, uint32_t now_ms) const {
    if (index >= m_task_count || !m_tasks[index].enabled || m_tasks[index].task_function == nullptr) {
        return false;
    }

    // Unsigned subtraction safely handles uint32_t millis() rollover
    return ((now_ms - m_tasks[index].last_run_ms) >= m_tasks[index].period_ms);
}

void Scheduler::run(uint32_t now_ms) {
    for (size_t i = 0; i < m_task_count; ++i) {
        if (isTaskDue(i, now_ms)) {
            m_tasks[i].last_run_ms = now_ms;
            m_tasks[i].task_function(now_ms);
        }
    }
}

bool Scheduler::enableTask(size_t index) {
    if (index >= m_task_count) {
        return false;
    }
    m_tasks[index].enabled = true;
    return true;
}

bool Scheduler::disableTask(size_t index) {
    if (index >= m_task_count) {
        return false;
    }
    m_tasks[index].enabled = false;
    return true;
}

bool Scheduler::enableTask(const char* name) {
    if (name == nullptr) return false;
    for (size_t i = 0; i < m_task_count; ++i) {
        if (m_tasks[i].name != nullptr && std::strcmp(m_tasks[i].name, name) == 0) {
            m_tasks[i].enabled = true;
            return true;
        }
    }
    return false;
}

bool Scheduler::disableTask(const char* name) {
    if (name == nullptr) return false;
    for (size_t i = 0; i < m_task_count; ++i) {
        if (m_tasks[i].name != nullptr && std::strcmp(m_tasks[i].name, name) == 0) {
            m_tasks[i].enabled = false;
            return true;
        }
    }
    return false;
}

const SchedulerTask* Scheduler::getTask(size_t index) const {
    if (index >= m_task_count) {
        return nullptr;
    }
    return &m_tasks[index];
}

} // namespace RobofestDrone
