#include <condition_variable>
#include <damiao_can/can/socket/damiao_can_group.hpp>
#include <exception>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

namespace damiao_can::can::socket {

struct DamiaoCANGroup::Worker {
    explicit Worker(std::unique_ptr<DamiaoCAN> device_) : device(std::move(device_)) {}

    ~Worker() { stop_and_join(); }

    Worker(const Worker&) = delete;
    Worker& operator=(const Worker&) = delete;
    Worker(Worker&&) = delete;
    Worker& operator=(Worker&&) = delete;

    void start() {
        thread = std::thread([this]() { run(); });
    }

    void request_stop() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
            request = false;
        }

        cv.notify_all();
    }

    void join() noexcept {
        if (thread.joinable()) {
            thread.join();
        }
    }

    void stop_and_join() noexcept {
        request_stop();
        join();
    }

    void request_recv_all(int new_timeout_us) {
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (stop) {
                throw std::runtime_error("DamiaoCANGroup worker is stopped");
            }

            timeout_us = new_timeout_us;
            done = false;
            request = true;
        }

        cv.notify_one();
    }

    DamiaoCANRecvResult wait_result() {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this]() { return done || stop; });

        if (!done && stop) {
            DamiaoCANRecvResult stopped_result;
            stopped_result.interface = device ? device->can_interface() : "";
            stopped_result.ok = false;
            stopped_result.error = "DamiaoCANGroup worker stopped before completing operation";
            return stopped_result;
        }

        return result;
    }

    void run() noexcept {
        while (true) {
            int local_timeout_us = 0;

            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [this]() { return stop || request; });

                if (stop) {
                    return;
                }

                local_timeout_us = timeout_us;
                request = false;
            }

            DamiaoCANRecvResult local_result = execute_recv_all(local_timeout_us);

            {
                std::lock_guard<std::mutex> lock(mutex);
                result = std::move(local_result);
                done = true;
            }

            cv.notify_all();
        }
    }

    DamiaoCANRecvResult execute_recv_all(int local_timeout_us) noexcept {
        DamiaoCANRecvResult local_result;
        local_result.interface = device->can_interface();

        try {
            local_result.expected = device->expected_response_count();
            local_result.received = device->recv_all(local_timeout_us);
            local_result.ok = (local_result.received == local_result.expected);
        } catch (const std::exception& e) {
            local_result.ok = false;
            local_result.error = e.what();
        } catch (...) {
            local_result.ok = false;
            local_result.error = "Unknown exception in DamiaoCANGroup worker";
        }

        return local_result;
    }

    std::unique_ptr<DamiaoCAN> device;
    std::thread thread;
    std::mutex mutex;
    std::condition_variable cv;

    bool stop = false;
    bool request = false;
    bool done = true;

    int timeout_us = 500;
    DamiaoCANRecvResult result;
};

DamiaoCANGroup::DamiaoCANGroup(const std::vector<std::string>& can_interfaces, bool enable_fd) {
    std::set<std::string> seen_interfaces;

    for (const auto& can_interface : can_interfaces) {
        if (!seen_interfaces.insert(can_interface).second) {
            throw std::invalid_argument("Duplicate CAN interface in DamiaoCANGroup: " +
                                        can_interface);
        }
    }

    workers_.reserve(can_interfaces.size());

    for (const auto& can_interface : can_interfaces) {
        workers_.push_back(
            std::make_unique<Worker>(std::make_unique<DamiaoCAN>(can_interface, enable_fd)));
    }

    for (auto& worker : workers_) {
        worker->start();
    }
}

DamiaoCANGroup::~DamiaoCANGroup() {
    for (auto& worker : workers_) {
        worker->request_stop();
    }

    for (auto& worker : workers_) {
        worker->join();
    }
}

DamiaoCAN& DamiaoCANGroup::get_device(std::size_t index) {
    std::lock_guard<std::mutex> lock(api_mutex_);

    if (index >= workers_.size()) {
        throw std::out_of_range("DamiaoCANGroup index out of range");
    }

    return *workers_[index]->device;
}

const DamiaoCAN& DamiaoCANGroup::get_device(std::size_t index) const {
    std::lock_guard<std::mutex> lock(api_mutex_);

    if (index >= workers_.size()) {
        throw std::out_of_range("DamiaoCANGroup index out of range");
    }

    return *workers_[index]->device;
}

DamiaoCAN& DamiaoCANGroup::get_device(const std::string& can_interface) {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        if (worker->device->can_interface() == can_interface) {
            return *worker->device;
        }
    }

    throw std::out_of_range("CAN interface not found in DamiaoCANGroup: " + can_interface);
}

const DamiaoCAN& DamiaoCANGroup::get_device(const std::string& can_interface) const {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (const auto& worker : workers_) {
        if (worker->device->can_interface() == can_interface) {
            return *worker->device;
        }
    }

    throw std::out_of_range("CAN interface not found in DamiaoCANGroup: " + can_interface);
}

void DamiaoCANGroup::enable_all() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->device->enable_all();
    }
}

void DamiaoCANGroup::disable_all() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->device->disable_all();
    }
}

void DamiaoCANGroup::set_zero_all() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->device->set_zero_all();
    }
}

void DamiaoCANGroup::flush_rx() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->device->flush_rx();
    }
}

void DamiaoCANGroup::refresh_all() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->device->refresh_all();
    }
}

std::vector<DamiaoCANRecvResult> DamiaoCANGroup::recv_all(int timeout_us) {
    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->request_recv_all(timeout_us);
    }

    std::vector<DamiaoCANRecvResult> results;
    results.reserve(workers_.size());

    for (auto& worker : workers_) {
        results.push_back(worker->wait_result());
    }

    return results;
}

}  // namespace damiao_can::can::socket