#include <condition_variable>
#include <damiao_can/can/socket/damiao_can_group.hpp>
#include <exception>
#include <mutex>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace damiao_can::can::socket {

DamiaoCANGroupRecvResult::DamiaoCANGroupRecvResult(std::vector<DamiaoCANRecvResult> results)
    : results_(std::move(results)) {}

DamiaoCANRecvResult DamiaoCANGroupRecvResult::get(std::optional<std::size_t> index,
                                                  std::optional<std::string> can_id) const {
    if (can_id.has_value()) {
        for (const auto& result : results_) {
            if (result.can_interface == *can_id) {
                return result;
            }
        }

        throw std::out_of_range("CAN interface not found in recv result: " + *can_id);
    }

    if (!index.has_value()) {
        throw std::invalid_argument("get() requires index or can_id");
    }

    if (*index >= results_.size()) {
        throw std::out_of_range("DamiaoCANGroupRecvResult index out of range");
    }

    return results_[*index];
}

std::string DamiaoCANGroupRecvResult::to_string() const {
    std::ostringstream os;
    os << "[";

    if (!results_.empty()) {
        os << "\n";
    }

    for (std::size_t i = 0; i < results_.size(); ++i) {
        os << "  " << results_[i];
        if (i + 1 < results_.size()) {
            os << ",";
        }
        os << "\n";
    }

    os << "]";
    return os.str();
}

std::ostream& operator<<(std::ostream& os, const DamiaoCANGroupRecvResult& result) {
    return os << result.to_string();
}

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
            error = nullptr;
            request = true;
        }

        cv.notify_one();
    }

    DamiaoCANRecvResult wait_result() {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this]() { return done || stop; });

        if (!done && stop) {
            throw std::runtime_error("DamiaoCANGroup worker stopped before completing recv_all");
        }

        if (error) {
            std::rethrow_exception(error);
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

            DamiaoCANRecvResult local_result;
            std::exception_ptr local_error;

            try {
                local_result = device->recv_all(local_timeout_us);
            } catch (...) {
                local_error = std::current_exception();
            }

            {
                std::lock_guard<std::mutex> lock(mutex);
                result = std::move(local_result);
                error = local_error;
                done = true;
            }

            cv.notify_all();
        }
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
    std::exception_ptr error;
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

DamiaoCANGroupRecvResult DamiaoCANGroup::recv_all(int timeout_us) {
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

    return DamiaoCANGroupRecvResult(std::move(results));
}

}  // namespace damiao_can::can::socket