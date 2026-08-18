#pragma once

#include <cstddef>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "damiao_can.hpp"

namespace damiao_can::can::socket {

class DamiaoCANGroupRecvResult {
public:
    DamiaoCANGroupRecvResult() = default;
    explicit DamiaoCANGroupRecvResult(std::vector<DamiaoCANRecvResult> results);

    std::size_t size() const noexcept { return results_.size(); }

    DamiaoCANRecvResult get(std::optional<std::size_t> index = std::nullopt,
                            std::optional<std::string> can_id = std::nullopt) const;

    std::string to_string() const;

private:
    std::vector<DamiaoCANRecvResult> results_;
};

std::ostream& operator<<(std::ostream& os, const DamiaoCANGroupRecvResult& result);

class DamiaoCANGroup {
public:
    DamiaoCANGroup(const std::vector<std::string>& can_interfaces, bool enable_fd = false);
    ~DamiaoCANGroup();

    DamiaoCANGroup(const DamiaoCANGroup&) = delete;
    DamiaoCANGroup& operator=(const DamiaoCANGroup&) = delete;
    DamiaoCANGroup(DamiaoCANGroup&&) = delete;
    DamiaoCANGroup& operator=(DamiaoCANGroup&&) = delete;

    std::size_t size() const noexcept { return workers_.size(); }

    DamiaoCAN& get_device(std::size_t index);
    const DamiaoCAN& get_device(std::size_t index) const;

    DamiaoCAN& get_device(const std::string& can_interface);
    const DamiaoCAN& get_device(const std::string& can_interface) const;

    void enable_all();
    void disable_all();
    void set_zero_all();

    void flush_rx();
    void refresh_all();
    DamiaoCANGroupRecvResult recv_all(int timeout_us = 500);

private:
    struct Worker;

    std::vector<std::unique_ptr<Worker>> workers_;
    mutable std::mutex api_mutex_;
};

}  // namespace damiao_can::can::socket
