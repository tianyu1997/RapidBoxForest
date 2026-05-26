#include <SBF/runtime.h>

#include <algorithm>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace rbf {

namespace {

thread_local int g_current_worker_id = -1;

class WorkerIdScope {
public:
    explicit WorkerIdScope(int worker_id)
        : previous_(g_current_worker_id) {
        g_current_worker_id = worker_id;
    }

    ~WorkerIdScope() {
        g_current_worker_id = previous_;
    }

private:
    int previous_;
};

}  // namespace

CancellationToken::CancellationToken()
    : flag_(std::make_shared<std::atomic<bool>>(false))
{}

CancellationToken::CancellationToken(std::shared_ptr<std::atomic<bool>> flag)
    : flag_(std::move(flag))
{
    if (!flag_) {
        flag_ = std::make_shared<std::atomic<bool>>(false);
    }
}

void CancellationToken::cancel() {
    flag_->store(true, std::memory_order_relaxed);
}

bool CancellationToken::cancelled() const {
    return flag_->load(std::memory_order_relaxed);
}

void CancellationToken::reset() {
    flag_->store(false, std::memory_order_relaxed);
}

Deadline Deadline::after_ms(double timeout_ms) {
    Deadline out;
    if (timeout_ms > 0.0) {
        out.deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>(timeout_ms));
    }
    return out;
}

bool Deadline::expired() const {
    return deadline_.has_value() && Clock::now() >= *deadline_;
}

double Deadline::remaining_ms() const {
    if (!deadline_) return -1.0;
    const auto now = Clock::now();
    if (now >= *deadline_) return 0.0;
    return std::chrono::duration<double, std::milli>(*deadline_ - now).count();
}

void InlineExecutor::parallel_for(int begin,
                                  int end,
                                  const std::function<void(int)>& fn) {
    WorkerIdScope worker_scope(0);
    for (int i = begin; i < end; ++i) {
        fn(i);
    }
}

ThreadPoolExecutor::ThreadPoolExecutor(int n_threads)
    : n_threads_(std::max(1, n_threads))
{}

void ThreadPoolExecutor::parallel_for(int begin,
                                      int end,
                                      const std::function<void(int)>& fn) {
    if (end <= begin) return;
    const int n_items = end - begin;
    const int workers = std::max(1, std::min(n_threads_, n_items));
    if (workers == 1) {
        InlineExecutor inline_executor;
        inline_executor.parallel_for(begin, end, fn);
        return;
    }

    std::atomic<int> next{begin};
    std::exception_ptr first_error;
    std::mutex error_mutex;

    auto worker = [&](int worker_id) {
        WorkerIdScope worker_scope(worker_id);
        while (true) {
            const int item = next.fetch_add(1, std::memory_order_relaxed);
            if (item >= end) break;
            try {
                fn(item);
            } catch (...) {
                std::lock_guard<std::mutex> lock(error_mutex);
                if (!first_error) first_error = std::current_exception();
            }
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(static_cast<std::size_t>(workers));
    for (int worker_id = 0; worker_id < workers; ++worker_id) {
        threads.emplace_back(worker, worker_id);
    }
    for (auto& thread : threads) {
        thread.join();
    }
    if (first_error) std::rethrow_exception(first_error);
}

std::shared_ptr<TaskExecutor> make_executor(const RuntimeConfig& config) {
    if (config.mode == ExecutionMode::Parallel && config.n_threads > 1) {
        return std::make_shared<ThreadPoolExecutor>(config.n_threads);
    }
    return std::make_shared<InlineExecutor>();
}

int current_worker_id() {
    return g_current_worker_id;
}

StageContext::StageContext()
    : StageContext(RuntimeConfig{})
{}

StageContext::StageContext(RuntimeConfig config,
                           Deadline deadline,
                           std::shared_ptr<CancellationToken> cancellation,
                           std::shared_ptr<TaskExecutor> executor,
                           std::shared_ptr<StageDiagnostics> diagnostics)
    : runtime_(config)
    , deadline_(deadline)
    , cancellation_(std::move(cancellation))
    , executor_(std::move(executor))
    , diagnostics_(std::move(diagnostics))
{
    if (!cancellation_) {
        cancellation_ = std::make_shared<CancellationToken>();
    }
    if (!executor_) {
        executor_ = make_executor(runtime_);
    }
    if (!diagnostics_) {
        diagnostics_ = std::make_shared<StageDiagnostics>();
    }
}

void StageDiagnostics::add_counter(const std::string& key, double delta) {
    std::lock_guard<std::mutex> lock(mutex_);
    values_[key] += delta;
}

void StageDiagnostics::set_value(const std::string& key, double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    values_[key] = value;
}

void StageDiagnostics::record_timing(const std::string& key, double elapsed_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string prefix = "profile." + key;
    values_[prefix + ".calls"] += 1.0;
    values_[prefix + ".total_ms"] += elapsed_ms;
    auto max_it = values_.find(prefix + ".max_ms");
    if (max_it == values_.end() || elapsed_ms > max_it->second) {
        values_[prefix + ".max_ms"] = elapsed_ms;
    }
    auto min_it = values_.find(prefix + ".min_ms");
    if (min_it == values_.end() || elapsed_ms < min_it->second) {
        values_[prefix + ".min_ms"] = elapsed_ms;
    }
}

double StageDiagnostics::value(const std::string& key, double fallback) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = values_.find(key);
    return it == values_.end() ? fallback : it->second;
}

std::unordered_map<std::string, double> StageDiagnostics::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return values_;
}

ScopedStageTimer::ScopedStageTimer(StageDiagnostics& diagnostics, std::string key)
    : diagnostics_(diagnostics)
    , key_(std::move(key))
    , start_(std::chrono::steady_clock::now())
{}

ScopedStageTimer::~ScopedStageTimer() {
    const auto stop = std::chrono::steady_clock::now();
    diagnostics_.record_timing(key_, std::chrono::duration<double, std::milli>(stop - start_).count());
}

StageContext StageContext::serial() {
    return StageContext(RuntimeConfig{});
}

StageContext StageContext::from_runtime(const RuntimeConfig& config,
                                        Deadline deadline) {
    return StageContext(config, deadline);
}

bool StageContext::should_stop() const {
    return cancellation_->cancelled() || deadline_.expired();
}

std::shared_ptr<std::atomic<bool>> StageContext::native_cancel_flag() const {
    return cancellation_->native_flag();
}

}  // namespace rbf