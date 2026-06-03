#pragma once

#include <SBF/api.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace rbf {

class CancellationToken {
public:
	CancellationToken();
	explicit CancellationToken(std::shared_ptr<std::atomic<bool>> flag);

	void cancel();
	bool cancelled() const;
	void reset();
	std::shared_ptr<std::atomic<bool>> native_flag() const { return flag_; }

private:
	std::shared_ptr<std::atomic<bool>> flag_;
};

class Deadline {
public:
	using Clock = std::chrono::steady_clock;

	static Deadline after_ms(double timeout_ms);
	bool expired() const;
	double remaining_ms() const;

private:
	std::optional<Clock::time_point> deadline_;
};

class TaskExecutor {
public:
	virtual ~TaskExecutor() = default;
	virtual void parallel_for(int begin, int end, const std::function<void(int)>& fn) = 0;
	virtual int n_threads() const = 0;
};

class InlineExecutor final : public TaskExecutor {
public:
	void parallel_for(int begin, int end, const std::function<void(int)>& fn) override;
	int n_threads() const override { return 1; }
};

class ThreadPoolExecutor final : public TaskExecutor {
public:
	explicit ThreadPoolExecutor(int n_threads);
	~ThreadPoolExecutor() override;
	void parallel_for(int begin, int end, const std::function<void(int)>& fn) override;
	int n_threads() const override { return n_threads_; }

private:
	void worker_loop(int worker_id);

	int n_threads_ = 1;
	std::vector<std::thread> threads_;
	std::mutex mutex_;
	std::condition_variable work_cv_;
	std::condition_variable done_cv_;
	std::function<void(int)> fn_;
	std::atomic<int> next_{0};
	std::atomic<int> remaining_{0};
	int begin_ = 0;
	int end_ = 0;
	bool has_work_ = false;
	bool stop_ = false;
	std::exception_ptr first_error_;
};

std::shared_ptr<TaskExecutor> make_executor(const RuntimeConfig& config);
int current_worker_id();

class StageDiagnostics {
public:
	void add_counter(const std::string& key, double delta = 1.0);
	void set_value(const std::string& key, double value);
	void record_timing(const std::string& key, double elapsed_ms);
	double value(const std::string& key, double fallback = 0.0) const;
	std::unordered_map<std::string, double> snapshot() const;

private:
	mutable std::mutex mutex_;
	std::unordered_map<std::string, double> values_;
};

class ScopedStageTimer {
public:
	ScopedStageTimer(StageDiagnostics& diagnostics, std::string key);
	~ScopedStageTimer();

private:
	StageDiagnostics& diagnostics_;
	std::string key_;
	std::chrono::steady_clock::time_point start_;
};

class StageContext {
public:
	StageContext();
	explicit StageContext(RuntimeConfig config,
						  Deadline deadline = {},
						  std::shared_ptr<CancellationToken> cancellation = {},
						  std::shared_ptr<TaskExecutor> executor = {},
						  std::shared_ptr<StageDiagnostics> diagnostics = {});

	static StageContext serial();
	static StageContext from_runtime(const RuntimeConfig& config, Deadline deadline = {});

	bool should_stop() const;
	std::shared_ptr<std::atomic<bool>> native_cancel_flag() const;

	const RuntimeConfig& runtime() const { return runtime_; }
	Deadline& deadline() { return deadline_; }
	const Deadline& deadline() const { return deadline_; }
	CancellationToken& cancellation() { return *cancellation_; }
	const CancellationToken& cancellation() const { return *cancellation_; }
	TaskExecutor& executor() { return *executor_; }
	const TaskExecutor& executor() const { return *executor_; }
	StageDiagnostics& diagnostics() { return *diagnostics_; }
	const StageDiagnostics& diagnostics() const { return *diagnostics_; }

private:
	RuntimeConfig runtime_;
	Deadline deadline_;
	std::shared_ptr<CancellationToken> cancellation_;
	std::shared_ptr<TaskExecutor> executor_;
	std::shared_ptr<StageDiagnostics> diagnostics_;
};

}  // namespace rbf
