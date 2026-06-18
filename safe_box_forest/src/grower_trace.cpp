#include <SBF/grower.h>

#include <filesystem>
#include <iomanip>
#include <ostream>
#include <string>
#include <string_view>

namespace rbf {
namespace {

void write_json_string(std::ostream& out, std::string_view value) {
    out << '"';
    for (char ch : value) {
        switch (ch) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\b': out << "\\b"; break;
            case '\f': out << "\\f"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20) {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(ch))
                        << std::dec << std::setfill(' ');
                } else {
                    out << ch;
                }
        }
    }
    out << '"';
}

void write_json_vector(std::ostream& out, const Eigen::Ref<const Eigen::VectorXd>& vector) {
    out << '[';
    for (int index = 0; index < vector.size(); ++index) {
        if (index > 0) out << ',';
        out << std::setprecision(17) << vector[index];
    }
    out << ']';
}

void write_json_intervals(std::ostream& out, const std::vector<Interval>& intervals) {
    out << '[';
    for (std::size_t index = 0; index < intervals.size(); ++index) {
        if (index > 0) out << ',';
        out << '[' << std::setprecision(17) << intervals[index].lo << ',' << intervals[index].hi << ']';
    }
    out << ']';
}

void write_json_face(std::ostream& out, const GrowTraceFace& face) {
    out << '{'
        << "\"valid\":" << (face.valid ? "true" : "false")
        << ",\"rank\":" << face.rank
        << ",\"selected\":" << (face.selected ? "true" : "false")
        << ",\"seed_covered\":" << (face.seed_covered ? "true" : "false")
        << ",\"parent_index\":" << face.parent_index
        << ",\"parent_box_id\":" << face.parent_box_id
        << ",\"dim\":" << face.dim
        << ",\"side\":" << face.side
        << ",\"face_value\":" << std::setprecision(17) << face.face_value
        << ",\"score\":" << std::setprecision(17) << face.score
        << ",\"scanned_boxes\":" << face.scanned_boxes
        << ",\"scanned_faces\":" << face.scanned_faces
        << '}';
}

void write_json_faces(std::ostream& out, const std::vector<GrowTraceFace>& faces) {
    out << '[';
    for (std::size_t index = 0; index < faces.size(); ++index) {
        if (index > 0) out << ',';
        write_json_face(out, faces[index]);
    }
    out << ']';
}

const GrowTraceFace* trace_face_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr && worker_result->selected_face.valid) {
        return &worker_result->selected_face;
    }
    if (task != nullptr && task->selected_face.valid) {
        return &task->selected_face;
    }
    return nullptr;
}

int trace_task_id_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return worker_result->task_id;
    return task != nullptr ? task->task_id : -1;
}

int trace_iteration_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return worker_result->iteration;
    return task != nullptr ? task->iteration : -1;
}

int trace_target_root_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return worker_result->target_root_id;
    return task != nullptr ? task->target_root_id : -1;
}

const char* trace_target_type_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return grow_target_type_str(worker_result->target_type);
    return task != nullptr ? grow_target_type_str(task->target_type) : "";
}

const Eigen::VectorXd* trace_target_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr && worker_result->target.size() > 0) return &worker_result->target;
    if (task != nullptr && task->target.size() > 0) return &task->target;
    return nullptr;
}

}  // namespace

void RrtGrower::open_trace() {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    trace_event_count_ = 0;
    trace_opened_ = false;
    if (trace_file_.is_open()) {
        trace_file_.close();
    }
    if (!config_.trace_enabled || config_.trace_path.empty()) {
        return;
    }
    const std::filesystem::path path(config_.trace_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    trace_file_.open(path, std::ios::out | std::ios::trunc);
    trace_opened_ = trace_file_.is_open();
}

void RrtGrower::close_trace() {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    if (trace_file_.is_open()) {
        trace_file_.flush();
        trace_file_.close();
    }
    trace_opened_ = false;
}

bool RrtGrower::trace_enabled() const {
    return config_.trace_enabled && trace_opened_ &&
           (config_.trace_max_events <= 0 ||
            trace_event_count_ < static_cast<std::uint64_t>(config_.trace_max_events));
}

void RrtGrower::write_trace_event(const std::string& event,
                                  const std::function<void(std::ostream&)>& write_fields) const {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    if (!config_.trace_enabled || !trace_opened_ || !trace_file_.is_open()) {
        return;
    }
    if (config_.trace_max_events > 0 &&
        trace_event_count_ >= static_cast<std::uint64_t>(config_.trace_max_events)) {
        return;
    }
    trace_file_ << "{\"event_index\":" << trace_event_count_
                << ",\"event\":";
    write_json_string(trace_file_, event);
    write_fields(trace_file_);
    trace_file_ << "}\n";
    trace_event_count_ += 1;
}

void RrtGrower::trace_root_seed(int iteration,
                                int root_id,
                                const Eigen::Ref<const Eigen::VectorXd>& seed) const {
    write_trace_event("root_seed", [&](std::ostream& out) {
        out << ",\"iteration\":" << iteration
            << ",\"worker_id\":-1"
            << ",\"root_id\":" << root_id
            << ",\"seed\":";
        write_json_vector(out, seed);
    });
}

void RrtGrower::trace_task_plan(const GrowTask& task) const {
    write_trace_event("rrt_sampled_target", [&](std::ostream& out) {
        out << ",\"task_id\":" << task.task_id
            << ",\"iteration\":" << task.iteration
            << ",\"worker_id\":-1"
            << ",\"source_root_id\":" << task.source_root_id
            << ",\"root_id\":" << task.root_id
            << ",\"target_root_id\":" << task.target_root_id
            << ",\"intertree_goal_bias\":" << (task.intertree_goal_bias ? "true" : "false")
            << ",\"component_connect_target\":" << (task.component_connect_target ? "true" : "false")
            << ",\"component_pair_unknown_failures\":" << task.component_pair_unknown_failures
            << ",\"component_connect_staged_target\":" << (task.component_connect_staged_target ? "true" : "false")
            << ",\"component_connect_gap_sq\":" << std::setprecision(17) << task.component_connect_gap_sq
            << ",\"target_type\":";
        write_json_string(out, grow_target_type_str(task.target_type));
        out << ",\"target\":";
        if (task.target.size() > 0) {
            write_json_vector(out, task.target);
        } else {
            out << "[]";
        }
    });
    if (task.selected_face.valid) {
        if (!task.face_candidates.empty()) {
            write_trace_event("nearest_face_candidates", [&](std::ostream& out) {
                out << ",\"task_id\":" << task.task_id
                    << ",\"iteration\":" << task.iteration
                    << ",\"worker_id\":-1"
                    << ",\"selected_rank\":" << task.selected_face.rank
                    << ",\"candidate_count\":" << task.face_candidates.size()
                    << ",\"faces\":";
                write_json_faces(out, task.face_candidates);
            });
        }
        write_trace_event("nearest_face_candidate", [&](std::ostream& out) {
            out << ",\"task_id\":" << task.task_id
                << ",\"iteration\":" << task.iteration
                << ",\"worker_id\":-1"
                << ",\"selected\":true"
                << ",\"face\":";
            write_json_face(out, task.selected_face);
        });
        write_trace_event("selected_face", [&](std::ostream& out) {
            out << ",\"task_id\":" << task.task_id
                << ",\"iteration\":" << task.iteration
                << ",\"worker_id\":-1"
                << ",\"seed\":";
            write_json_vector(out, task.seed);
            out << ",\"target\":";
            if (task.target.size() > 0) {
                write_json_vector(out, task.target);
            } else {
                out << "[]";
            }
            out << ",\"face\":";
            write_json_face(out, task.selected_face);
        });
    }
    write_trace_event("rrt_seed", [&](std::ostream& out) {
        out << ",\"task_id\":" << task.task_id
            << ",\"iteration\":" << task.iteration
            << ",\"worker_id\":-1"
            << ",\"parent_box_id\":" << task.parent_box_id
            << ",\"root_id\":" << task.root_id
            << ",\"target_root_id\":" << task.target_root_id
            << ",\"component_connect_target\":" << (task.component_connect_target ? "true" : "false")
            << ",\"component_pair_unknown_failures\":" << task.component_pair_unknown_failures
            << ",\"component_connect_staged_target\":" << (task.component_connect_staged_target ? "true" : "false")
            << ",\"seed\":";
        write_json_vector(out, task.seed);
    });
}

void RrtGrower::trace_ffb_result(const std::string& event,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed,
                                 const FindFreeBoxResult& ffb_result,
                                 int parent_box_id,
                                 int root_id,
                                 const GrowTask* task,
                                 const GrowWorkerResult* worker_result,
                                 int worker_id,
                                 int ffb_depth) const {
    const bool component_connect_target = worker_result != nullptr
        ? worker_result->component_connect_target
        : (task != nullptr && task->component_connect_target);
    const int source_root_id = worker_result != nullptr
        ? worker_result->source_root_id
        : (task != nullptr ? task->source_root_id : -1);
    write_trace_event(event, [&](std::ostream& out) {
        out << ",\"task_id\":" << trace_task_id_from(task, worker_result)
            << ",\"iteration\":" << trace_iteration_from(task, worker_result)
            << ",\"worker_id\":" << worker_id
            << ",\"parent_box_id\":" << parent_box_id
            << ",\"root_id\":" << root_id
            << ",\"source_root_id\":" << source_root_id
            << ",\"target_root_id\":" << trace_target_root_from(task, worker_result)
            << ",\"ffb_depth\":" << ffb_depth
            << ",\"component_connect_target\":" << (component_connect_target ? "true" : "false")
            << ",\"component_pair_unknown_failures\":"
            << (worker_result != nullptr ? worker_result->component_pair_unknown_failures : (task != nullptr ? task->component_pair_unknown_failures : 0))
            << ",\"component_connect_staged_target\":"
            << ((worker_result != nullptr ? worker_result->component_connect_staged_target : (task != nullptr && task->component_connect_staged_target)) ? "true" : "false")
            << ",\"component_connect_gap_sq\":" << std::setprecision(17)
            << (worker_result != nullptr ? worker_result->component_connect_gap_sq : (task != nullptr ? task->component_connect_gap_sq : 0.0))
            << ",\"found\":" << (ffb_result.found ? "true" : "false")
            << ",\"node\":" << ffb_result.node
            << ",\"fail_code\":" << ffb_result.fail_code
            << ",\"splits\":" << ffb_result.splits
            << ",\"decisions\":" << ffb_result.decisions
            << ",\"seed_collision\":" << (ffb_result.seed_collision ? "true" : "false")
            << ",\"hit_reserved_depth_cap\":" << (ffb_result.hit_reserved_depth_cap ? "true" : "false")
            << ",\"hit_unknown_depth_cap\":" << (ffb_result.hit_unknown_depth_cap ? "true" : "false")
            << ",\"deadline_reached\":" << (ffb_result.deadline_reached ? "true" : "false")
            << ",\"total_ms\":" << std::setprecision(17) << ffb_result.total_ms
            << ",\"target_type\":";
        write_json_string(out, trace_target_type_from(task, worker_result));
        out << ",\"seed\":";
        write_json_vector(out, seed);
        out << ",\"target\":";
        if (const auto* target = trace_target_from(task, worker_result)) {
            write_json_vector(out, *target);
        } else {
            out << "[]";
        }
        if (!ffb_result.intervals.empty()) {
            out << ",\"intervals\":";
            write_json_intervals(out, ffb_result.intervals);
        }
        if (const auto* face = trace_face_from(task, worker_result)) {
            out << ",\"selected_face\":";
            write_json_face(out, *face);
        }
    });
}

void RrtGrower::trace_box_added(const BoxNode& box,
                                const GrowTask* task,
                                const GrowWorkerResult* worker_result,
                                int worker_id) const {
    write_trace_event("box_added", [&](std::ostream& out) {
        out << ",\"task_id\":" << trace_task_id_from(task, worker_result)
            << ",\"iteration\":" << trace_iteration_from(task, worker_result)
            << ",\"worker_id\":" << worker_id
            << ",\"box_id\":" << box.id
            << ",\"parent_box_id\":" << box.parent_box_id
            << ",\"root_id\":" << box.root_id
            << ",\"tree_id\":" << box.tree_id
            << ",\"target_root_id\":" << trace_target_root_from(task, worker_result)
            << ",\"volume\":" << std::setprecision(17) << box.volume
            << ",\"seed\":";
        write_json_vector(out, box.seed_config);
        out << ",\"intervals\":";
        write_json_intervals(out, box.joint_intervals);
        if (const auto* face = trace_face_from(task, worker_result)) {
            out << ",\"selected_face\":";
            write_json_face(out, *face);
        }
    });
}

void RrtGrower::trace_box_rejected(const std::string& reason,
                                   const Eigen::Ref<const Eigen::VectorXd>& seed,
                                   int parent_box_id,
                                   int root_id,
                                   const GrowTask* task,
                                   const GrowWorkerResult* worker_result,
                                   int worker_id,
                                   const FindFreeBoxResult* ffb_result) const {
    write_trace_event("box_rejected", [&](std::ostream& out) {
        out << ",\"task_id\":" << trace_task_id_from(task, worker_result)
            << ",\"iteration\":" << trace_iteration_from(task, worker_result)
            << ",\"worker_id\":" << worker_id
            << ",\"parent_box_id\":" << parent_box_id
            << ",\"root_id\":" << root_id
            << ",\"reason\":";
        write_json_string(out, reason);
        out << ",\"seed\":";
        write_json_vector(out, seed);
        if (ffb_result != nullptr) {
            out << ",\"ffb_node\":" << ffb_result->node
                << ",\"ffb_fail_code\":" << ffb_result->fail_code;
        }
        if (const auto* face = trace_face_from(task, worker_result)) {
            out << ",\"selected_face\":";
            write_json_face(out, *face);
        }
    });
}

}  // namespace rbf
