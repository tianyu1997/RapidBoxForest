#include <SBF/sbf.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Args {
    int nx = 28;
    int ny = 18;
    int nz = 3;
    double tolerance = 1e-9;
    double gap_tolerance = 0.0;
    std::string out_json;
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto require_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
            return argv[++i];
        };
        if (key == "--nx") {
            args.nx = std::stoi(require_value("--nx"));
        } else if (key == "--ny") {
            args.ny = std::stoi(require_value("--ny"));
        } else if (key == "--nz") {
            args.nz = std::stoi(require_value("--nz"));
        } else if (key == "--tolerance") {
            args.tolerance = std::stod(require_value("--tolerance"));
        } else if (key == "--gap-tolerance") {
            args.gap_tolerance = std::stod(require_value("--gap-tolerance"));
        } else if (key == "--out-json") {
            args.out_json = require_value("--out-json");
        } else if (key == "--help" || key == "-h") {
            std::cout << "Usage: tro2026_graph_equivalence_benchmark [--nx N] [--ny N] [--nz N] [--out-json PATH]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + key);
        }
    }
    return args;
}

std::vector<rbf::BoxNode> make_boxes(const Args& args) {
    std::vector<rbf::BoxNode> boxes;
    boxes.reserve(static_cast<std::size_t>(args.nx * args.ny * args.nz));
    int id = 0;
    for (int ix = 0; ix < args.nx; ++ix) {
        for (int iy = 0; iy < args.ny; ++iy) {
            for (int iz = 0; iz < args.nz; ++iz) {
                const double jitter = 1e-5 * static_cast<double>((17 * ix + 31 * iy + 13 * iz) % 11);
                rbf::BoxNode box;
                box.id = id++;
                box.joint_intervals = {
                    {0.21 * ix, 0.21 * ix + 0.21 + jitter},
                    {0.19 * iy, 0.19 * iy + 0.19},
                    {0.17 * iz, 0.17 * iz + 0.17},
                    {-0.2, 0.2 + 0.001 * ((ix + iy + iz) % 5)},
                };
                box.compute_volume();
                boxes.push_back(box);
            }
        }
    }
    return boxes;
}

std::set<std::pair<int, int>> edge_set(const rbf::AdjacencyGraph& graph) {
    std::set<std::pair<int, int>> edges;
    for (const auto& [id, neighbors] : graph) {
        for (int neighbor : neighbors) {
            edges.insert({std::min(id, neighbor), std::max(id, neighbor)});
        }
    }
    return edges;
}

template <typename Fn>
auto timed(Fn&& fn) {
    const auto t0 = std::chrono::steady_clock::now();
    auto value = fn();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return std::pair{std::move(value), ms};
}

std::string json_payload(int n_boxes,
                         std::size_t reference_edges,
                         std::size_t indexed_edges,
                         double reference_ms,
                         double indexed_ms,
                         bool equivalent) {
    const double speedup = indexed_ms > 0.0 ? reference_ms / indexed_ms : 0.0;
    return std::string("{\n") +
        "  \"experiment\": \"graph_equivalence_benchmark\",\n" +
        "  \"n_boxes\": " + std::to_string(n_boxes) + ",\n" +
        "  \"reference_edges\": " + std::to_string(reference_edges) + ",\n" +
        "  \"indexed_edges\": " + std::to_string(indexed_edges) + ",\n" +
        "  \"reference_ms\": " + std::to_string(reference_ms) + ",\n" +
        "  \"indexed_ms\": " + std::to_string(indexed_ms) + ",\n" +
        "  \"speedup\": " + std::to_string(speedup) + ",\n" +
        "  \"equivalent\": " + std::string(equivalent ? "true" : "false") + "\n" +
        "}\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        const auto boxes = make_boxes(args);
        const auto [reference_graph, reference_ms] = timed([&]() {
            return rbf::compute_adjacency_reference(boxes, args.tolerance, 0, args.gap_tolerance);
        });
        const auto [indexed_graph, indexed_ms] = timed([&]() {
            return rbf::compute_adjacency(boxes, args.tolerance, 0, args.gap_tolerance);
        });
        const auto reference_edges = edge_set(reference_graph);
        const auto indexed_edges = edge_set(indexed_graph);
        const bool equivalent = reference_edges == indexed_edges;
        const std::string payload = json_payload(static_cast<int>(boxes.size()),
                                                 reference_edges.size(),
                                                 indexed_edges.size(),
                                                 reference_ms,
                                                 indexed_ms,
                                                 equivalent);
        if (!args.out_json.empty()) {
            std::ofstream out(args.out_json);
            out << payload;
        }
        std::cout << payload;
        return equivalent ? 0 : 2;
    } catch (const std::exception& exc) {
        std::cerr << exc.what() << "\n";
        return 1;
    }
}
