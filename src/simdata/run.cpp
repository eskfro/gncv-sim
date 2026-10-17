#include "simdata/run.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace fs = std::filesystem;
using nlohmann::json;
using nlohmann::ordered_json;

namespace simdata {

namespace {

constexpr int kMaxRunDirSuffix = 1000;

std::tm LocalTime(std::time_t when) {
    std::tm tm{};
    localtime_r(&when, &tm);
    return tm;
}

std::string FormatTime(std::time_t when, const char* format) {
    const std::tm tm = LocalTime(when);
    std::ostringstream out;
    out << std::put_time(&tm, format);
    return out.str();
}

// Folder names sort in time order. Legacy names have HH:MM:SS, so drop the
// colons to sort them among the new YYYYMMDD_HHMMSS names.
std::string SortKey(const Run& run) {
    std::string key = run.name;
    key.erase(std::remove(key.begin(), key.end(), ':'), key.end());
    return key;
}

bool IsCsv(const fs::directory_entry& entry) {
    std::error_code ec;
    return entry.is_regular_file(ec) && entry.path().extension() == ".csv";
}

Run LegacyRun(const fs::path& csv) {
    return Run{csv.stem().string(), csv.parent_path(), csv, {}};
}

}  // namespace

const Column* Metadata::FindColumn(std::string_view name) const {
    const auto it = std::find_if(columns.begin(), columns.end(),
                                 [&](const Column& c) { return c.name == name; });
    return it == columns.end() ? nullptr : &*it;
}

// ---------------------------------------------------------------------------
// metadata.json
// ---------------------------------------------------------------------------

ordered_json ToJson(const Metadata& m) {
    ordered_json columns = ordered_json::array();
    for (const auto& c : m.columns) {
        columns.push_back({{"name", c.name}, {"unit", c.unit}, {"description", c.description}});
    }
    return {
        {"format_version", m.format_version},
        {"simulator", m.simulator},
        {"vessel", m.vessel},
        {"created", m.created},
        {"dt", m.dt},
        {"duration", m.duration},
        {"samples", m.samples},
        {"data_file", m.data_file},
        {"parameters", m.parameters},
        {"columns", columns},
    };
}

Metadata MetadataFromJson(const json& j) {
    if (!j.is_object()) throw std::runtime_error("metadata is not a JSON object");
    Metadata m;
    try {
        m.format_version = j.value("format_version", m.format_version);
        if (m.format_version > kFormatVersion) {
            throw std::runtime_error("metadata format_version " + std::to_string(m.format_version) +
                                     " is newer than this program (" + std::to_string(kFormatVersion) +
                                     "), update it");
        }
        m.simulator = j.value("simulator", m.simulator);
        m.vessel = j.value("vessel", m.vessel);
        m.created = j.value("created", m.created);
        m.dt = j.value("dt", m.dt);
        m.duration = j.value("duration", m.duration);
        m.samples = j.value("samples", m.samples);
        m.data_file = j.value("data_file", m.data_file);
        if (const auto it = j.find("parameters"); it != j.end()) m.parameters = ordered_json(*it);
        if (const auto it = j.find("columns"); it != j.end()) {
            for (const auto& c : *it) {
                m.columns.push_back({c.at("name").get<std::string>(), c.value("unit", ""),
                                     c.value("description", "")});
            }
        }
    } catch (const json::exception& e) {
        throw std::runtime_error(std::string("bad metadata: ") + e.what());
    }
    return m;
}

void WriteMetadata(const fs::path& run_dir, const Metadata& metadata) {
    // Write to a temporary file and rename, so readers never see half a file
    const fs::path path = run_dir / kMetadataFile;
    const fs::path tmp = run_dir / (std::string(kMetadataFile) + ".tmp");
    {
        std::ofstream out(tmp);
        out << ToJson(metadata).dump(2) << '\n';
        if (!out) throw std::runtime_error("Could not write " + tmp.string());
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);
    if (ec) throw std::runtime_error("Could not write " + path.string() + ": " + ec.message());
}

Metadata ReadMetadata(const fs::path& run_dir) {
    const fs::path path = run_dir / kMetadataFile;
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Could not open " + path.string());
    try {
        return MetadataFromJson(json::parse(in));
    } catch (const json::exception& e) {
        throw std::runtime_error(path.string() + ": " + e.what());
    } catch (const std::runtime_error& e) {
        throw std::runtime_error(path.string() + ": " + e.what());
    }
}

// ---------------------------------------------------------------------------
// Run folders
// ---------------------------------------------------------------------------

std::string IsoTimestamp(std::time_t when) { return FormatTime(when, "%Y-%m-%dT%H:%M:%S%z"); }

std::string RunName(std::string_view simulator, std::time_t when) {
    return FormatTime(when, "%Y%m%d_%H%M%S") + "_" + std::string(simulator);
}

fs::path CreateRunDir(const fs::path& simdata_dir, std::string_view simulator, std::time_t when) {
    std::error_code ec;
    fs::create_directories(simdata_dir, ec);
    if (ec) throw std::runtime_error("Could not create " + simdata_dir.string() + ": " + ec.message());

    const std::string base = RunName(simulator, when);
    for (int n = 1; n <= kMaxRunDirSuffix; n++) {
        const fs::path dir = simdata_dir / (n == 1 ? base : base + "_" + std::to_string(n));
        if (fs::create_directory(dir, ec)) return dir;
        if (ec) throw std::runtime_error("Could not create " + dir.string() + ": " + ec.message());
    }
    throw std::runtime_error("Too many runs named " + base + " in " + simdata_dir.string());
}

bool IsRunDir(const fs::path& dir) {
    std::error_code ec;
    return fs::is_regular_file(dir / kMetadataFile, ec) || fs::is_regular_file(dir / kDataFile, ec);
}

std::optional<Run> ResolveRun(const fs::path& path) {
    std::error_code ec;
    if (fs::is_directory(path, ec)) {
        if (!IsRunDir(path)) return std::nullopt;
        const fs::path dir = path.lexically_normal();
        Run run{dir.filename().string(), dir, dir / kDataFile, {}};
        if (dir.filename().empty()) run.name = dir.parent_path().filename().string();  // "run/"
        if (fs::is_regular_file(dir / kMetadataFile, ec)) {
            run.metadata_file = dir / kMetadataFile;
            try {
                // filename(): the csv always lives in the run folder
                const fs::path named = fs::path(ReadMetadata(dir).data_file).filename();
                if (!named.empty()) run.data_file = dir / named;
            } catch (const std::runtime_error&) {
                // Unreadable metadata: still list the run, loading it reports the error
            }
        }
        return run;
    }
    if (!fs::is_regular_file(path, ec)) return std::nullopt;

    if (path.filename() == kMetadataFile) return ResolveRun(path.parent_path().empty() ? "." : path.parent_path());
    if (path.extension() != ".csv") return std::nullopt;

    const fs::path parent = path.parent_path().empty() ? fs::path(".") : path.parent_path();
    if (fs::is_regular_file(parent / kMetadataFile, ec)) {
        auto run = ResolveRun(parent);
        if (run) run->data_file = path;
        return run;
    }
    return LegacyRun(path);
}

std::vector<Run> ListRuns(const fs::path& simdata_dir) {
    std::vector<Run> runs;
    std::error_code ec;
    for (fs::directory_iterator it(simdata_dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (IsCsv(*it)) {
            runs.push_back(LegacyRun(it->path()));
        } else if (std::error_code dir_ec; it->is_directory(dir_ec)) {
            if (auto run = ResolveRun(it->path())) {
                runs.push_back(std::move(*run));
                continue;
            }
            // A folder of legacy csv files, e.g. simdata/simulator_v1/
            std::error_code sub_ec;
            for (fs::directory_iterator sub(it->path(), sub_ec); !sub_ec && sub != end; sub.increment(sub_ec)) {
                if (IsCsv(*sub)) runs.push_back(LegacyRun(sub->path()));
            }
        }
    }
    std::sort(runs.begin(), runs.end(), [](const Run& a, const Run& b) {
        const std::string ka = SortKey(a);
        const std::string kb = SortKey(b);
        return ka != kb ? ka > kb : a.data_file > b.data_file;
    });
    return runs;
}

// ---------------------------------------------------------------------------
// RunWriter
// ---------------------------------------------------------------------------

RunWriter::RunWriter(const fs::path& simdata_dir, Metadata metadata, int precision)
    : metadata_(std::move(metadata)) {
    if (metadata_.columns.empty()) throw std::invalid_argument("RunWriter: no columns");
    if (metadata_.simulator.empty()) throw std::invalid_argument("RunWriter: no simulator name");
    if (metadata_.data_file.empty()) metadata_.data_file = kDataFile;

    const std::time_t now = std::time(nullptr);
    if (metadata_.created.empty()) metadata_.created = IsoTimestamp(now);
    dir_ = CreateRunDir(simdata_dir, metadata_.simulator, now);

    const fs::path csv_path = dir_ / metadata_.data_file;
    csv_.open(csv_path);
    if (!csv_) throw std::runtime_error("Could not open " + csv_path.string());
    for (std::size_t i = 0; i < metadata_.columns.size(); i++) {
        csv_ << (i == 0 ? "" : ",") << metadata_.columns[i].name;
    }
    csv_ << '\n' << std::setprecision(precision);
}

void RunWriter::WriteRow(const std::vector<double>& row) {
    if (finished_) throw std::logic_error("RunWriter: WriteRow after Finish");
    if (row.size() != metadata_.columns.size()) {
        throw std::invalid_argument("RunWriter: row has " + std::to_string(row.size()) + " values, expected " +
                                    std::to_string(metadata_.columns.size()));
    }
    for (std::size_t i = 0; i < row.size(); i++) csv_ << (i == 0 ? "" : ",") << row[i];
    csv_ << '\n';

    if (!first_t_) first_t_ = row.front();
    last_t_ = row.front();
    metadata_.samples++;
}

void RunWriter::Finish() {
    if (finished_) return;
    finished_ = true;
    csv_.close();
    if (!csv_) throw std::runtime_error("Could not write " + (dir_ / metadata_.data_file).string());
    metadata_.duration = first_t_ ? last_t_ - *first_t_ : 0.0;
    WriteMetadata(dir_, metadata_);
}

}  // namespace simdata
