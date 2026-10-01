#include "recording.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace playback2d {

namespace {

std::string Trim(std::string_view s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) return {};
    const auto last = s.find_last_not_of(" \t\r\n");
    return std::string(s.substr(first, last - first + 1));
}

std::vector<std::string> SplitHeader(const std::string& line) {
    std::vector<std::string> names;
    std::size_t start = 0;
    while (true) {
        const auto comma = line.find(',', start);
        names.push_back(Trim(std::string_view(line).substr(start, comma - start)));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return names;
}

// Parses exactly `count` comma separated numbers. False on any malformed field.
bool ParseRow(const std::string& line, std::size_t count, std::vector<double>& out) {
    out.clear();
    const char* p = line.c_str();
    for (std::size_t i = 0; i < count; i++) {
        char* end = nullptr;
        errno = 0;
        const double value = std::strtod(p, &end);
        if (end == p || errno == ERANGE) return false;
        out.push_back(value);
        while (*end == ' ' || *end == '\t') end++;
        const bool last = (i + 1 == count);
        if (last) {
            while (*end == '\r' || *end == '\n') end++;
            return *end == '\0';
        }
        if (*end != ',') return false;
        p = end + 1;
    }
    return false;
}

}  // namespace

double WrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

double Frame::Get(std::string_view column, double fallback) const {
    if (recording_ == nullptr) return fallback;
    const auto i = recording_->ColumnIndex(column);
    return i ? values_[*i] : fallback;
}

bool Frame::Has(std::string_view column) const {
    return recording_ != nullptr && recording_->Has(column);
}

// ---------------------------------------------------------------------------
// Recording
// ---------------------------------------------------------------------------

Recording Recording::LoadCsv(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Could not open " + path.string());
    return ParseCsv(file, path.filename().string());
}

Recording Recording::ParseCsv(std::istream& in, const std::string& source_name) {
    Recording rec;
    rec.source_name_ = source_name;

    std::string line;
    if (!std::getline(in, line)) throw std::runtime_error(source_name + " is empty");
    rec.names_ = SplitHeader(line);

    for (std::size_t i = 0; i < rec.names_.size(); i++) {
        const auto& name = rec.names_[i];
        if (name.empty()) throw std::runtime_error(source_name + ": empty column name in header");
        if (!rec.index_.emplace(name, i).second) {
            throw std::runtime_error(source_name + ": duplicate column '" + name + "'");
        }
        rec.interp_.push_back(DefaultInterp(name));
    }

    const auto t_col = rec.ColumnIndex(col::kT);
    if (!t_col) throw std::runtime_error(source_name + ": no time column 't'");
    rec.time_column_ = *t_col;

    const std::size_t n_cols = rec.names_.size();
    rec.columns_.assign(n_cols, {});
    std::vector<double> row;
    while (std::getline(in, line)) {
        if (Trim(line).empty()) continue;
        // Skip malformed rows, and rows whose time does not increase,
        // so a partly written file still plays
        const bool ok = ParseRow(line, n_cols, row) && std::isfinite(row[*t_col]) &&
                        (rec.columns_[*t_col].empty() || row[*t_col] > rec.columns_[*t_col].back());
        if (!ok) {
            rec.skipped_rows_++;
            continue;
        }
        for (std::size_t c = 0; c < n_cols; c++) rec.columns_[c].push_back(row[c]);
    }

    if (rec.columns_[*t_col].empty()) throw std::runtime_error(source_name + " has no data rows");
    return rec;
}

std::optional<std::size_t> Recording::ColumnIndex(std::string_view column) const {
    const auto it = index_.find(column);
    if (it == index_.end()) return std::nullopt;
    return it->second;
}

const std::vector<double>* Recording::Column(std::string_view column) const {
    const auto i = ColumnIndex(column);
    return i ? &columns_[*i] : nullptr;
}

std::vector<std::string_view> Recording::MissingRequired() const {
    std::vector<std::string_view> missing;
    for (const auto name : kRequiredColumns) {
        if (!Has(name)) missing.push_back(name);
    }
    return missing;
}

std::size_t Recording::IndexAt(double t) const {
    const auto& time = Time();
    const auto it = std::upper_bound(time.begin(), time.end(), t);
    if (it == time.begin()) return 0;
    return static_cast<std::size_t>(it - time.begin()) - 1;
}

Frame Recording::Sample(double t) const {
    const auto& time = Time();
    t = std::clamp(t, StartTime(), EndTime());

    Frame frame;
    frame.recording_ = this;
    frame.t_ = t;
    frame.index_ = IndexAt(t);

    const std::size_t i = frame.index_;
    const std::size_t j = std::min(i + 1, Rows() - 1);
    const double span = time[j] - time[i];
    const double a = span > 0.0 ? (t - time[i]) / span : 0.0;

    frame.values_.resize(columns_.size());
    for (std::size_t c = 0; c < columns_.size(); c++) {
        const double v0 = columns_[c][i];
        const double v1 = columns_[c][j];
        switch (interp_[c]) {
            case Interp::Linear: frame.values_[c] = v0 + a * (v1 - v0); break;
            case Interp::Angle: frame.values_[c] = WrapAngle(v0 + a * WrapAngle(v1 - v0)); break;
            case Interp::Hold: frame.values_[c] = v0; break;
        }
    }
    frame.values_[time_column_] = t;
    return frame;
}

}  // namespace playback2d
