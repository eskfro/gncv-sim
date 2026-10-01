#pragma once
// ============================================================================
// A simulation recording loaded from csv, and sampling of it at any time.
//
// Columns are stored by name, so the loader does not care which columns a csv
// has. Callers check Has() before using optional columns.
// ============================================================================
#include <cstddef>
#include <filesystem>
#include <functional>
#include <istream>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "channels.hpp"

namespace playback2d {

class Recording;

// All columns of a recording sampled at the same time
class Frame {
public:
    double Get(std::string_view column, double fallback = 0.0) const;
    bool Has(std::string_view column) const;
    double Time() const { return t_; }
    std::size_t Index() const { return index_; }  // last row at or before Time()

private:
    friend class Recording;
    const Recording* recording_{};
    std::vector<double> values_;
    double t_{};
    std::size_t index_{};
};

class Recording {
public:
    // Both throw std::runtime_error with a readable message on failure
    static Recording LoadCsv(const std::filesystem::path& path);
    static Recording ParseCsv(std::istream& in, const std::string& source_name);

    std::size_t Rows() const { return Time().size(); }
    double StartTime() const { return Time().front(); }
    double EndTime() const { return Time().back(); }
    double Duration() const { return EndTime() - StartTime(); }

    bool Has(std::string_view column) const { return index_.count(column) != 0; }
    std::optional<std::size_t> ColumnIndex(std::string_view column) const;
    // nullptr when the column is missing
    const std::vector<double>* Column(std::string_view column) const;
    const std::vector<std::string>& Names() const { return names_; }
    // Required columns (see channels.hpp) that this recording lacks
    std::vector<std::string_view> MissingRequired() const;

    // Last row with time <= t (0 when t is before the start)
    std::size_t IndexAt(double t) const;
    // Interpolated values at t, clamped to [StartTime, EndTime]
    Frame Sample(double t) const;

    const std::string& SourceName() const { return source_name_; }
    std::size_t SkippedRows() const { return skipped_rows_; }

private:
    Recording() = default;
    const std::vector<double>& Time() const { return columns_[time_column_]; }

    std::string source_name_;
    std::vector<std::string> names_;
    std::map<std::string, std::size_t, std::less<>> index_;
    std::vector<Interp> interp_;
    std::vector<std::vector<double>> columns_;  // columns_[column][row]
    std::size_t time_column_{};
    std::size_t skipped_rows_{};
};

// Smallest signed angle, in (-pi, pi]
double WrapAngle(double angle);

}  // namespace playback2d
