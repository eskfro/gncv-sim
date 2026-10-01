#include "app_state.hpp"

#include <algorithm>
#include <exception>

#include "data_files.hpp"

namespace playback2d {

bool AppState::Load(const std::filesystem::path& path) {
    try {
        Recording rec = Recording::LoadCsv(path);
        const auto missing = rec.MissingRequired();
        if (!missing.empty()) {
            std::string list;
            for (const auto name : missing) list += (list.empty() ? "" : ", ") + std::string(name);
            load_error = path.filename().string() + " is missing columns: " + list;
            return false;
        }
        recording = std::move(rec);
    } catch (const std::exception& e) {
        load_error = e.what();
        return false;
    }

    load_error.clear();
    loaded_file = path;
    clock.Reset(recording->StartTime(), recording->EndTime());
    clock.Play();
    fit_requested = true;
    was_following = false;  // re-apply the follow zoom after the fit
    return true;
}

void AppState::RefreshFiles() { files = ListCsvFiles(data_dir); }

void AppState::FitTrack() {
    if (!recording) return;
    const auto& xs = *recording->Column(col::kX);
    const auto& ys = *recording->Column(col::kY);
    const auto [n_min, n_max] = std::minmax_element(xs.begin(), xs.end());
    const auto [e_min, e_max] = std::minmax_element(ys.begin(), ys.end());
    // Pad by a ship length so the hull is not cut at the edges
    const double pad = ship.length;
    camera.Fit({*n_min - pad, *e_min - pad}, {*n_max + pad, *e_max + pad}, 20.0);
}

}  // namespace playback2d
