#include "app_state.hpp"

#include <algorithm>
#include <exception>
#include <system_error>

#include "vessel_model/vessel_files.hpp"

namespace playback2d {

namespace {

// Size of the shape drawn when a vessel has no model_2d.svg. Matches
// vessel::VesselParams in src/dynamics.hpp.
constexpr double kDefaultLength = 70.0;  // [m]
constexpr double kDefaultBreadth = 10.0;  // [m]

}  // namespace

bool AppState::Load(const std::filesystem::path& path) {
    if (!session.Open(path)) return false;
    LoadModel();
    fit_requested = true;
    was_following = false;  // re-apply the follow zoom after the fit
    return true;
}

void AppState::LoadModel() {
    model = vessel_model::DefaultModel2D(kDefaultLength, kDefaultBreadth);
    if (session.VesselDir().empty()) return;  // the session has already said so

    const auto file = session.VesselDir() / vessel_model::kModel2dFile;
    std::error_code ec;
    if (!std::filesystem::exists(file, ec)) {
        session.AddNote("Vessel '" + session.VesselName() + "' has no " + std::string(vessel_model::kModel2dFile) +
                        ", drawing a default shape");
        return;
    }
    try {
        model = vessel_model::LoadModel2D(file);
    } catch (const std::exception& e) {
        session.AddNote(std::string(e.what()) + ", drawing a default shape");
        return;
    }
    for (const auto& warning : model.warnings) session.AddNote(model.source + ": " + warning);
}

double AppState::ShipLength() const { return model.Empty() ? kDefaultLength : model.Length(); }

void AppState::FitTrack() {
    if (!session.Loaded()) return;
    const auto& rec = session.GetRecording();
    const auto& xs = *rec.Column(col::kX);
    const auto& ys = *rec.Column(col::kY);
    const auto [n_min, n_max] = std::minmax_element(xs.begin(), xs.end());
    const auto [e_min, e_max] = std::minmax_element(ys.begin(), ys.end());
    // Pad by a ship length so the hull is not cut at the edges
    const double pad = ShipLength();
    camera.Fit({*n_min - pad, *e_min - pad}, {*n_max + pad, *e_max + pad}, 20.0);
}

}  // namespace playback2d
