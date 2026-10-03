#include "app_state.hpp"

#include <algorithm>
#include <exception>
#include <system_error>

#include "simdata/channels.hpp"
#include "vessel_model/vessel_files.hpp"

namespace playback3d {

namespace {

namespace col = simdata::col;

// Size of the shape drawn when a vessel has no model_3d.obj. Matches
// vessel::VesselParams in src/dynamics.hpp.
constexpr double kDefaultLength = 70.0;   // [m]
constexpr double kDefaultBreadth = 10.0;  // [m]
constexpr double kDefaultDraft = 3.25;    // [m]

constexpr double kResetYaw = 0.6;         // [rad] from the heading, looking over the port quarter
constexpr double kResetPitch = 0.33;      // [rad] ~19 deg below the horizon
constexpr double kResetDistance = 2.2;    // [ship lengths]

}  // namespace

bool AppState::Load(const std::filesystem::path& path) {
    if (!session.Open(path)) return false;
    LoadModel();
    reset_requested = true;
    return true;
}

void AppState::LoadModel() {
    model = vessel_model::DefaultModel3D(kDefaultLength, kDefaultBreadth, kDefaultDraft);
    model_version++;
    if (session.VesselDir().empty()) return;  // the session has already said so

    const auto file = session.VesselDir() / vessel_model::kModel3dFile;
    std::error_code ec;
    if (!std::filesystem::exists(file, ec)) {
        session.AddNote("Vessel '" + session.VesselName() + "' has no " + std::string(vessel_model::kModel3dFile) +
                        ", drawing a default shape");
        return;
    }
    try {
        model = vessel_model::LoadModel3D(file);
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
    camera.Fit({*n_min - pad, *e_min - pad, 0.0}, {*n_max + pad, *e_max + pad, 0.0});
}

void AppState::ResetView() {
    if (!session.Loaded()) return;
    const auto frame = session.GetRecording().Sample(session.clock.Time());
    camera.SetTarget({frame.Get(col::kX), frame.Get(col::kY), 0.0});
    chase_yaw = kResetYaw;
    camera.SetYaw(frame.Get(col::kPsi) + kResetYaw);
    camera.SetPitch(kResetPitch);
    camera.SetDistance(kResetDistance * ShipLength());
}

}  // namespace playback3d
