#include "playback/session.hpp"

#include <exception>
#include <system_error>

#include "simdata/channels.hpp"
#include "vessel_model/vessel_files.hpp"

namespace fs = std::filesystem;

namespace playback {

void Session::Refresh() { runs_ = simdata::ListRuns(simdata_dir_); }

bool Session::Open(const fs::path& path) {
    const auto run = simdata::ResolveRun(path);
    if (!run) {
        error_ = path.string() + " is not a simulation run (a run folder, metadata.json or csv file)";
        return false;
    }

    std::vector<std::string> notes;
    std::optional<simdata::Metadata> metadata;
    if (run->HasMetadata()) {
        try {
            metadata = simdata::ReadMetadata(run->dir);
        } catch (const std::exception& e) {
            notes.push_back(std::string("metadata ignored: ") + e.what());
        }
    } else {
        notes.push_back("No metadata.json (run from before run folders)");
    }

    std::optional<simdata::Recording> recording;
    try {
        recording = simdata::Recording::LoadCsv(run->data_file);
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    const auto missing = recording->MissingRequired();
    if (!missing.empty()) {
        std::string list;
        for (const auto name : missing) list += (list.empty() ? "" : ", ") + std::string(name);
        error_ = run->data_file.filename().string() + " is missing columns: " + list;
        return false;
    }
    if (recording->SkippedRows() > 0) {
        notes.push_back(std::to_string(recording->SkippedRows()) + " malformed row(s) skipped");
    }

    // Which vessel
    std::string vessel = vessel_override_;
    if (vessel.empty() && metadata) vessel = metadata->vessel;
    if (vessel.empty()) {
        vessel = vessel_model::kDefaultVessel;
        notes.push_back("Run does not name its vessel, assuming " + vessel);
    }
    fs::path vessel_dir;
    std::error_code ec;
    if (!project_root_.empty() && fs::is_directory(vessel_model::VesselDir(project_root_, vessel), ec)) {
        vessel_dir = vessel_model::VesselDir(project_root_, vessel);
    } else {
        notes.push_back("Vessel '" + vessel + "' not found in vessels/, drawing a default shape");
    }

    run_ = *run;
    recording_ = std::move(recording);
    metadata_ = std::move(metadata);
    vessel_name_ = vessel;
    vessel_dir_ = vessel_dir;
    notes_ = std::move(notes);
    error_.clear();
    load_id_++;

    clock.Reset(recording_->StartTime(), recording_->EndTime());
    clock.Play();
    return true;
}

std::vector<std::string> Session::GuidanceModes() const {
    if (metadata_) {
        const auto it = metadata_->parameters.find("guidance_modes");
        if (it != metadata_->parameters.end() && it->is_array()) {
            std::vector<std::string> modes;
            for (const auto& mode : *it) {
                if (mode.is_string()) modes.push_back(mode.get<std::string>());
            }
            if (!modes.empty()) return modes;
        }
    }
    return {simdata::kGuidanceModes.begin(), simdata::kGuidanceModes.end()};
}

}  // namespace playback
