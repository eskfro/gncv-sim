#pragma once
// ============================================================================
// What a playback app has open: the list of runs, the loaded run (csv +
// metadata), which vessel it is, and the playback clock. GUI free, shared by
// playback_2d and playback_3d. Each app loads its own vessel model from
// VesselDir() whenever LoadId() changes.
// ============================================================================
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "playback/playback_clock.hpp"
#include "simdata/recording.hpp"
#include "simdata/run.hpp"

namespace playback {

class Session {
public:
    // Where vessels/ is. Empty: vessels are never found.
    void SetProjectRoot(std::filesystem::path root) { project_root_ = std::move(root); }
    // Where runs are listed (normally <root>/simdata)
    void SetSimdataDir(std::filesystem::path dir) { simdata_dir_ = std::move(dir); }
    // Use this vessel for every run instead of the one in metadata. "" = off.
    void SetVesselOverride(std::string name) { vessel_override_ = std::move(name); }

    const std::filesystem::path& ProjectRoot() const { return project_root_; }
    const std::filesystem::path& SimdataDir() const { return simdata_dir_; }

    // Re-read the run list from SimdataDir()
    void Refresh();
    const std::vector<simdata::Run>& Runs() const { return runs_; }

    // Opens a run folder, its metadata.json or a csv, and rewinds the clock.
    // On failure the current run stays loaded and Error() says why.
    bool Open(const std::filesystem::path& path);

    bool Loaded() const { return recording_.has_value(); }
    // Only valid when Loaded()
    const simdata::Recording& GetRecording() const { return *recording_; }
    const simdata::Run& GetRun() const { return run_; }
    // nullptr for runs without (readable) metadata.json
    const simdata::Metadata* GetMetadata() const { return metadata_ ? &*metadata_ : nullptr; }
    bool IsCurrent(const simdata::Run& run) const { return Loaded() && run.data_file == run_.data_file; }

    // Vessel of the loaded run: override, then metadata, then the default
    const std::string& VesselName() const { return vessel_name_; }
    // vessels/<VesselName()>, empty when that folder does not exist
    const std::filesystem::path& VesselDir() const { return vessel_dir_; }

    // Changes on every successful Open()
    std::uint64_t LoadId() const { return load_id_; }
    // Why the last Open() failed, empty after a success
    const std::string& Error() const { return error_; }
    // Things worth knowing about the loaded run that did not stop it loading
    const std::vector<std::string>& Notes() const { return notes_; }
    void AddNote(std::string note) { notes_.push_back(std::move(note)); }

    // Guidance mode names, from metadata when it has them
    std::vector<std::string> GuidanceModes() const;

    PlaybackClock clock;

private:
    std::filesystem::path project_root_;
    std::filesystem::path simdata_dir_;
    std::string vessel_override_;
    std::vector<simdata::Run> runs_;

    simdata::Run run_;
    std::optional<simdata::Recording> recording_;
    std::optional<simdata::Metadata> metadata_;
    std::string vessel_name_;
    std::filesystem::path vessel_dir_;
    std::uint64_t load_id_{0};
    std::string error_;
    std::vector<std::string> notes_;
};

}  // namespace playback
