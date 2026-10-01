#include "data_files.hpp"

#include <algorithm>
#include <system_error>

namespace fs = std::filesystem;

namespace playback2d {

fs::path ExecutableDir() {
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    return ec ? fs::path{} : exe.parent_path();
}

fs::path FindDataDir(const fs::path& exe_dir) {
    std::error_code ec;
    std::vector<fs::path> candidates = {fs::current_path(ec) / kDefaultDataSubdir};

    // The binary usually lives in <root>/build, so walk a few levels up
    fs::path dir = exe_dir;
    for (int level = 0; level < 4 && !dir.empty(); level++) {
        candidates.push_back(dir / kDefaultDataSubdir);
        if (dir == dir.parent_path()) break;
        dir = dir.parent_path();
    }

    for (const auto& candidate : candidates) {
        if (fs::is_directory(candidate, ec)) return candidate.lexically_normal();
    }
    return candidates.front();
}

std::vector<fs::path> ListCsvFiles(const fs::path& dir) {
    std::vector<fs::path> files;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (it->is_regular_file(ec) && it->path().extension() == ".csv") {
            files.push_back(it->path());
        }
    }
    std::sort(files.begin(), files.end(),
              [](const fs::path& a, const fs::path& b) { return a.filename() > b.filename(); });
    return files;
}

}  // namespace playback2d
