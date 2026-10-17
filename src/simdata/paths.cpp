#include "simdata/paths.hpp"

#include <system_error>
#include <vector>

namespace fs = std::filesystem;

namespace simdata {

namespace {

constexpr int kMaxLevelsUp = 6;

fs::path SearchUpwards(fs::path dir) {
    for (int level = 0; level <= kMaxLevelsUp && !dir.empty(); level++) {
        if (IsProjectRoot(dir)) return dir.lexically_normal();
        if (dir == dir.parent_path()) break;
        dir = dir.parent_path();
    }
    return {};
}

}  // namespace

fs::path ExecutableDir() {
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    return ec ? fs::path{} : exe.parent_path();
}

bool IsProjectRoot(const fs::path& dir) {
    std::error_code ec;
    return fs::is_directory(dir / "vessels", ec) && fs::is_regular_file(dir / "CMakeLists.txt", ec);
}

fs::path FindProjectRoot() {
    std::error_code ec;
    for (const fs::path& start : {fs::current_path(ec), ExecutableDir()}) {
        if (start.empty()) continue;
        if (fs::path root = SearchUpwards(start); !root.empty()) return root;
    }
    return {};
}

fs::path SimdataDir(const fs::path& project_root) {
    return project_root.empty() ? fs::path("simdata") : project_root / "simdata";
}

}  // namespace simdata
