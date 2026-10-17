#include "vessel_model/vessel_files.hpp"

#include <algorithm>
#include <system_error>

namespace fs = std::filesystem;

namespace vessel_model {

fs::path VesselDir(const fs::path& project_root, std::string_view name) {
    return project_root / kVesselsDir / std::string(name);
}

std::vector<std::string> ListVessels(const fs::path& project_root) {
    std::vector<std::string> names;
    std::error_code ec;
    for (fs::directory_iterator it(project_root / kVesselsDir, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code dir_ec;
        if (it->is_directory(dir_ec)) names.push_back(it->path().filename().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}

}  // namespace vessel_model
