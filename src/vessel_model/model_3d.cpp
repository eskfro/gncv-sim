#include "vessel_model/model_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace vessel_model {

namespace {

constexpr Color kDefaultColor{200, 200, 200, 255};

Vec3 Sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

std::optional<Vec3> Normalized(const Vec3& v) {
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (!(n > 1e-12) || !std::isfinite(n)) return std::nullopt;
    return Vec3{v.x / n, v.y / n, v.z / n};
}

std::uint8_t Channel(double unit) {
    return static_cast<std::uint8_t>(std::lround(255.0 * std::clamp(unit, 0.0, 1.0)));
}

std::string RestOfLine(std::istringstream& in) {
    std::string rest;
    std::getline(in >> std::ws, rest);
    while (!rest.empty() && (rest.back() == '\r' || rest.back() == ' ' || rest.back() == '\t')) rest.pop_back();
    return rest;
}

// OBJ indices are 1-based, negative counts back from the latest element
bool ResolveIndex(long index, std::size_t count, std::size_t& out) {
    if (index > 0 && static_cast<std::size_t>(index) <= count) {
        out = static_cast<std::size_t>(index - 1);
        return true;
    }
    if (index < 0 && static_cast<std::size_t>(-index) <= count) {
        out = count - static_cast<std::size_t>(-index);
        return true;
    }
    return false;
}

struct FaceVertex {
    std::size_t v;
    std::optional<std::size_t> vn;
};

bool ParseIndex(const std::string& text, std::size_t count, std::size_t& out) {
    char* end = nullptr;
    const long index = std::strtol(text.c_str(), &end, 10);
    return !text.empty() && *end == '\0' && ResolveIndex(index, count, out);
}

// "7", "7/3", "7//2", "7/3/2"
bool ParseFaceVertex(const std::string& token, std::size_t n_v, std::size_t n_vn, FaceVertex& out) {
    const auto slash1 = token.find('/');
    if (!ParseIndex(token.substr(0, slash1), n_v, out.v)) return false;
    out.vn.reset();
    if (slash1 == std::string::npos) return true;
    const auto slash2 = token.find('/', slash1 + 1);
    if (slash2 == std::string::npos) return true;  // v/vt: texture index unused
    std::size_t vn = 0;
    if (!ParseIndex(token.substr(slash2 + 1), n_vn, vn)) return false;
    out.vn = vn;
    return true;
}

}  // namespace

std::size_t Model3D::Triangles() const {
    std::size_t n = 0;
    for (const auto& part : parts) n += part.Triangles();
    return n;
}

Vec3 Model3D::Pivot(std::string_view part) const {
    const auto it = pivots.find(part);
    return it == pivots.end() ? Vec3{} : it->second;
}

Materials ParseMtl(std::istream& mtl) {
    Materials materials;
    std::string line;
    std::string current;
    while (std::getline(mtl, line)) {
        std::istringstream in(line);
        std::string key;
        if (!(in >> key) || key[0] == '#') continue;
        if (key == "newmtl") {
            current = RestOfLine(in);
            materials[current] = kDefaultColor;
        } else if (current.empty()) {
            continue;
        } else if (key == "Kd") {
            double r = 0, g = 0, b = 0;
            if (in >> r >> g >> b) {
                auto& c = materials[current];
                c = {Channel(r), Channel(g), Channel(b), c.a};
            }
        } else if (key == "d" || key == "Tr") {
            double value = 1.0;
            if (in >> value) materials[current].a = Channel(key == "d" ? value : 1.0 - value);
        }
    }
    return materials;
}

Model3D ParseObj(std::istream& obj, const std::string& source_name, const MaterialLoader& load_materials) {
    Model3D model;
    model.source = source_name;
    auto warn = [&](const std::string& message) {
        if (std::find(model.warnings.begin(), model.warnings.end(), message) == model.warnings.end()) {
            model.warnings.push_back(message);
        }
    };

    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    Materials materials;
    Color color = kDefaultColor;
    std::map<std::string, std::size_t, std::less<>> part_index;
    std::size_t current = 0;
    auto select_part = [&](const std::string& name) {
        const auto [it, inserted] = part_index.emplace(name, model.parts.size());
        if (inserted) model.parts.push_back(Part3D{name, {}, {}, {}, {}});
        current = it->second;
    };
    select_part("");

    std::string line;
    std::size_t line_no = 0;
    std::vector<FaceVertex> face;
    while (std::getline(obj, line)) {
        line_no++;
        std::istringstream in(line);
        std::string key;
        if (!(in >> key) || key[0] == '#') continue;
        auto error = [&](const std::string& what) {
            return std::runtime_error(source_name + ":" + std::to_string(line_no) + ": " + what);
        };

        if (key == "v" || key == "vn") {
            Vec3 p;
            if (!(in >> p.x >> p.y >> p.z)) throw error("expected three numbers after '" + key + "'");
            if (key == "v") {
                vertices.push_back(p);
            } else {
                normals.push_back(Normalized(p).value_or(Vec3{0.0, 0.0, -1.0}));
            }
        } else if (key == "f") {
            face.clear();
            std::string token;
            while (in >> token) {
                FaceVertex fv{};
                if (!ParseFaceVertex(token, vertices.size(), normals.size(), fv)) {
                    throw error("bad face vertex '" + token + "'");
                }
                face.push_back(fv);
            }
            if (face.size() < 3) throw error("a face needs at least three vertices");

            Part3D& part = model.parts[current];
            for (std::size_t k = 1; k + 1 < face.size(); k++) {  // triangle fan
                const FaceVertex tri[3] = {face[0], face[k], face[k + 1]};
                const Vec3& a = vertices[tri[0].v];
                const auto flat = Normalized(Cross(Sub(vertices[tri[1].v], a), Sub(vertices[tri[2].v], a)));
                if (!flat) continue;  // degenerate
                for (const auto& fv : tri) {
                    part.positions.push_back(vertices[fv.v]);
                    part.normals.push_back(fv.vn ? normals[*fv.vn] : *flat);
                    part.colors.push_back(color);
                    part.bounds.Add(vertices[fv.v]);
                }
            }
        } else if (key == "o" || key == "g") {
            std::string name;
            in >> name;  // a g line may list several groups: use the first
            select_part(name);
        } else if (key == "usemtl") {
            const std::string name = RestOfLine(in);
            const auto it = materials.find(name);
            if (it != materials.end()) {
                color = it->second;
            } else {
                warn("unknown material '" + name + "'");
                color = kDefaultColor;
            }
        } else if (key == "mtllib") {
            const std::string file = RestOfLine(in);
            try {
                for (auto& [name, c] : load_materials(file)) materials[name] = c;
            } catch (const std::runtime_error& e) {
                warn(e.what());
            }
        }
        // vt, s, l, p, ... are not needed for drawing
    }

    // Drop empty parts, collect bounds and pivots
    model.parts.erase(std::remove_if(model.parts.begin(), model.parts.end(),
                                     [](const Part3D& p) { return p.positions.empty(); }),
                      model.parts.end());
    for (const auto& part : model.parts) {
        model.bounds.Add(part.bounds);
        if (!part.name.empty()) {
            const Vec3 c = part.bounds.Center();
            model.pivots[part.name] = {part.bounds.max.x, c.y, c.z};
        }
    }
    return model;
}

Model3D LoadModel3D(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Could not open " + path.string());
    const std::filesystem::path dir = path.parent_path();
    const MaterialLoader load = [&dir](const std::string& name) {
        const std::filesystem::path mtl_path = dir / name;
        std::ifstream mtl(mtl_path);
        if (!mtl) throw std::runtime_error("Could not open " + mtl_path.filename().string() + ", using grey");
        return ParseMtl(mtl);
    };
    Model3D model = ParseObj(file, path.filename().string(), load);
    if (model.Empty()) throw std::runtime_error(path.filename().string() + " has no faces");
    return model;
}

Model3D DefaultModel3D(double length, double breadth, double draft) {
    const double l = 0.5 * length;
    const double b = 0.5 * breadth;
    const double top = -0.8 * draft;  // freeboard
    const double r = 0.08 * length;   // rudder chord

    // Same outline as DefaultModel2D, extruded from the keel to the deck
    std::ostringstream obj;
    obj << "mtllib built-in\n";
    for (const double z : {top, draft}) {
        obj << "v " << l << " 0 " << z << "\nv " << 0.45 * l << " " << b << " " << z << "\n"
            << "v " << -l << " " << b << " " << z << "\nv " << -l << " " << -b << " " << z << "\n"
            << "v " << 0.45 * l << " " << -b << " " << z << "\n";
    }
    obj << "o hull\nusemtl deck\nf 1 2 3 4 5\nusemtl hull\nf 10 9 8 7 6\n"
        << "f 1 6 7 2\nf 2 7 8 3\nf 3 8 9 4\nf 4 9 10 5\nf 5 10 6 1\n";
    // Rudder: a thin plate behind the stern
    obj << "v " << -l << " 0 " << 0.3 * draft << "\nv " << -l - r << " 0 " << 0.3 * draft << "\n"
        << "v " << -l - r << " 0 " << draft << "\nv " << -l << " 0 " << draft << "\n"
        << "o rudder\nusemtl rudder\nf 11 12 13 14\n";

    std::istringstream in(obj.str());
    const MaterialLoader colors = [](const std::string&) {
        return Materials{{"deck", {200, 200, 190, 255}}, {"hull", {60, 70, 90, 255}}, {"rudder", {230, 80, 60, 255}}};
    };
    return ParseObj(in, "built-in", colors);
}

}  // namespace vessel_model
