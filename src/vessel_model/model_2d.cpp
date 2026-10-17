#include "vessel_model/model_2d.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "vessel_model/parts.hpp"

namespace vessel_model {

namespace {

constexpr int kCurveSegments = 12;   // per Bezier segment
constexpr int kCircleSegments = 48;  // full circle or ellipse
constexpr double kDeg2Rad = M_PI / 180.0;

// ---------------------------------------------------------------------------
// Affine transform: x' = a x + c y + e, y' = b x + d y + f
// ---------------------------------------------------------------------------
struct Affine {
    double a{1}, b{0}, c{0}, d{1}, e{0}, f{0};

    Vec2 Apply(Vec2 p) const { return {a * p.x + c * p.y + e, b * p.x + d * p.y + f}; }
    // this * other: apply other first
    Affine Then(const Affine& o) const {
        return {a * o.a + c * o.b, b * o.a + d * o.b, a * o.c + c * o.d,
                b * o.c + d * o.d, a * o.e + c * o.f + e, b * o.e + d * o.f + f};
    }
    double Scale() const { return std::sqrt(std::abs(a * d - b * c)); }
};

// ---------------------------------------------------------------------------
// Text helpers
// ---------------------------------------------------------------------------
std::string Trim(std::string_view s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) return {};
    const auto last = s.find_last_not_of(" \t\r\n");
    return std::string(s.substr(first, last - first + 1));
}

std::string Lower(std::string s) {
    for (auto& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return s;
}

bool IsSeparator(char ch) { return std::isspace(static_cast<unsigned char>(ch)) || ch == ','; }

// Reads the next number in a list like "1,2 -3.5e2.5", skipping separators
bool ReadNumber(const std::string& s, std::size_t& i, double& out) {
    while (i < s.size() && IsSeparator(s[i])) i++;
    if (i >= s.size()) return false;
    const char* begin = s.c_str() + i;
    char* end = nullptr;
    out = std::strtod(begin, &end);
    if (end == begin) return false;
    i += static_cast<std::size_t>(end - begin);
    return std::isfinite(out);
}

// Arc flags may be written without separators: "a5 5 0 015 5"
bool ReadFlag(const std::string& s, std::size_t& i, bool& out) {
    while (i < s.size() && IsSeparator(s[i])) i++;
    if (i >= s.size() || (s[i] != '0' && s[i] != '1')) return false;
    out = s[i++] == '1';
    return true;
}

std::vector<double> ReadNumbers(const std::string& s) {
    std::vector<double> values;
    std::size_t i = 0;
    double v = 0.0;
    while (ReadNumber(s, i, v)) values.push_back(v);
    return values;
}

double NumberOr(const std::string& s, double fallback) {
    std::size_t i = 0;
    double v = 0.0;
    return ReadNumber(s, i, v) ? v : fallback;
}

// ---------------------------------------------------------------------------
// Colors
// ---------------------------------------------------------------------------
struct NamedColor {
    const char* name;
    Color color;
};

constexpr std::array<NamedColor, 17> kNamedColors = {{
    {"black", {0, 0, 0, 255}},        {"white", {255, 255, 255, 255}}, {"red", {255, 0, 0, 255}},
    {"green", {0, 128, 0, 255}},      {"blue", {0, 0, 255, 255}},      {"yellow", {255, 255, 0, 255}},
    {"orange", {255, 165, 0, 255}},   {"gray", {128, 128, 128, 255}},  {"grey", {128, 128, 128, 255}},
    {"silver", {192, 192, 192, 255}}, {"navy", {0, 0, 128, 255}},      {"maroon", {128, 0, 0, 255}},
    {"lime", {0, 255, 0, 255}},       {"cyan", {0, 255, 255, 255}},    {"magenta", {255, 0, 255, 255}},
    {"brown", {165, 42, 42, 255}},    {"darkgray", {169, 169, 169, 255}},
}};

int HexDigit(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

// nullopt for "none". `ok` is false for colors that are not understood.
std::optional<Color> ParseColor(const std::string& text, bool& ok) {
    ok = true;
    const std::string s = Lower(Trim(text));
    if (s == "none" || s == "transparent") return std::nullopt;
    if (!s.empty() && s[0] == '#') {
        std::array<int, 6> h{};
        const std::size_t n = s.size() - 1;
        if (n == 3 || n == 6) {
            bool hex = true;
            for (std::size_t i = 0; i < n; i++) hex = hex && (h[i] = HexDigit(s[i + 1])) >= 0;
            if (hex && n == 3) {
                return Color{static_cast<std::uint8_t>(h[0] * 17), static_cast<std::uint8_t>(h[1] * 17),
                             static_cast<std::uint8_t>(h[2] * 17), 255};
            }
            if (hex) {
                return Color{static_cast<std::uint8_t>(h[0] * 16 + h[1]), static_cast<std::uint8_t>(h[2] * 16 + h[3]),
                             static_cast<std::uint8_t>(h[4] * 16 + h[5]), 255};
            }
        }
    }
    if (s.rfind("rgb(", 0) == 0) {
        const auto v = ReadNumbers(s.substr(4));
        if (v.size() >= 3) {
            auto channel = [](double x) { return static_cast<std::uint8_t>(std::clamp(x, 0.0, 255.0)); };
            return Color{channel(v[0]), channel(v[1]), channel(v[2]), 255};
        }
    }
    for (const auto& named : kNamedColors) {
        if (s == named.name) return named.color;
    }
    ok = false;
    return Color{0, 0, 0, 255};
}

Color WithAlpha(Color c, double opacity) {
    c.a = static_cast<std::uint8_t>(std::lround(c.a * std::clamp(opacity, 0.0, 1.0)));
    return c;
}

// ---------------------------------------------------------------------------
// Minimal XML tag scanner
// ---------------------------------------------------------------------------
struct Tag {
    std::string name;
    std::map<std::string, std::string> attributes;
    bool closing{false};       // </name>
    bool self_closing{false};  // <name/>
};

class TagScanner {
public:
    explicit TagScanner(std::string_view text) : text_(text) {}

    // Next tag, skipping text, comments, <?...?> and <!...>. False at the end.
    bool Next(Tag& tag) {
        while (true) {
            const auto lt = text_.find('<', pos_);
            if (lt == std::string_view::npos) return false;
            pos_ = lt;
            if (StartsWith("<!--")) {
                SkipPast("-->");
            } else if (StartsWith("<![CDATA[")) {
                SkipPast("]]>");
            } else if (StartsWith("<?")) {
                SkipPast("?>");
            } else if (StartsWith("<!")) {
                SkipPast(">");
            } else {
                return ParseTag(tag);
            }
        }
    }

private:
    bool StartsWith(std::string_view s) const { return text_.substr(pos_, s.size()) == s; }

    void SkipPast(std::string_view end) {
        const auto at = text_.find(end, pos_);
        if (at == std::string_view::npos) throw std::runtime_error("unterminated " + std::string(text_.substr(pos_, 4)));
        pos_ = at + end.size();
    }

    static bool IsNameChar(char ch) {
        return !std::isspace(static_cast<unsigned char>(ch)) && ch != '=' && ch != '/' && ch != '>';
    }

    bool ParseTag(Tag& tag) {
        tag = Tag{};
        pos_++;  // '<'
        if (pos_ < text_.size() && text_[pos_] == '/') {
            tag.closing = true;
            pos_++;
        }
        tag.name = ReadName();
        if (tag.name.empty()) throw std::runtime_error("malformed tag");

        while (true) {
            SkipSpace();
            if (pos_ >= text_.size()) throw std::runtime_error("unterminated <" + tag.name + ">");
            if (text_[pos_] == '>') {
                pos_++;
                return true;
            }
            if (text_[pos_] == '/') {
                tag.self_closing = true;
                pos_++;
                continue;
            }
            const std::string key = ReadName();
            if (key.empty()) throw std::runtime_error("malformed attribute in <" + tag.name + ">");
            SkipSpace();
            std::string value;
            if (pos_ < text_.size() && text_[pos_] == '=') {
                pos_++;
                SkipSpace();
                if (pos_ >= text_.size()) break;
                const char quote = text_[pos_];
                if (quote == '"' || quote == '\'') {
                    const auto close = text_.find(quote, pos_ + 1);
                    if (close == std::string_view::npos) break;
                    value = std::string(text_.substr(pos_ + 1, close - pos_ - 1));
                    pos_ = close + 1;
                } else {
                    value = ReadName();
                }
            }
            tag.attributes[key] = value;
        }
        throw std::runtime_error("unterminated <" + tag.name + ">");
    }

    std::string ReadName() {
        const std::size_t start = pos_;
        while (pos_ < text_.size() && IsNameChar(text_[pos_])) pos_++;
        return std::string(text_.substr(start, pos_ - start));
    }

    void SkipSpace() {
        while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) pos_++;
    }

    std::string_view text_;
    std::size_t pos_{0};
};

// ---------------------------------------------------------------------------
// Inherited state while walking the element tree
// ---------------------------------------------------------------------------
struct Context {
    Affine transform;
    std::optional<Color> fill{Color{0, 0, 0, 255}};  // SVG default: black fill
    std::optional<Color> stroke;                     // SVG default: no stroke
    double stroke_width{1.0};
    double opacity{1.0};
    double fill_opacity{1.0};
    double stroke_opacity{1.0};
    std::string part;
    bool hidden{false};
};

bool ParseTransform(const std::string& text, Affine& out) {
    std::size_t i = 0;
    Affine result;
    while (true) {
        while (i < text.size() && IsSeparator(text[i])) i++;
        if (i >= text.size()) break;
        const auto open = text.find('(', i);
        const auto close = text.find(')', open);
        if (open == std::string::npos || close == std::string::npos) return false;
        const std::string name = Trim(std::string_view(text).substr(i, open - i));
        const auto v = ReadNumbers(text.substr(open + 1, close - open - 1));
        Affine t;
        if (name == "matrix" && v.size() == 6) {
            t = {v[0], v[1], v[2], v[3], v[4], v[5]};
        } else if (name == "translate" && !v.empty()) {
            t.e = v[0];
            t.f = v.size() > 1 ? v[1] : 0.0;
        } else if (name == "scale" && !v.empty()) {
            t.a = v[0];
            t.d = v.size() > 1 ? v[1] : v[0];
        } else if (name == "rotate" && !v.empty()) {
            const double r = v[0] * kDeg2Rad;
            const Affine rot{std::cos(r), std::sin(r), -std::sin(r), std::cos(r), 0, 0};
            if (v.size() >= 3) {
                t = Affine{1, 0, 0, 1, v[1], v[2]}.Then(rot).Then(Affine{1, 0, 0, 1, -v[1], -v[2]});
            } else {
                t = rot;
            }
        } else if (name == "skewX" && v.size() == 1) {
            t.c = std::tan(v[0] * kDeg2Rad);
        } else if (name == "skewY" && v.size() == 1) {
            t.b = std::tan(v[0] * kDeg2Rad);
        } else {
            return false;
        }
        result = result.Then(t);
        i = close + 1;
    }
    out = result;
    return true;
}

class SvgParser {
public:
    explicit SvgParser(const std::string& source) { model_.source = source; }

    Model2D Parse(std::string_view text) {
        TagScanner scanner(text);
        std::vector<Context> stack;
        std::vector<std::string> open_tags;
        Tag tag;
        bool seen_svg = false;
        while (scanner.Next(tag)) {
            if (tag.closing) {
                if (!open_tags.empty()) {
                    open_tags.pop_back();
                    stack.pop_back();
                }
                continue;
            }
            const std::string name = LocalName(tag.name);
            if (name == "svg" && !seen_svg) {
                seen_svg = true;
            } else if (!seen_svg) {
                throw std::runtime_error("no <svg> element before <" + tag.name + ">");
            }

            Context ctx = stack.empty() ? Context{} : stack.back();
            Apply(tag, name, stack.empty(), ctx);
            if (!ctx.hidden) Emit(tag, name, ctx);
            if (!tag.self_closing) {
                open_tags.push_back(tag.name);
                stack.push_back(ctx);
            }
        }
        if (!seen_svg) throw std::runtime_error("no <svg> element");
        Finish();
        return std::move(model_);
    }

private:
    // "svg:path" -> "path"
    static std::string LocalName(const std::string& name) {
        const auto colon = name.find(':');
        return colon == std::string::npos ? name : name.substr(colon + 1);
    }

    void Warn(const std::string& message) {
        for (const auto& w : model_.warnings) {
            if (w == message) return;
        }
        model_.warnings.push_back(message);
    }

    void SetStyle(const std::string& key, const std::string& value, Context& ctx) {
        const std::string v = Trim(value);
        bool ok = true;
        if (key == "fill") {
            ctx.fill = ParseColor(v, ok);
        } else if (key == "stroke") {
            ctx.stroke = ParseColor(v, ok);
        } else if (key == "stroke-width") {
            ctx.stroke_width = NumberOr(v, ctx.stroke_width);
        } else if (key == "opacity") {
            ctx.opacity *= NumberOr(v, 1.0);
        } else if (key == "fill-opacity") {
            ctx.fill_opacity = NumberOr(v, 1.0);
        } else if (key == "stroke-opacity") {
            ctx.stroke_opacity = NumberOr(v, 1.0);
        } else if (key == "display") {
            ctx.hidden = ctx.hidden || v == "none";
        }
        if (!ok) Warn("unknown color '" + v + "', using black");
    }

    void Apply(const Tag& tag, const std::string& name, bool root, Context& ctx) {
        // Definitions are only drawn where they are used, which is not supported
        static const char* kNotDrawn[] = {"defs", "clipPath", "mask", "symbol", "marker", "pattern",
                                          "linearGradient", "radialGradient", "metadata", "title", "desc"};
        for (const char* skip : kNotDrawn) ctx.hidden = ctx.hidden || name == skip;

        const auto& attrs = tag.attributes;
        if (const auto it = attrs.find("transform"); it != attrs.end()) {
            Affine t;
            if (ParseTransform(it->second, t)) {
                ctx.transform = ctx.transform.Then(t);
            } else {
                Warn("unsupported transform '" + it->second + "' ignored");
            }
        }
        for (const char* key : {"fill", "stroke", "stroke-width", "opacity", "fill-opacity", "stroke-opacity", "display"}) {
            if (const auto it = attrs.find(key); it != attrs.end()) SetStyle(key, it->second, ctx);
        }
        // style="fill:#fff;stroke:none" overrides the attributes
        if (const auto it = attrs.find("style"); it != attrs.end()) {
            std::istringstream decls(it->second);
            std::string decl;
            while (std::getline(decls, decl, ';')) {
                const auto colon = decl.find(':');
                if (colon != std::string::npos) SetStyle(Trim(decl.substr(0, colon)), decl.substr(colon + 1), ctx);
            }
        }

        // A moving part keeps its name for everything inside it, even
        // elements with their own ids (Inkscape gives every element one)
        const auto id = attrs.find("id");
        const bool in_moving_part = FindPartMotion(ctx.part) != nullptr;
        if (!root && id != attrs.end() && !id->second.empty() && !in_moving_part) {
            ctx.part = id->second;
            if (const auto pivot = attrs.find("data-pivot"); pivot != attrs.end()) {
                const auto v = ReadNumbers(pivot->second);
                if (v.size() == 2) {
                    model_.pivots[ctx.part] = ctx.transform.Apply({v[0], v[1]});
                    explicit_pivots_.push_back(ctx.part);
                } else {
                    Warn("data-pivot of '" + ctx.part + "' should be \"x,y\"");
                }
            }
        }
    }

    double Attr(const Tag& tag, const char* key, double fallback = 0.0) const {
        const auto it = tag.attributes.find(key);
        return it == tag.attributes.end() ? fallback : NumberOr(it->second, fallback);
    }

    std::string AttrText(const Tag& tag, const char* key) const {
        const auto it = tag.attributes.find(key);
        return it == tag.attributes.end() ? std::string{} : it->second;
    }

    void Emit(const Tag& tag, const std::string& name, const Context& ctx) {
        if (name == "path") {
            ParsePath(AttrText(tag, "d"), ctx);
        } else if (name == "polygon" || name == "polyline") {
            const auto v = ReadNumbers(AttrText(tag, "points"));
            std::vector<Vec2> points;
            for (std::size_t i = 0; i + 1 < v.size(); i += 2) points.push_back({v[i], v[i + 1]});
            AddShape(std::move(points), name == "polygon", ctx);
        } else if (name == "line") {
            AddShape({{Attr(tag, "x1"), Attr(tag, "y1")}, {Attr(tag, "x2"), Attr(tag, "y2")}}, false, ctx);
        } else if (name == "rect") {
            const double x = Attr(tag, "x");
            const double y = Attr(tag, "y");
            const double w = Attr(tag, "width");
            const double h = Attr(tag, "height");
            if (w > 0.0 && h > 0.0) AddShape({{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}}, true, ctx);
        } else if (name == "circle" || name == "ellipse") {
            const double cx = Attr(tag, "cx");
            const double cy = Attr(tag, "cy");
            const double rx = name == "circle" ? Attr(tag, "r") : Attr(tag, "rx");
            const double ry = name == "circle" ? rx : Attr(tag, "ry");
            if (rx <= 0.0 || ry <= 0.0) return;
            std::vector<Vec2> points;
            for (int k = 0; k < kCircleSegments; k++) {
                const double a = 2.0 * M_PI * k / kCircleSegments;
                points.push_back({cx + rx * std::cos(a), cy + ry * std::sin(a)});
            }
            AddShape(std::move(points), true, ctx);
        } else if (name == "text" || name == "image" || name == "use" || name == "foreignObject") {
            Warn("<" + name + "> is not supported and was skipped");
        }
    }

    void AddShape(std::vector<Vec2> points, bool closed, const Context& ctx) {
        // Repeated points (and the end point of a closed path that returns
        // to its start) make zero length edges, which polygon fills dislike
        auto same = [](Vec2 a, Vec2 b) { return std::abs(a.x - b.x) < 1e-9 && std::abs(a.y - b.y) < 1e-9; };
        points.erase(std::unique(points.begin(), points.end(), same), points.end());
        if (closed && points.size() > 1 && same(points.front(), points.back())) points.pop_back();
        if (points.size() < 2) return;
        Shape2D shape;
        shape.part = ctx.part;
        shape.closed = closed;
        // As in SVG, open paths are filled too, as if closed
        if (ctx.fill && points.size() >= 3) shape.fill = WithAlpha(*ctx.fill, ctx.opacity * ctx.fill_opacity);
        if (ctx.stroke && ctx.stroke_width > 0.0) {
            shape.stroke = WithAlpha(*ctx.stroke, ctx.opacity * ctx.stroke_opacity);
            shape.stroke_width = ctx.stroke_width * ctx.transform.Scale();
        }
        if (!shape.fill && !shape.stroke) return;  // invisible
        for (auto& p : points) {
            p = ctx.transform.Apply(p);
            model_.bounds.Add(Vec3{p.x, p.y, 0.0});
        }
        shape.points = std::move(points);
        model_.shapes.push_back(std::move(shape));
    }

    // Endpoint arc to points, SVG spec appendix F.6.5. Excludes the start point.
    static void ArcPoints(Vec2 p1, Vec2 p2, double rx, double ry, double phi_deg, bool large, bool sweep,
                          std::vector<Vec2>& out) {
        rx = std::abs(rx);
        ry = std::abs(ry);
        if (rx == 0.0 || ry == 0.0) {
            out.push_back(p2);
            return;
        }
        const double phi = phi_deg * kDeg2Rad;
        const double cp = std::cos(phi);
        const double sp = std::sin(phi);
        const double dx2 = 0.5 * (p1.x - p2.x);
        const double dy2 = 0.5 * (p1.y - p2.y);
        const double x1p = cp * dx2 + sp * dy2;
        const double y1p = -sp * dx2 + cp * dy2;
        const double lambda = x1p * x1p / (rx * rx) + y1p * y1p / (ry * ry);
        if (lambda > 1.0) {
            rx *= std::sqrt(lambda);
            ry *= std::sqrt(lambda);
        }
        const double num = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p;
        const double den = rx * rx * y1p * y1p + ry * ry * x1p * x1p;
        const double coef = (large == sweep ? -1.0 : 1.0) * std::sqrt(std::max(0.0, den > 0.0 ? num / den : 0.0));
        const double cxp = coef * rx * y1p / ry;
        const double cyp = -coef * ry * x1p / rx;
        const double cx = cp * cxp - sp * cyp + 0.5 * (p1.x + p2.x);
        const double cy = sp * cxp + cp * cyp + 0.5 * (p1.y + p2.y);

        auto angle = [](double ux, double uy, double vx, double vy) {
            return std::atan2(ux * vy - uy * vx, ux * vx + uy * vy);
        };
        const double ux = (x1p - cxp) / rx;
        const double uy = (y1p - cyp) / ry;
        const double theta1 = angle(1.0, 0.0, ux, uy);
        double delta = angle(ux, uy, (-x1p - cxp) / rx, (-y1p - cyp) / ry);
        if (!sweep && delta > 0.0) delta -= 2.0 * M_PI;
        if (sweep && delta < 0.0) delta += 2.0 * M_PI;

        const int n = std::max(2, static_cast<int>(std::ceil(std::abs(delta) / (2.0 * M_PI) * kCircleSegments)));
        for (int k = 1; k <= n; k++) {
            const double t = theta1 + delta * k / n;
            out.push_back({cx + rx * cp * std::cos(t) - ry * sp * std::sin(t),
                           cy + rx * sp * std::cos(t) + ry * cp * std::sin(t)});
        }
        out.back() = p2;  // exact end point
    }

    void ParsePath(const std::string& d, const Context& ctx) {
        std::vector<Vec2> points;
        Vec2 cur{};
        Vec2 start{};
        Vec2 last_ctrl{};      // for S and T
        char prev_cmd = ' ';
        char cmd = ' ';
        std::size_t i = 0;

        auto flush = [&](bool closed) {
            AddShape(std::move(points), closed, ctx);
            points.clear();
        };
        auto bezier = [&](Vec2 p0, Vec2 c1, Vec2 c2, Vec2 p3) {
            for (int k = 1; k <= kCurveSegments; k++) {
                const double t = static_cast<double>(k) / kCurveSegments;
                const double u = 1.0 - t;
                points.push_back({u * u * u * p0.x + 3 * u * u * t * c1.x + 3 * u * t * t * c2.x + t * t * t * p3.x,
                                  u * u * u * p0.y + 3 * u * u * t * c1.y + 3 * u * t * t * c2.y + t * t * t * p3.y});
            }
        };
        auto quad = [&](Vec2 p0, Vec2 c, Vec2 p2) {
            // Degree elevation: quadratic as cubic
            bezier(p0, {p0.x + 2.0 / 3.0 * (c.x - p0.x), p0.y + 2.0 / 3.0 * (c.y - p0.y)},
                   {p2.x + 2.0 / 3.0 * (c.x - p2.x), p2.y + 2.0 / 3.0 * (c.y - p2.y)}, p2);
        };

        while (true) {
            while (i < d.size() && IsSeparator(d[i])) i++;
            if (i >= d.size()) break;
            if (std::isalpha(static_cast<unsigned char>(d[i]))) {
                cmd = d[i++];
            } else if (cmd == ' ') {
                Warn("path data must start with a command");
                return;
            }
            // A number after M/m continues as L/l
            const bool relative = std::islower(static_cast<unsigned char>(cmd));
            const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(cmd)));
            const Vec2 base = relative ? cur : Vec2{0.0, 0.0};
            auto pt = [&](double x, double y) { return Vec2{base.x + x, base.y + y}; };
            double v[7] = {};
            auto read = [&](int n) {
                for (int k = 0; k < n; k++) {
                    if (!ReadNumber(d, i, v[k])) return false;
                }
                return true;
            };
            bool ok = true;

            switch (upper) {
                case 'M':
                    if ((ok = read(2))) {
                        if (!points.empty()) flush(false);
                        cur = start = pt(v[0], v[1]);
                        points.push_back(cur);
                        cmd = relative ? 'l' : 'L';
                    }
                    break;
                case 'L':
                    if ((ok = read(2))) points.push_back(cur = pt(v[0], v[1]));
                    break;
                case 'H':
                    if ((ok = read(1))) points.push_back(cur = {base.x + v[0], cur.y});
                    break;
                case 'V':
                    if ((ok = read(1))) points.push_back(cur = {cur.x, base.y + v[0]});
                    break;
                case 'C':
                    if ((ok = read(6))) {
                        const Vec2 c2 = pt(v[2], v[3]);
                        const Vec2 end = pt(v[4], v[5]);
                        bezier(cur, pt(v[0], v[1]), c2, end);
                        last_ctrl = c2;
                        cur = end;
                    }
                    break;
                case 'S':
                    if ((ok = read(4))) {
                        const bool smooth = prev_cmd == 'C' || prev_cmd == 'S';
                        const Vec2 c1 = smooth ? Vec2{2 * cur.x - last_ctrl.x, 2 * cur.y - last_ctrl.y} : cur;
                        const Vec2 c2 = pt(v[0], v[1]);
                        const Vec2 end = pt(v[2], v[3]);
                        bezier(cur, c1, c2, end);
                        last_ctrl = c2;
                        cur = end;
                    }
                    break;
                case 'Q':
                    if ((ok = read(4))) {
                        const Vec2 c = pt(v[0], v[1]);
                        const Vec2 end = pt(v[2], v[3]);
                        quad(cur, c, end);
                        last_ctrl = c;
                        cur = end;
                    }
                    break;
                case 'T':
                    if ((ok = read(2))) {
                        const bool smooth = prev_cmd == 'Q' || prev_cmd == 'T';
                        const Vec2 c = smooth ? Vec2{2 * cur.x - last_ctrl.x, 2 * cur.y - last_ctrl.y} : cur;
                        const Vec2 end = pt(v[0], v[1]);
                        quad(cur, c, end);
                        last_ctrl = c;
                        cur = end;
                    }
                    break;
                case 'A': {
                    bool large = false;
                    bool sweep = false;
                    ok = ReadNumber(d, i, v[0]) && ReadNumber(d, i, v[1]) && ReadNumber(d, i, v[2]) &&
                         ReadFlag(d, i, large) && ReadFlag(d, i, sweep) && ReadNumber(d, i, v[3]) &&
                         ReadNumber(d, i, v[4]);
                    if (ok) {
                        const Vec2 end = pt(v[3], v[4]);
                        ArcPoints(cur, end, v[0], v[1], v[2], large, sweep, points);
                        cur = end;
                    }
                    break;
                }
                case 'Z':
                    if (!points.empty()) flush(true);
                    cur = start;
                    // A path may go on after Z without a new M
                    points.push_back(cur);
                    break;
                default:
                    Warn(std::string("unknown path command '") + cmd + "'");
                    return;
            }
            if (!ok) {
                Warn("malformed path data near '" + d.substr(i, 20) + "'");
                break;
            }
            prev_cmd = upper;
            if (upper == 'Z') cmd = ' ';  // numbers right after Z are an error
        }
        if (points.size() >= 2) flush(false);
    }

    // Leading edge of each named part without an explicit pivot
    void Finish() {
        std::map<std::string, Bounds3, std::less<>> part_bounds;
        for (const auto& shape : model_.shapes) {
            if (shape.part.empty()) continue;
            for (const auto& p : shape.points) part_bounds[shape.part].Add(Vec3{p.x, p.y, 0.0});
        }
        for (const auto& [part, b] : part_bounds) {
            const bool is_explicit = std::find(explicit_pivots_.begin(), explicit_pivots_.end(), part) != explicit_pivots_.end();
            if (!is_explicit) model_.pivots[part] = {b.max.x, b.Center().y};
        }
    }

    Model2D model_;
    std::vector<std::string> explicit_pivots_;
};

}  // namespace

Vec2 Model2D::Pivot(std::string_view part) const {
    const auto it = pivots.find(part);
    return it == pivots.end() ? Vec2{} : it->second;
}

Model2D ParseSvg(std::string_view svg, const std::string& source_name) {
    try {
        return SvgParser(source_name).Parse(svg);
    } catch (const std::runtime_error& e) {
        throw std::runtime_error(source_name + ": " + e.what());
    }
}

Model2D LoadModel2D(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Could not open " + path.string());
    std::ostringstream text;
    text << file.rdbuf();
    Model2D model = ParseSvg(text.str(), path.filename().string());
    if (model.Empty()) throw std::runtime_error(path.filename().string() + " has no visible shapes");
    return model;
}

Model2D DefaultModel2D(double length, double breadth) {
    const double l = 0.5 * length;
    const double b = 0.5 * breadth;
    Model2D model;
    model.source = "built-in";

    Shape2D hull;
    hull.points = {{l, 0.0}, {0.45 * l, b}, {-l, b}, {-l, -b}, {0.45 * l, -b}};
    hull.fill = Color{235, 235, 225, 255};
    hull.stroke = Color{30, 30, 30, 255};
    hull.stroke_width = 0.02 * breadth;

    Shape2D bridge;
    bridge.points = {{-0.35 * l, 0.7 * b}, {-0.65 * l, 0.7 * b}, {-0.65 * l, -0.7 * b}, {-0.35 * l, -0.7 * b}};
    bridge.fill = Color{120, 130, 140, 255};

    Shape2D rudder;
    rudder.part = "rudder";
    rudder.points = {{-l, 0.0}, {-1.18 * l, 0.0}};
    rudder.closed = false;
    rudder.stroke = Color{255, 90, 70, 255};
    rudder.stroke_width = 0.06 * breadth;

    for (auto* shape : {&hull, &bridge, &rudder}) {
        for (const auto& p : shape->points) model.bounds.Add(Vec3{p.x, p.y, 0.0});
        model.shapes.push_back(*shape);
    }
    model.pivots["rudder"] = {-l, 0.0};
    return model;
}

}  // namespace vessel_model
