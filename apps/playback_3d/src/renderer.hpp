#pragma once
// ============================================================================
// OpenGL 3.3 renderer for the 3D view.
//
// Draw order: the vessel mesh (lit, opaque, writes depth), then the scene
// triangles (water, track, markers) blended and depth tested against the
// vessel, then the scene lines. The water is translucent, so the hull below
// the waterline shows through it, tinted.
// ============================================================================
#include <array>
#include <string>
#include <vector>

#include "math3d.hpp"
#include "scene.hpp"
#include "vessel_model/model_3d.hpp"

namespace playback3d {

struct RenderView {
    std::array<int, 4> viewport{};  // x, y (from the bottom), width, height in framebuffer pixels
    Mat4 view;
    Mat4 projection;
    Rgba sky;          // background, and the fog color far away
    float fog_start{};  // [m] from the eye
    float fog_end{};
};

class Renderer {
public:
    Renderer() = default;
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer();

    // Needs a current OpenGL 3.3 core context. False with a message on failure.
    bool Init(std::string& error);
    // Uploads the model, replacing the previous one. Parts keep their order.
    void SetModel(const vessel_model::Model3D& model);
    void Render(const RenderView& view, const SceneBatch& scene);

private:
    struct Mesh {
        unsigned vao{0};
        unsigned vbo{0};
        int count{0};
    };
    void Release();

    unsigned mesh_program_{0};
    unsigned flat_program_{0};
    unsigned dynamic_vao_{0};
    unsigned dynamic_vbo_{0};
    std::vector<Mesh> parts_;
};

}  // namespace playback3d
