#include "renderer.hpp"

// Linux: libGL exports the OpenGL 3.3 functions, so no loader is needed
#define GL_GLEXT_PROTOTYPES 1
#include <SDL_opengl.h>

#include <cstddef>

namespace playback3d {

namespace {

// Lit mesh. Lighting is two sided, so face winding in the OBJ does not matter.
constexpr const char* kMeshVertex = R"glsl(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec4 a_color;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec3 v_normal;
out vec4 v_color;
out vec3 v_eye;
void main() {
    vec4 eye = u_view * u_model * vec4(a_pos, 1.0);
    v_normal = mat3(u_model) * a_normal;
    v_color = a_color;
    v_eye = eye.xyz;
    gl_Position = u_proj * eye;
}
)glsl";

constexpr const char* kMeshFragment = R"glsl(#version 330 core
in vec3 v_normal;
in vec4 v_color;
in vec3 v_eye;
uniform vec3 u_light;
uniform vec4 u_fog_color;
uniform vec2 u_fog;
out vec4 frag;
void main() {
    float diffuse = abs(dot(normalize(v_normal), u_light));
    vec3 color = v_color.rgb * (0.35 + 0.65 * diffuse);
    float fog = smoothstep(u_fog.x, u_fog.y, length(v_eye));
    frag = vec4(mix(color, u_fog_color.rgb, fog), v_color.a);
}
)glsl";

// Unlit colored triangles and lines
constexpr const char* kFlatVertex = R"glsl(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec4 a_color;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec4 v_color;
out vec3 v_eye;
void main() {
    vec4 eye = u_view * vec4(a_pos, 1.0);
    v_color = a_color;
    v_eye = eye.xyz;
    gl_Position = u_proj * eye;
}
)glsl";

constexpr const char* kFlatFragment = R"glsl(#version 330 core
in vec4 v_color;
in vec3 v_eye;
uniform vec4 u_fog_color;
uniform vec2 u_fog;
out vec4 frag;
void main() {
    // Per fragment: the water is one big quad, so per vertex would fog all of it
    float fog = smoothstep(u_fog.x, u_fog.y, length(v_eye));
    frag = vec4(mix(v_color.rgb, u_fog_color.rgb, fog), v_color.a);
}
)glsl";

// Towards the sun, NED: high up, from the south west
const V3 kLight = Normalize({-0.35, -0.25, -1.0});

struct MeshVertex {
    float pos[3];
    float normal[3];
    float color[4];
};

GLuint Compile(GLenum type, const char* source, std::string& error) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        char log[1024] = {};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        error = std::string("shader compile failed: ") + log;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint Link(const char* vertex, const char* fragment, std::string& error) {
    const GLuint vs = Compile(GL_VERTEX_SHADER, vertex, error);
    if (vs == 0) return 0;
    const GLuint fs = Compile(GL_FRAGMENT_SHADER, fragment, error);
    if (fs == 0) {
        glDeleteShader(vs);
        return 0;
    }
    const GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        char log[1024] = {};
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        error = std::string("shader link failed: ") + log;
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

void SetMatrix(GLuint program, const char* name, const Mat4& m) {
    const auto f = m.ToFloat();
    glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, f.data());
}

void SetFog(GLuint program, const RenderView& view) {
    glUniform4f(glGetUniformLocation(program, "u_fog_color"), view.sky.r, view.sky.g, view.sky.b, 1.0f);
    glUniform2f(glGetUniformLocation(program, "u_fog"), view.fog_start, view.fog_end);
}

void DrawDynamic(GLuint vbo, const std::vector<Vertex>& vertices, GLenum mode) {
    if (vertices.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data(),
                 GL_STREAM_DRAW);
    glDrawArrays(mode, 0, static_cast<GLsizei>(vertices.size()));
}

}  // namespace

Renderer::~Renderer() { Release(); }

void Renderer::Release() {
    for (auto& mesh : parts_) {
        glDeleteBuffers(1, &mesh.vbo);
        glDeleteVertexArrays(1, &mesh.vao);
    }
    parts_.clear();
    if (dynamic_vbo_ != 0) glDeleteBuffers(1, &dynamic_vbo_);
    if (dynamic_vao_ != 0) glDeleteVertexArrays(1, &dynamic_vao_);
    if (mesh_program_ != 0) glDeleteProgram(mesh_program_);
    if (flat_program_ != 0) glDeleteProgram(flat_program_);
    dynamic_vbo_ = dynamic_vao_ = mesh_program_ = flat_program_ = 0;
}

bool Renderer::Init(std::string& error) {
    mesh_program_ = Link(kMeshVertex, kMeshFragment, error);
    if (mesh_program_ == 0) return false;
    flat_program_ = Link(kFlatVertex, kFlatFragment, error);
    if (flat_program_ == 0) return false;

    glGenVertexArrays(1, &dynamic_vao_);
    glGenBuffers(1, &dynamic_vbo_);
    glBindVertexArray(dynamic_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, dynamic_vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, x)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, r)));
    glBindVertexArray(0);
    return true;
}

void Renderer::SetModel(const vessel_model::Model3D& model) {
    for (auto& mesh : parts_) {
        glDeleteBuffers(1, &mesh.vbo);
        glDeleteVertexArrays(1, &mesh.vao);
    }
    parts_.clear();

    std::vector<MeshVertex> vertices;
    for (const auto& part : model.parts) {
        vertices.clear();
        for (std::size_t i = 0; i < part.positions.size(); i++) {
            const auto& p = part.positions[i];
            const auto& n = part.normals[i];
            const auto& c = part.colors[i];
            vertices.push_back({{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)},
                                {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z)},
                                {c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, c.a / 255.0f}});
        }
        Mesh mesh;
        mesh.count = static_cast<int>(vertices.size());
        glGenVertexArrays(1, &mesh.vao);
        glGenBuffers(1, &mesh.vbo);
        glBindVertexArray(mesh.vao);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(MeshVertex)), vertices.data(),
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                              reinterpret_cast<void*>(offsetof(MeshVertex, pos)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                              reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                              reinterpret_cast<void*>(offsetof(MeshVertex, color)));
        parts_.push_back(mesh);
    }
    glBindVertexArray(0);
}

void Renderer::Render(const RenderView& view, const SceneBatch& scene) {
    const auto& vp = view.viewport;
    if (vp[2] <= 0 || vp[3] <= 0) return;
    glViewport(vp[0], vp[1], vp[2], vp[3]);
    glEnable(GL_SCISSOR_TEST);
    glScissor(vp[0], vp[1], vp[2], vp[3]);
    glClearColor(view.sky.r, view.sky.g, view.sky.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    // Vessel
    if (const auto& vessel = scene.GetVessel(); vessel && !parts_.empty()) {
        glUseProgram(mesh_program_);
        SetMatrix(mesh_program_, "u_view", view.view);
        SetMatrix(mesh_program_, "u_proj", view.projection);
        SetFog(mesh_program_, view);
        glUniform3f(glGetUniformLocation(mesh_program_, "u_light"), static_cast<float>(kLight.x),
                    static_cast<float>(kLight.y), static_cast<float>(kLight.z));
        for (std::size_t i = 0; i < parts_.size(); i++) {
            const Mat4 model = i < vessel->parts.size() ? vessel->body * vessel->parts[i] : vessel->body;
            SetMatrix(mesh_program_, "u_model", model);
            glBindVertexArray(parts_[i].vao);
            glDrawArrays(GL_TRIANGLES, 0, parts_[i].count);
        }
    }

    // Water, track and markers: blended, depth tested against the vessel only
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUseProgram(flat_program_);
    SetMatrix(flat_program_, "u_view", view.view);
    SetMatrix(flat_program_, "u_proj", view.projection);
    SetFog(flat_program_, view);
    glBindVertexArray(dynamic_vao_);
    DrawDynamic(dynamic_vbo_, scene.Triangles(), GL_TRIANGLES);
    DrawDynamic(dynamic_vbo_, scene.Lines(), GL_LINES);

    glBindVertexArray(0);
    glUseProgram(0);
    glDepthMask(GL_TRUE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_SCISSOR_TEST);
}

}  // namespace playback3d
