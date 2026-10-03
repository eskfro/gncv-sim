// playback_2d: replays simulation runs as a top-down 2D animation.
//
// Usage: playback_2d [RUN | DIR] [options]   (see --help)

#include <cstdio>
#include <cstdlib>
#include <string>

#include "app.hpp"

namespace {

void PrintUsage() {
    std::printf(
        "Usage: playback_2d [RUN | DIR] [options]\n"
        "\n"
        "Plays back a simulation run. RUN is a run folder (simdata/<run>/), its\n"
        "metadata.json or a csv file. DIR is a folder of runs. With no argument\n"
        "the newest run in simdata/ is opened. The vessel is drawn from\n"
        "vessels/<vessel>/model_2d.svg, with the vessel named in metadata.json.\n"
        "\n"
        "Options:\n"
        "  --vessel NAME      draw this vessel (folder in vessels/) for every run\n"
        "  --speed X          playback speed, default 1 (real time)\n"
        "  --follow           keep the camera centred on the ship\n"
        "  --loop             restart at the end\n"
        "  --ui-scale S       scale fonts and widgets, e.g. 1.5 on HiDPI screens\n"
        "  --screenshot FILE  render one frame to FILE (BMP) and exit\n"
        "  --time T           simulation time for --screenshot, default start\n"
        "  -h, --help         show this help\n");
}

bool ParseDouble(const char* text, double& out) {
    char* end = nullptr;
    out = std::strtod(text, &end);
    return end != text && *end == '\0';
}

}  // namespace

int main(int argc, char** argv) {
    playback2d::Options options;

    for (int i = 1; i < argc; i++) {
        const std::string arg = argv[i];
        const bool has_value = i + 1 < argc;
        double value = 0.0;

        if (arg == "-h" || arg == "--help") {
            PrintUsage();
            return 0;
        } else if (arg == "--follow") {
            options.follow = true;
        } else if (arg == "--loop") {
            options.loop = true;
        } else if (arg == "--speed" && has_value && ParseDouble(argv[++i], value)) {
            options.speed = value;
        } else if (arg == "--ui-scale" && has_value && ParseDouble(argv[++i], value) && value > 0.0) {
            options.ui_scale = static_cast<float>(value);
        } else if (arg == "--vessel" && has_value) {
            options.vessel = argv[++i];
        } else if (arg == "--screenshot" && has_value) {
            options.screenshot = argv[++i];
        } else if (arg == "--time" && has_value && ParseDouble(argv[++i], value)) {
            options.screenshot_time = value;
        } else if (!arg.empty() && arg[0] != '-' && options.input.empty()) {
            options.input = arg;
        } else {
            std::fprintf(stderr, "playback_2d: bad argument '%s'\n\n", arg.c_str());
            PrintUsage();
            return 2;
        }
    }

    return playback2d::Run(options);
}
