// playback_2d: replays simulator_v1 csv files as a top-down 2D animation.
//
// Usage: playback_2d [FILE.csv | DIR] [options]   (see --help)

#include <cstdio>
#include <cstdlib>
#include <string>

#include "app.hpp"

namespace {

void PrintUsage() {
    std::printf(
        "Usage: playback_2d [FILE.csv | DIR] [options]\n"
        "\n"
        "Plays back a simulator_v1 recording. With no argument the newest csv in\n"
        "simdata/simulator_v1 is opened.\n"
        "\n"
        "Options:\n"
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
