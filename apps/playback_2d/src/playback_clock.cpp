#include "playback_clock.hpp"

#include <algorithm>
#include <iterator>
#include <cmath>

namespace playback2d {

namespace {
constexpr double kSpeedPresets[] = {0.1, 0.25, 0.5, 1, 2, 4, 8, 16, 32, 64, 100};
constexpr double kEps = 1e-9;
}  // namespace

void PlaybackClock::Reset(double start, double end) {
    start_ = start;
    end_ = std::max(start, end);
    time_ = start_;
}

void PlaybackClock::Update(double real_dt) {
    if (!playing_ || real_dt <= 0.0) return;
    time_ += real_dt * speed_;
    if (time_ < end_) return;

    const double duration = end_ - start_;
    if (loop_ && duration > 0.0) {
        time_ = start_ + std::fmod(time_ - start_, duration);
    } else {
        time_ = end_;
        playing_ = false;
    }
}

void PlaybackClock::Play() {
    if (time_ >= end_) time_ = start_;
    playing_ = true;
}

void PlaybackClock::Seek(double t) { time_ = std::clamp(t, start_, end_); }

void PlaybackClock::SetSpeed(double speed) {
    if (!std::isfinite(speed)) return;
    speed_ = std::clamp(speed, kMinSpeed, kMaxSpeed);
}

void PlaybackClock::Faster() {
    for (const double s : kSpeedPresets) {
        if (s > speed_ + kEps) {
            SetSpeed(s);
            return;
        }
    }
    SetSpeed(kMaxSpeed);
}

void PlaybackClock::Slower() {
    for (auto it = std::rbegin(kSpeedPresets); it != std::rend(kSpeedPresets); ++it) {
        if (*it < speed_ - kEps) {
            SetSpeed(*it);
            return;
        }
    }
    SetSpeed(kMinSpeed);
}

double PlaybackClock::Progress() const {
    const double duration = end_ - start_;
    return duration > 0.0 ? (time_ - start_) / duration : 0.0;
}

}  // namespace playback2d
