#pragma once
// ============================================================================
// Playback time: maps wall clock time to simulation time at a chosen speed.
// ============================================================================

namespace playback {

class PlaybackClock {
public:
    static constexpr double kMinSpeed = 0.05;
    static constexpr double kMaxSpeed = 100.0;

    // Sets the playable range and rewinds to start. Speed and loop are kept.
    void Reset(double start, double end);

    // Advance by real (wall clock) seconds
    void Update(double real_dt);

    void Play();  // restarts from the beginning when at the end
    void Pause() { playing_ = false; }
    void TogglePlay() { playing_ ? Pause() : Play(); }
    bool Playing() const { return playing_; }

    void Seek(double t);
    void SeekBy(double dt) { Seek(time_ + dt); }

    double Speed() const { return speed_; }
    void SetSpeed(double speed);
    void Faster();  // next speed preset up
    void Slower();  // next speed preset down

    bool Loop() const { return loop_; }
    void SetLoop(bool loop) { loop_ = loop; }

    double Time() const { return time_; }
    double Start() const { return start_; }
    double End() const { return end_; }
    double Progress() const;  // 0 at start, 1 at end

private:
    double start_{0.0};
    double end_{0.0};
    double time_{0.0};
    double speed_{1.0};
    bool playing_{false};
    bool loop_{false};
};

}  // namespace playback
