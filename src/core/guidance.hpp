// #pragma twice
#pragma once

#include "core/common.hpp"
#include "core/config.hpp"

namespace guidance {

/*
Guides according to the mission
*/
class Guidance {
public:     
    void Step(double dt);

    const common::Desired& Desired() const { return desired_; }
    const common::GuidanceMode& GuidanceMode() const { return guidance_mode_; }

private:
    double time_{};
    config::HeadingReferenceModelParams psi_ref_params_{};
    config::SurgeSpeedReferenceModelParams surge_ref_params_{};
    common::GuidanceMode guidance_mode_{};
    common::Desired desired_{};
    
};

} // namespace control