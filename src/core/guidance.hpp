// #pragma twice
#pragma once

#include "core/common.hpp"

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
    common::HeadingReferenceModelParams ref_params_{};
    common::GuidanceMode guidance_mode_ = common::GuidanceMode::HeadingHold;
    common::Desired desired_{};
    
};

} // namespace control