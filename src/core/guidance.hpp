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

    const common::Reference& Reference() const { return reference_; }
    const common::GuidanceMode& GuidanceMode() const { return guidance_mode_; }

private:
    double time_{};
    common::GuidanceMode guidance_mode_ = common::GuidanceMode::HeadingHold;
    common::Reference reference_{};
};

} // namespace control