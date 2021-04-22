// Modified from concatenate_normalized_landmark_list_calculator.cc. output an empty list when there is no body detected.

#ifndef MEDIAPIPE_CALCULATORS_CORE_CONCATENATE_NORMALIZED_LIST_CALCULATOR_H_  // NOLINT
#define MEDIAPIPE_CALCULATORS_CORE_CONCATENATE_NORMALIZED_LIST_CALCULATOR_H_  // NOLINT

#include "mediapipe/calculators/core/concatenate_vector_calculator.pb.h"
#include "mediapipe/framework/api2/node.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/canonical_errors.h"
#include "mediapipe/framework/port/ret_check.h"
#include "mediapipe/framework/port/status.h"

namespace mediapipe {
namespace api2 {

// Concatenates several NormalizedLandmarkList protos following stream index
// order. This class assumes that every input stream contains a
// NormalizedLandmarkList proto object.
class ConcatenateNormalizedLandmarkListCalculatorV2 : public Node {
 public:
  static constexpr Input<NormalizedLandmarkList>::Multiple kIn{""};
  static constexpr Output<NormalizedLandmarkList> kOut{""};

  MEDIAPIPE_NODE_CONTRACT(kIn, kOut);

  static mediapipe::Status UpdateContract(CalculatorContract* cc) {
    RET_CHECK_GE(kIn(cc).Count(), 1);
    return mediapipe::OkStatus();
  }

  mediapipe::Status Open(CalculatorContext* cc) override {
    only_emit_if_all_present_ =
        cc->Options<::mediapipe::ConcatenateVectorCalculatorOptions>()
            .only_emit_if_all_present();
    return mediapipe::OkStatus();
  }

  mediapipe::Status Process(CalculatorContext* cc) override {
    bool misssingAny = false;
    if (only_emit_if_all_present_) {
      for (int i = 0; i < kIn(cc).Count(); ++i) {
        if (kIn(cc)[i].IsEmpty()) misssingAny = true;
      }
    }

    // output an empty list if missing any input
    NormalizedLandmarkList output;
    if (misssingAny) {
      output = NormalizedLandmarkList();
      kOut(cc).Send(std::move(output));
      return mediapipe::OkStatus();
    }

    for (int i = 0; i < kIn(cc).Count(); ++i) {
      if (kIn(cc)[i].IsEmpty()) continue;
      const NormalizedLandmarkList& input = *kIn(cc)[i];
      for (int j = 0; j < input.landmark_size(); ++j) {
        *output.add_landmark() = input.landmark(j);
      }
    }
    kOut(cc).Send(std::move(output));
    return mediapipe::OkStatus();
  }

 private:
  bool only_emit_if_all_present_;
};
MEDIAPIPE_REGISTER_NODE(ConcatenateNormalizedLandmarkListCalculatorV2);

}  // namespace api2
}  // namespace mediapipe

// NOLINTNEXTLINE
#endif  // MEDIAPIPE_CALCULATORS_CORE_CONCATENATE_NORMALIZED_LIST_CALCULATOR_H_
