#include "camera_calibration_studio/calibration.hpp"
#include "camera_calibration_studio/dataset.hpp"

#include <filesystem>
#include <iostream>
#include <map>
#include <string>

namespace {

void usage() {
  std::cout
      << "Mono:\n  calibrate_dataset --mono --images DIR --output FILE [--name NAME]\n"
      << "Stereo:\n  calibrate_dataset --stereo --left-images DIR --right-images DIR "
         "--left-intrinsics FILE --right-intrinsics FILE --output FILE\n";
}

std::map<std::string, std::string> arguments(const int argc, char** argv) {
  std::map<std::string, std::string> result;
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index];
    if (key == "--mono" || key == "--stereo") {
      result[key] = "true";
    } else if (key.rfind("--", 0) == 0 && index + 1 < argc) {
      result[key] = argv[++index];
    }
  }
  return result;
}

std::string required(const std::map<std::string, std::string>& args, const std::string& key) {
  const auto found = args.find(key);
  return found == args.end() ? std::string{} : found->second;
}

}  // namespace

int main(const int argc, char** argv) {
  const auto args = arguments(argc, argv);
  std::string error;
  if (args.count("--mono") != 0U) {
    const auto images_directory = required(args, "--images");
    const auto output = required(args, "--output");
    if (images_directory.empty() || output.empty()) {
      usage();
      return 2;
    }
    calibration_studio::Intrinsics intrinsics;
    if (!calibration_studio::calibrateMono(calibration_studio::listImages(images_directory), {},
                                           intrinsics, error)) {
      std::cerr << error << '\n';
      return 1;
    }
    const auto name = required(args, "--name").empty() ? "camera" : required(args, "--name");
    if (!calibration_studio::saveIntrinsics(output, name, intrinsics, error)) {
      std::cerr << error << '\n';
      return 1;
    }
    std::cout << "Mono calibration saved to " << output << ": RMS=" << intrinsics.rms_px
              << " px, used=" << intrinsics.used_images.size()
              << ", rejected=" << intrinsics.rejected_images.size() << '\n';
    return 0;
  }
  if (args.count("--stereo") != 0U) {
    const auto left_images = required(args, "--left-images");
    const auto right_images = required(args, "--right-images");
    const auto left_file = required(args, "--left-intrinsics");
    const auto right_file = required(args, "--right-intrinsics");
    const auto output = required(args, "--output");
    if (left_images.empty() || right_images.empty() || left_file.empty() || right_file.empty() ||
        output.empty()) {
      usage();
      return 2;
    }
    calibration_studio::Intrinsics left;
    calibration_studio::Intrinsics right;
    if (!calibration_studio::loadIntrinsics(left_file, left, error) ||
        !calibration_studio::loadIntrinsics(right_file, right, error)) {
      std::cerr << error << '\n';
      return 1;
    }
    calibration_studio::StereoResult result;
    if (!calibration_studio::calibrateStereoAruco(left_images, right_images, left, right, {},
                                                   result, error) ||
        !calibration_studio::saveStereoResult(output, result, error)) {
      std::cerr << error << '\n';
      return 1;
    }
    std::cout << "Stereo calibration saved to " << output << ": RMS=" << result.rms_px
              << " px, baseline=" << result.baseline_m << " m, pairs=" << result.used_pairs
              << '\n';
    return 0;
  }
  usage();
  return 2;
}

