#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <optional>

#include <sys/ioctl.h>
#include <unistd.h>

#include "conversions.hpp"
#include "escape_codes.hpp"
#include <cxxopts.hpp>
#include <opencv2/opencv.hpp>
#include <orbslam3/System.h>
#include <sl/Camera.hpp>

auto hr(char c = '-', int n = 80) -> std::string {
    return std::string(n, c);
}

struct TerminalDimensions {
    unsigned int width;
    unsigned int height;
};

auto get_terminal_dimensions() -> TerminalDimensions {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    return TerminalDimensions{
        .width = w.ws_col,
        .height = w.ws_row,
    };
}

auto progressbar(double ratio, unsigned int width) -> void {
    unsigned int c = ratio * width;
    for (unsigned int x = 0; x < c; x++) std::cout << "=";
    for (unsigned int x = c; x < width; x++) std::cout << " ";
    std::cout << (unsigned int) (ratio * 100) << "% ";
    std::cout << "\r" << std::flush;
}

// struct ImuInformation {
//     sl::Timestamp timestamp;
//     sl::Transform pose; // IMU 6-DoF pose
//     sl::float3 linear_acceleration;
//     // sl::float3 angular_velocity; // \note Not available in SVO or Stream mode. (see /usr/local/zed/include/sl/Camera.hpp)
// };

auto retrive_imu_data(sl::Camera& camera) -> std::optional<sl::SensorsData::IMUData> {
    sl::SensorsData sensors_data;
    // TIME_REFERENCE::IMAGE is used to retrieve only frame synchronized data
    if (camera.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS) {
        return std::nullopt;
    }
    return sensors_data.imu;
    // const auto imu_data = sensors_data.imu;

    // return imu_data;

    // return ImuInformation{
    //     .timestamp = imu_data.timestamp,
    //     .pose = imu_data.pose,
    //     .timestamp = imu_data.timestamp.getNanoseconds(),
    //     .linear_acceleration = imu_data.linear_acceleration,
    //     .angular_velocity = imu_data.angular_velocity,
    // };
}

auto pretty_print_imu_data(const sl::SensorsData::IMUData& imu_data) -> void {
    std::cout << "IMU Data:" << std::endl;
    // std::cout << "  Timestamp: " << imu_data.timestamp.getNanoseconds() << std::endl;
    // std::cout << "  Pose: " << imu_data.pose << std::endl;
    // std::cout << "  Linear Acceleration: " << imu_data.linear_acceleration << std::endl;
    // std::cout << "  Angular Velocity: " << imu_data.angular_velocity << std::endl;
}



auto verify_filepath_argument(const std::string &filepath,
                              const std::string required_file_extension = "")
    -> bool {
  // 1. Verify that the file exists and is readable
  auto f = std::ifstream(filepath);
  if (!f.good()) {
    std::fprintf(stderr, "%serror%s: Cannot read file at %s%s%s\n",
                 escape_codes::colors::fg::red, escape_codes::reset,
                 escape_codes::bold, filepath.c_str(), escape_codes::reset);
    f.close();
    return false;
  }

  if (!required_file_extension.empty()) {
    // 2. Verify that the file extension is correct
    std::string extension = filepath.substr(filepath.find_last_of(".") + 1);
    if (extension != required_file_extension) {
      std::fprintf(stderr, "%serror%s: File extension must be %s%s%s\n",
                   escape_codes::colors::fg::red, escape_codes::reset,
                   escape_codes::bold, required_file_extension.c_str(),
                   escape_codes::reset);
      f.close();
      return false;
    }
  }

  f.close();
  return true;
}

auto main(int argc, char const *argv[]) -> int {
  const std::string program_name = argv[0];
  const std::string program_description =
      "Feed a recorded *.svo file to ORB-SLAM3.";
  bool use_viewer = true;

    const std::string default_path_to_vocabulary =
      "/home/orin/multi-agent-robotics/ORB-SLAM3-STEREO-FIXED/vocabulary/"
      "ORBvoc.txt";
  const std::string default_path_to_settings =
      "/home/orin/ros2_ws/src/orbslam3/orbslam3_wrapper/config/stereo-inertial/"
      "EuRoC.yaml";
  auto options = cxxopts::Options(program_name, program_description);
  options.add_options()("f,file", "The *.svo file to give as input",
                        cxxopts::value<std::string>())(
      "vocabulary", "Path to ORB vocabulary", cxxopts::value<std::string>()->default_value(default_path_to_vocabulary))(
      "settings", "Path to ORB-SLAM3 settings", cxxopts::value<std::string>()->default_value(default_path_to_settings))(
      "use_viewer", "Use viewer, default true.",
      cxxopts::value(use_viewer))
    //   ("view", "The sl::VIEW to use for the .svo file", cxxopts::value<std::string>()->default_value("SIDE_BY_SIDE"))
      ("h,help", "Print usage information");

  auto result = options.parse(argc, argv);

  auto print_help = [&options]() {
    std::fprintf(stderr, "%s\n", hr().c_str());
    std::fprintf(stderr, "%s\n", options.help().c_str());
  };

  if (result.count("help") || argc == 1) {
    print_help();
    return EXIT_SUCCESS;
  }

  // Read input parameters
  if (result.count("file") == 0) {
    std::fprintf(
        stderr,
        "%serror:%s Please specify a file path to a video or image sequence.\n",
        escape_codes::colors::fg::red, escape_codes::reset);
    print_help();
    return EXIT_FAILURE;
  }
  std::string svo_filepath = result["file"].as<std::string>();

  if (svo_filepath.empty()) {
    std::fprintf(stderr,
                 "%serror:%s Please specify a file path to a *.svo file\n",
                 escape_codes::colors::fg::red, escape_codes::reset);
    return EXIT_FAILURE;
  }

    if (!verify_filepath_argument(svo_filepath, "svo")) {
        return EXIT_FAILURE;
    }

    // If the user did not specify a path to a vocabulary, use the default
    const bool use_default_vocabulary = result.count("vocabulary") == 0;
    if (use_default_vocabulary) {
        std::fprintf(stderr, "%sinfo:%s Using default ORB vocabulary at %s%s%s\n",
                     escape_codes::colors::fg::yellow, escape_codes::reset,
                     escape_codes::bold, default_path_to_vocabulary.c_str(),
                     escape_codes::reset);
    }

    std::string vocabulary_filepath = result["vocabulary"].as<std::string>();

    if (vocabulary_filepath.empty()) {
        std::fprintf(stderr,
                     "%serror:%s Please specify a file path to a ORB vocabulary.\n",
                     escape_codes::colors::fg::red, escape_codes::reset);
        return EXIT_FAILURE;
    }

    if (!verify_filepath_argument(vocabulary_filepath, "txt")) {
        return EXIT_FAILURE;
    }

    // If the user did not specify a path to a settings file, use the default
    const bool use_default_settings = result.count("settings") == 0;
    if (use_default_settings) {
        std::fprintf(stderr, "%sinfo:%s Using default ORB-SLAM3 settings at %s%s%s\n",
                     escape_codes::colors::fg::yellow, escape_codes::reset,
                     escape_codes::bold, default_path_to_settings.c_str(),
                     escape_codes::reset);
    }

    std::string settings_filepath = result["settings"].as<std::string>();

    if (settings_filepath.empty()) {
        std::fprintf(stderr,
                     "%serror:%s Please specify a file path to a ORB-SLAM3 settings.\n",
                     escape_codes::colors::fg::red, escape_codes::reset);
        return EXIT_FAILURE;
    }

    if (!verify_filepath_argument(settings_filepath, "yaml")) {
        return EXIT_FAILURE;
    }



  // Create a ZED camera object
    sl::Camera zed_camera;
    sl::InitParameters init_params;
    init_params.input.setFromSVOFile(svo_filepath.c_str());
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;

    // Open the camera
    auto returned_state = zed_camera.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::fprintf(stderr, "%serror:%s Camera failed to open with error code: ",
                     escape_codes::colors::fg::red, escape_codes::reset);
                     std::cerr << returned_state << std::endl;
        return EXIT_FAILURE;
    }

        auto resolution = zed_camera.getCameraInformation().camera_configuration.resolution;

// Define OpenCV window size (resize to max 720/404)
    sl::Resolution low_resolution(std::min(720, (int)resolution.width) * 2, std::min(404, (int)resolution.height));
    sl::Mat svo_image(low_resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    cv::Mat svo_image_ocv = sl_mat_to_cv_mat(svo_image);

    // Setup key, images, times
    char key = ' ';
    std::cout << " Press 's' to save SVO image as a PNG" << std::endl;
    std::cout << " Press 'f' to jump forward in the video" << std::endl;
    std::cout << " Press 'b' to jump backward in the video" << std::endl;
    std::cout << " Press 'q' to exit..." << std::endl;

    int svo_frame_rate = zed_camera.getInitParameters().camera_fps;
    int nb_frames = zed_camera.getSVONumberOfFrames();
    std::fprintf(stdout, "[Info] SVO file is %d frames\n", nb_frames);

    // Start SVO playback

     while (key != 'q') {
        returned_state = zed_camera.grab();
        if (returned_state == sl::ERROR_CODE::SUCCESS) {

            // Get the side by side image
            // const sl::VIEW svo_view_mode = sl::VIEW::SIDE_BY_SIDE; // or UNRECTIFIED | GREYSCALE 
            const sl::VIEW svo_view_mode = sl::VIEW::SIDE_BY_SIDE; // or UNRECTIFIED | GREYSCALE 
            zed_camera.retrieveImage(svo_image, svo_view_mode, sl::MEM::CPU, low_resolution);
                    // Get frame count      
            int svo_position = zed_camera.getSVOPosition();

            if (const auto opt = retrive_imu_data(zed_camera)) {
                const auto imu_data = opt.value();
                pretty_print_imu_data(imu_data);
            }

            // Display the frame
            cv::imshow("View", svo_image_ocv);
            key = cv::waitKey(10);
            
            switch (key) {
            case 's':
                svo_image.write(("capture_" + to_string(svo_position) + ".png").c_str());
                break;
            case 'f':
                zed_camera.setSVOPosition(svo_position + svo_frame_rate);
                break;
            case 'b':
                zed_camera.setSVOPosition(svo_position - svo_frame_rate);
                break;
            }

            // progressbar(static_cast<double>(svo_position) / static_cast<double>(nb_frames), get_terminal_dimensions().width - 10);
            progressbar(static_cast<double>(svo_position) / static_cast<double>(nb_frames), 80);
        }
        else if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
            std::fprintf(stderr, "[Info] SVO end has been reached. Looping back to 0\n");
            zed_camera.setSVOPosition(0);
        }
        else {
            std::cerr << "Grab ZED : " << returned_state << std::endl;
            break;
        }

     } 


  // Create a orbslam3 system object

//   orbslam3::System orbslam3_system(vocabulary_filepath, settings_filepath,
                                //    orbslam3::System::IMU_STEREO, use_viewer);




  zed_camera.close();
  return EXIT_SUCCESS;
}
