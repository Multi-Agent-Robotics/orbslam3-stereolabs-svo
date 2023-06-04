#include <cstdio>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

// Needed for terminal dimensions
#include <sys/ioctl.h>
#include <unistd.h>

#include "conversions.hpp"
#include "escape_codes.hpp"
#include "formatters.hpp"
#include "macros.hpp"
#include <cxxopts.hpp>
#include <opencv2/opencv.hpp>
#include <orbslam3/System.h>
#include <sl/Camera.hpp>

auto hr(char c = '-', int n = 80) -> std::string { return std::string(n, c); }

struct TerminalDimensions {
	unsigned int width;
	unsigned int height;
};

auto get_terminal_dimensions() -> TerminalDimensions {
	struct winsize w;
	ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
	return TerminalDimensions {
		.width	= w.ws_col,
		.height = w.ws_row,
	};
}

auto progressbar(double ratio, unsigned int width) -> void {
	unsigned int c = ratio * width;
    
    std::printf("%s", std::string(c, '=').c_str());
    std::printf("%s", std::string(width - c, ' ').c_str());
    std::printf(" %3d%%\r", (int)(ratio * 100));
    std::fflush(stdout);
}

auto retrieve_imu_data(sl::Camera& camera) -> std::optional<sl::SensorsData::IMUData> {
	sl::SensorsData sensors_data;
	// TIME_REFERENCE::IMAGE is used to retrieve only frame synchronized data
	if (camera.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS) {
		return std::nullopt;
	}
	return sensors_data.imu;
}

auto pretty_print_imu_data(sl::SensorsData::IMUData& imu_data) -> void {
    const auto translation = imu_data.pose.getTranslation();
    const auto orientation = imu_data.pose.getOrientation();
    std::printf("imu_data:\n");
    std::printf("  .timestamp (ms): %lu\n", imu_data.timestamp.getMilliseconds());
    std::printf("  .pose:\n");
    std::printf("    .translation: (%3f, %3f, %3f)\n", translation.x, translation.y, translation.z);
    std::printf("    .orientation: (%3f, %3f, %3f, %3f)\n", orientation.x, orientation.y, orientation.z, orientation.w);
    std::printf("  .linear_acceleration: (%3f, %3f, %3f)\n", imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
    std::printf("  .angular_velocity: (%3f, %3f, %3f)\n", imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
    std::printf("\n");
}

auto verify_filepath_argument(const std::string& filepath,
							  const std::string required_file_extension = "") -> bool {
	// 1. Verify that the file exists and is readable
	auto f = std::ifstream(filepath);
	if (! f.good()) {
		std::fprintf(stderr, "%serror%s: Cannot read file at %s%s%s\n",
					 escape_codes::colors::fg::red, escape_codes::reset, escape_codes::bold,
					 filepath.c_str(), escape_codes::reset);
		f.close();
		return false;
	}

	if (! required_file_extension.empty()) {
		// 2. Verify that the file extension is correct
		std::string extension = filepath.substr(filepath.find_last_of(".") + 1);
		if (extension != required_file_extension) {
			std::fprintf(stderr, "%serror%s: File extension must be %s%s%s\n",
						 escape_codes::colors::fg::red, escape_codes::reset, escape_codes::bold,
						 required_file_extension.c_str(), escape_codes::reset);
			f.close();
			return false;
		}
	}

	f.close();
	return true;
}


struct Options {
    std::string path_to_vocabulary;
    std::string path_to_settings;
    std::string path_to_svo_file;
    bool use_viewer;
    bool loop_recording;
    bool progressbar;
    bool help;
};

auto pretty_print_options(const Options& options) -> void {
    std::printf("Options:\n");
    std::printf("  .path_to_vocabulary: %s\n", format_filepath(options.path_to_vocabulary).c_str());
    std::printf("  .path_to_settings: %s\n", format_filepath(options.path_to_settings).c_str());
    std::printf("  .path_to_svo_file: %s\n", format_filepath(options.path_to_svo_file).c_str());
    std::printf("  .use_viewer: %s\n", format_bool(options.use_viewer).c_str());
    std::printf("  .loop_recording: %s\n", format_bool(options.loop_recording).c_str());
    std::printf("  .progressbar: %s\n", format_bool(options.progressbar).c_str());
    std::printf("  .help: %s\n", format_bool(options.help).c_str());
    std::printf("\n");
}

constexpr auto default_path_to_vocabulary = "/home/orin/multi-agent-robotics/ORB-SLAM3-STEREO-FIXED/vocabulary/ORBvoc.txt";
constexpr auto default_path_to_settings = "/home/orin/ros2_ws/src/orbslam3/orbslam3_wrapper/config/stereo-inertial/EuRoC.yaml";

auto create_options_parser(int argc, char* argv[]) -> cxxopts::Options {
    const std::string program_name		  = argv[0];
	const std::string program_description = "Feed a recorded *.svo file to ORB-SLAM3.";

	bool use_viewer		= true;
	bool loop_recording = false;

	auto options = cxxopts::Options(program_name, program_description);
	options.add_options()("svo", "The *.svo file to give as input",
						  cxxopts::value<std::string>())(
		"vocabulary", "Path to ORB vocabulary",
		cxxopts::value<std::string>()->default_value(default_path_to_vocabulary))(
		"settings", "Path to ORB-SLAM3 settings",
		cxxopts::value<std::string>()->default_value(default_path_to_settings))(
		"use-viewer", "Use viewer",
		cxxopts::value(use_viewer))("loop-recording", "Loop around and replay the recording when it finishes",
									cxxopts::value(loop_recording))
                                                    ("progressbar", "Display a progress bar while replaying the recording", cxxopts::value<bool>()->implicit_value("true"))
		//   ("view", "The sl::VIEW to use for the .svo file",
		//   cxxopts::value<std::string>()->default_value("SIDE_BY_SIDE"))
		("h,help", "Print usage information");

    return options;
}

auto parse_argv(cxxopts::Options& parser, int argc, char *argv[]) -> std::optional<Options> {
    auto result = parser.parse(argc, argv);

    if (result.count("svo") == 0) {
        std::fprintf(stderr, "%serror:%s Missing required argument: --svo\n", escape_codes::colors::fg::red, escape_codes::reset);
        return std::nullopt;
    }
	const std::string path_to_svo = result["svo"].as<std::string>();

	if (! verify_filepath_argument(path_to_svo, "svo")) {
		std::fprintf(stderr, "%serror:%s Invalid argument: --svo\n", escape_codes::colors::fg::red, escape_codes::reset);
        return std::nullopt;
	}

	// If the user did not specify a path to a vocabulary, use the default
	const bool use_default_vocabulary = result.count("vocabulary") == 0;
	if (use_default_vocabulary) {
		std::fprintf(stderr, "%sinfo:%s Using default ORB vocabulary at %s%s%s\n",
					 escape_codes::colors::fg::yellow, escape_codes::reset, escape_codes::bold,
					 default_path_to_vocabulary, escape_codes::reset);
	}

    const std::string path_to_vocabulary = result["vocabulary"].as<std::string>();
    if (! verify_filepath_argument(path_to_vocabulary, "txt")) {
        std::fprintf(stderr, "%serror:%s Invalid argument: --vocabulary\n", escape_codes::colors::fg::red, escape_codes::reset);
        return std::nullopt;
    }

    // If the user did not specify a path to a settings file, use the default
    const bool use_default_settings = result.count("settings") == 0;
    if (use_default_settings) {
        std::fprintf(stderr, "%sinfo:%s Using default ORB-SLAM3 settings at %s%s%s\n",
                     escape_codes::colors::fg::yellow, escape_codes::reset, escape_codes::bold,
                     default_path_to_settings, escape_codes::reset);
    }

    const std::string path_to_settings = result["settings"].as<std::string>();
    if (! verify_filepath_argument(path_to_settings, "yaml")) {
        std::fprintf(stderr, "%serror:%s Invalid argument: --settings\n", escape_codes::colors::fg::red, escape_codes::reset);
        return std::nullopt;
    }

	return Options {
		.path_to_vocabulary = path_to_vocabulary,
		.path_to_settings	= path_to_settings,
		.path_to_svo_file	= path_to_svo,
		.use_viewer			= result.count("use_viewer") != 0,
		.loop_recording		= result.count("loop") != 0,
        .progressbar        = result["progressbar"].as<bool>(),
		.help				= result.count("help") || argc == 1,
	};
}


auto main(int argc, char * argv[]) -> int {
    // Parse command line arguments
    auto parser = create_options_parser(argc, argv);
    const auto opt = parse_argv(parser, argc, argv);
    auto print_help = [&parser]() {
		std::fprintf(stderr, "%s\n", hr().c_str());
		std::fprintf(stderr, "%s\n", parser.help().c_str());
	};
    if (! opt) {
        print_help();
        return EXIT_FAILURE;
    }

    const auto options = opt.value();
    pretty_print_options(options);

    if (options.help) {
        print_help();
        return EXIT_SUCCESS;
    }

	// Create a ZED camera object
	sl::Camera zed_camera;
	sl::InitParameters init_params;
	init_params.input.setFromSVOFile(options.path_to_svo_file.c_str());
	init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;

	// Open the camera
	auto returned_state = zed_camera.open(init_params);
	if (returned_state != sl::ERROR_CODE::SUCCESS) {
        DEBUG_LOG(stderr, "Camera failed to open with error code: %d\n", returned_state);
		std::fprintf(stderr, "%serror:%s Camera failed to open with error code: ",
					 escape_codes::colors::fg::red, escape_codes::reset);
		std::cerr << returned_state << std::endl;
		return EXIT_FAILURE;
	}

	const sl::CameraInformation camera_information = zed_camera.getCameraInformation();
	const sl::Resolution resolution = camera_information.camera_configuration.resolution;
	const float fps					= camera_information.camera_configuration.fps;
	// Define OpenCV window size (resize to max 720/404)
	sl::Resolution low_resolution(std::min(720, ( int )resolution.width) * 2,
								  std::min(404, ( int )resolution.height));
	sl::Mat svo_image(low_resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
	cv::Mat svo_image_ocv = sl_mat_to_cv_mat(svo_image);

	// Setup key, images, times
	char key = ' ';
	// std::cout << " Press 's' to save SVO image as a PNG" << std::endl;
	// std::cout << " Press 'f' to jump forward in the video" << std::endl;
	// std::cout << " Press 'b' to jump backward in the video" << std::endl;
	// std::cout << " Press 'q' to exit..." << std::endl;

	int svo_frame_rate = zed_camera.getInitParameters().camera_fps;
	int nb_frames	   = zed_camera.getSVONumberOfFrames();
	std::fprintf(stdout, "[Info] SVO file is %d frames\n", nb_frames);

	// Create a orbslam3 system object

	//   orbslam3::System orbslam3_system(vocabulary_filepath, settings_filepath,
	//    orbslam3::System::IMU_STEREO, use_viewer);

	// Start SVO playback

	while (key != 'q') {
		returned_state = zed_camera.grab();
		if (returned_state == sl::ERROR_CODE::SUCCESS) {

			// Get the side by side image
			// const sl::VIEW svo_view_mode = sl::VIEW::SIDE_BY_SIDE; // or
			// UNRECTIFIED | GREYSCALE
			const sl::VIEW svo_view_mode = sl::VIEW::SIDE_BY_SIDE; // or UNRECTIFIED | GREYSCALE
			zed_camera.retrieveImage(svo_image, svo_view_mode, sl::MEM::CPU, low_resolution);
			// Get frame count
			int svo_position = zed_camera.getSVOPosition();

			if (const auto opt = retrieve_imu_data(zed_camera)) {
				auto imu_data = opt.value();
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

			if (options.progressbar && isatty(fileno(stdout))) {
				progressbar(static_cast<double>(svo_position) / static_cast<double>(nb_frames), 80);
            }

		} else if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
			std::fprintf(stderr, "[Info] SVO end has been reached. Looping back to 0\n");
			if (options.loop_recording) {
				zed_camera.setSVOPosition(0);

			} else {
				break;
			}
		} else {
			std::cerr << "Grab ZED : " << returned_state << std::endl;
			break;
		}
	}

	zed_camera.close();
	return EXIT_SUCCESS;
}
