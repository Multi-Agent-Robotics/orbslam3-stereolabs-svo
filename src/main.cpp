#include <bits/c++config.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <optional>
#include <orbslam3/ImuTypes.h>
#include <sophus/se3.hpp>
#include <string>
#include <thread>

// Needed for terminal dimensions
#include <sys/ioctl.h>
#include <unistd.h>

#include "conversions.hpp"
#include "escape_codes.hpp"
#include "formatters.hpp"
#include "macros.hpp"
#include "typedefs.hpp"
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
	std::printf(" %3d%%\r", ( int )(ratio * 100));
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
	std::printf("    .orientation: (%3f, %3f, %3f, %3f)\n", orientation.x, orientation.y,
				orientation.z, orientation.w);
	std::printf("  .linear_acceleration: (%3f, %3f, %3f)\n", imu_data.linear_acceleration.x,
				imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
	std::printf("  .angular_velocity: (%3f, %3f, %3f)\n", imu_data.angular_velocity.x,
				imu_data.angular_velocity.y, imu_data.angular_velocity.z);
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

auto pretty_print_svo(sl::Camera& zed_camera) -> void {
	const int nb_frames		 = zed_camera.getSVONumberOfFrames();
	const int svo_frame_rate = zed_camera.getInitParameters().camera_fps;
	const sl::Resolution resolution =
		zed_camera.getCameraInformation().camera_configuration.resolution;
	std::fprintf(stdout, "SVO:\n");
	std::fprintf(stdout, "  - %d frames\n", nb_frames);
	std::fprintf(stdout, "  - %d FPS\n", svo_frame_rate);
	const double duration = nb_frames / static_cast<double>(svo_frame_rate);
	std::fprintf(stdout, "  - %.2f seconds\n", duration);
	std::fprintf(stdout, "  - %zux%zu resolution\n", resolution.width, resolution.height);
	std::fprintf(stdout, "  - depth minimum distance: %f\n",
				 zed_camera.getInitParameters().depth_minimum_distance);
	std::fprintf(stdout, "  - depth maximum distance: %f\n",
				 zed_camera.getInitParameters().depth_maximum_distance);
}

constexpr auto default_path_to_vocabulary =
	"/home/orin/multi-agent-robotics/ORB-SLAM3-STEREO-FIXED/vocabulary/ORBvoc.txt";
constexpr auto default_path_to_settings =
	"/home/orin/ros2_ws/src/orbslam3/orbslam3_wrapper/config/stereo-inertial/EuRoC.yaml";

auto create_options_parser(int argc, char* argv[]) -> cxxopts::Options {
	const std::string program_name		  = argv[0];
	const std::string program_description = "Feed a recorded *.svo file to ORB-SLAM3.";

	bool use_viewer		= true;
	bool loop_recording = false;

	auto options = cxxopts::Options(program_name, program_description);
	options.add_options()("svo", "The *.svo file to give as input", cxxopts::value<std::string>())(
		"vocabulary", "Path to ORB vocabulary",
		cxxopts::value<std::string>()->default_value(default_path_to_vocabulary))(
		"settings", "Path to ORB-SLAM3 settings",
		cxxopts::value<std::string>()->default_value(default_path_to_settings))(
		"use-viewer", "Use ORB-SLAM3 Pangolin viewer", cxxopts::value(use_viewer))(
		"loop-recording", "Loop around and replay the recording when it finishes",
		cxxopts::value(loop_recording))("progressbar",
										"Display a progress bar while replaying the recording",
										cxxopts::value<bool>()->implicit_value("true"))
		//   ("view", "The sl::VIEW to use for the .svo file",
		//   cxxopts::value<std::string>()->default_value("SIDE_BY_SIDE"))
		("h,help", "Print usage information");

	return options;
}

auto parse_argv(cxxopts::Options& parser, int argc, char* argv[]) -> std::optional<Options> {
	auto result = parser.parse(argc, argv);

	if (result.count("svo") == 0) {
		std::fprintf(stderr, "%serror:%s Missing required argument: --svo\n",
					 escape_codes::colors::fg::red, escape_codes::reset);
		return std::nullopt;
	}
	const std::string path_to_svo = result["svo"].as<std::string>();

	if (! verify_filepath_argument(path_to_svo, "svo")) {
		std::fprintf(stderr, "%serror:%s Invalid argument: --svo\n", escape_codes::colors::fg::red,
					 escape_codes::reset);
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
		std::fprintf(stderr, "%serror:%s Invalid argument: --vocabulary\n",
					 escape_codes::colors::fg::red, escape_codes::reset);
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
		std::fprintf(stderr, "%serror:%s Invalid argument: --settings\n",
					 escape_codes::colors::fg::red, escape_codes::reset);
		return std::nullopt;
	}

	return Options {
		.path_to_vocabulary = path_to_vocabulary,
		.path_to_settings	= path_to_settings,
		.path_to_svo_file	= path_to_svo,
		.use_viewer			= result.count("use-viewer") != 0,
		.loop_recording		= result.count("loop-recording") != 0,
		.progressbar		= result["progressbar"].as<bool>(),
		.help				= result.count("help") || argc == 1,
	};
}

auto main(int argc, char* argv[]) -> int {
	// Parse command line arguments
	auto parser		= create_options_parser(argc, argv);
	const auto opt	= parse_argv(parser, argc, argv);
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
	const sl::Resolution resolution_captured_at =
		camera_information.camera_configuration.resolution;

	using std::min;
	const std::size_t max_width	 = 720;
	const std::size_t max_height = 404;
	const sl::Resolution desired_resolution(min(max_width, resolution_captured_at.width),
											min(max_height, resolution_captured_at.height));
	// Define OpenCV window size (resize to max 720/404)
	// sl::Resolution low_resolution(std::min(720, ( int )resolution.width) * 2,
	// 							  std::min(404, ( int )resolution.height));
	// sl::Mat svo_image(low_resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
	// cv::Mat svo_image_ocv = sl_mat_to_cv_mat(svo_image);

	pretty_print_svo(zed_camera);

	const int nb_frames = zed_camera.getSVONumberOfFrames();

	// Create a orbslam3 system object
	orbslam3::System orbslam3_system(options.path_to_vocabulary, options.path_to_settings,
									 orbslam3::System::IMU_STEREO, options.use_viewer);

	// Instantiate image matrices outside of the loop, to avoid memory allocation
	// 'sl_mat_to_cv_mat' is a helper function to convert sl::Mat to cv::Mat. It
	// avoids memory allocation and data copy. So it only needs to be called once.
	sl::Mat svo_img_left(resolution_captured_at, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
	sl::Mat svo_img_right(resolution_captured_at, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
	cv::Mat svo_img_left_ocv  = sl_mat_to_cv_mat(svo_img_left);
	cv::Mat svo_img_right_ocv = sl_mat_to_cv_mat(svo_img_right);

	auto imu_measurements = std::vector<orbslam3::IMU::Point>();

	// Start SVO playback
	char key = ' ';
	while (key != 'q') {
		const auto err = zed_camera.grab();
		if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
			std::fprintf(stderr, "[info] SVO end has been reached.\n");
			if (options.loop_recording) {
				std::fprintf(stderr, "[info] Looping back to start of SVO file.\n");
				zed_camera.setSVOPosition(0);
			} else {
				std::fprintf(stderr, "[info] Exiting.\n");
				break;
			}
		} else if (err != sl::ERROR_CODE::SUCCESS) {
			std::fprintf(stderr, "%serror:%s Failed to grab frame with error code: ",
						 escape_codes::colors::fg::red, escape_codes::reset);
                         std::cerr << err << std::endl;
			break;
		}

		// Get the left and right images
		// NOTE: The images will be rectified, by `retrieveImage`
		zed_camera.retrieveImage(svo_img_left, sl::VIEW::LEFT, sl::MEM::CPU,
								 resolution_captured_at);
		zed_camera.retrieveImage(svo_img_right, sl::VIEW::RIGHT, sl::MEM::CPU,
								 resolution_captured_at);

		auto imu_data = retrieve_imu_data(zed_camera).value();

        const auto acc = cv::Point3f(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
        const auto gyr = cv::Point3f(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);

        // TODO: using SVO mode will only allow the most recent IMU measurement to be used
        // I think
        const auto imu_measurements = Vec<orbslam3::IMU::Point> {
            orbslam3::IMU::Point {
                acc,
                gyr,
                static_cast<double>(imu_data.timestamp.getNanoseconds())
            }
        };
    

        auto timestamp = zed_camera.getTimestamp(sl::TIME_REFERENCE::IMAGE);

		// Get the IMU data
		// auto imu_data = [&] {
		//     const auto opt = retrieve_imu_data(zed_camera);
		//     if (opt) {
		//         return opt.value();
		//     } else {
		//         return orbslam3::IMU::Data();
		//     }
		// };

		// Get frame count
		const int svo_position = zed_camera.getSVOPosition();

		if (options.progressbar && isatty(fileno(stdout))) {
			progressbar(static_cast<double>(svo_position) / static_cast<double>(nb_frames), 80);
		}

		// cv::imshow("Left Image", svo_img_left_ocv);
		// cv::imshow("Right Image", svo_img_right_ocv);
		key = cv::waitKey(10); // delay in ms

		// Pass the images to orbslam3
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.

		Sophus::SE3f camera_pose = orbslam3_system.TrackStereo(svo_img_left_ocv, svo_img_right_ocv, static_cast<double>(timestamp.getNanoseconds()),
									imu_measurements);

		std::this_thread::sleep_for(1ms);
	}

	orbslam3_system.Shutdown();

	zed_camera.close();
	return EXIT_SUCCESS;
}
