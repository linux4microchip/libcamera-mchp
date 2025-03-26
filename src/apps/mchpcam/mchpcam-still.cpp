/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* mchpcam - mchpcam-still application for image capture
	*/
#include "mchpcam_common.h"
#include "mchpcam_image_processor.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <fstream>
#include <vector>
#include <getopt.h>
#include <jpeglib.h>
#include <png.h>
#include <sys/mman.h>
#include <cstring>

using namespace libcamera;
using namespace std::chrono_literals;

void printUsage(const char *argv0);

class MchpCamStill : public MchpCamCommon {
public:
	static MchpCamStill* getInstance() {
	static MchpCamStill instance;
	return &instance;
	}
	void setImageFormat(const std::string &format) {
	imageFormat_ = format;
	}
	void setAWBMode(int mode) {
	awbMode_ = mode;
	std::cout << "AWB mode set to " << mode << std::endl;
	}
	int captureStill(const std::string &filename);
	void setEnableSoftwareProcessing(bool enable) { enableSoftwareProcessing_ = enable; }
	void setRawCapture(bool enable) { rawCapture_ = enable; }

	/* New methods for algorithm control */
	void setEnableAGC(bool enable) { enableAGC_ = enable; }
	void setEnableBLC(bool enable) { enableBLC_ = enable; }
	void setEnableAWB(bool enable) { enableAWB_ = enable; }
	void setEnableCCM(bool enable) { enableCCM_ = enable; }
	void setEnableAllProcessing(bool enable) {
	enableAGC_ = enableBLC_ = enableAWB_ = enableCCM_ = enable;
	}

protected:
	void saveFrame(const FrameBuffer *buffer, const std::string &filename) override;

private:
	MchpCamStill() : MchpCamCommon(),
	imageFormat_("jpeg"),
	enableSoftwareProcessing_(false),
	rawCapture_(false),
	jpeg_quality_(95),
	png_compression_(6),
	enableAGC_(true),
	enableBLC_(false),
	enableAWB_(true),
	enableCCM_(true) {}
	std::string imageFormat_;
	bool enableSoftwareProcessing_;
	bool rawCapture_;
	int awbMode_ = 0; /* Default to auto (WB_AUTO) */


	/* Algorithm control flags */
	bool enableAGC_;
	bool enableBLC_;
	bool enableAWB_;
	bool enableCCM_;
	mchpcam::ImageProcessingParams processingParams_;

	void saveJpeg(const unsigned char *data, int width, int height, const std::string &filename);
	void savePng(const unsigned char *data, int width, int height, const std::string &filename);
	void saveRaw(const unsigned char *data, size_t size, const std::string &filename);
};

int MchpCamStill::captureStill(const std::string &filename)
{
	if (requests_.empty() || !requests_[0]) {
	std::cerr << "No valid request available" << std::endl;
	return -EINVAL;
	}

	/* Set up controls */
	ControlList controls(camera_->controls());

	/* Add other basic controls using the setControl helper function */
	auto setControl = [&](const ControlId *id, auto value) {
	if (camera_->controls().count(id)) {
	controls.set(id->id(), ControlValue(value));
	}
	};

	/* Basic controls */
	setControl(&controls::Brightness, brightness_);
	setControl(&controls::Contrast, contrast_);
	setControl(&controls::Gamma, gamma_);
  MchpCamCommon::applyAWBDefaults(controls, awbParams_);

	int ret = camera_->start(&controls);
	if (ret < 0) {
	std::cerr << "Failed to start camera: " << ret << std::endl;
	return ret;
	}

	ret = camera_->queueRequest(requests_[0].get());
	if (ret < 0) {
	std::cerr << "Failed to queue request: " << ret << std::endl;
	camera_->stop();
	return ret;
	}

	/* Wait for capture to complete - optimized for speed but enough time for processing */
	std::cout << "Capturing image..." << std::endl;
	/* Use shorter wait times for faster capture */
	auto wait_time = width_ >= 1920 ? std::chrono::milliseconds(1500) : std::chrono::milliseconds(800);
	std::this_thread::sleep_for(wait_time);

	camera_->stop();

	/* Find and save capture buffer */
	const auto &buffers = requests_[0]->buffers();
	for (const auto &[stream, buffer] : buffers) {
	if (stream == stream_) {
	std::cout << "Captured image with resolution: "
	<< width_ << "x" << height_ << std::endl;

	/* Check for AGC/BLC parameters in request metadata */
	const ControlList &metadata = requests_[0]->metadata();

	/* Use hardcoded constants */
	constexpr uint32_t AUTO_GAIN_ID = 0x009819d1;
	constexpr uint32_t BLACK_LEVEL_ID = 0x009819d0;

	/* Check for AUTO_GAIN_ID */
	if (metadata.contains(AUTO_GAIN_ID)) {
	processingParams_.gainValue = metadata.get(AUTO_GAIN_ID).get<int32_t>();
	}

	/* Check for BLACK_LEVEL_ID */
	if (metadata.contains(BLACK_LEVEL_ID)) {
	processingParams_.blackLevel = metadata.get(BLACK_LEVEL_ID).get<int32_t>();
	}

	saveFrame(buffer, filename);
	break;
	}
	}

	return 0;
}

void MchpCamStill::saveFrame(const FrameBuffer *buffer, const std::string &filename)
{
	const FrameBuffer::Plane &plane = buffer->planes()[0];
	void *mappedMemory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), plane.offset);
	if (mappedMemory == MAP_FAILED) {
	std::cerr << "Failed to map memory" << std::endl;
	return;
	}
	const unsigned char *planeData = static_cast<const unsigned char*>(mappedMemory);

	/* If raw capture is enabled, save the raw data directly without processing */
	if (rawCapture_) {
	saveRaw(static_cast<const unsigned char*>(mappedMemory), plane.length, filename);
	munmap(mappedMemory, plane.length);
	return;
	}

	/* Apply software processing if enabled */
	if (enableSoftwareProcessing_) {
	/* Check for metadata from the request */
	const ControlList &metadata = requests_[0]->metadata();

	/* Setup processing parameters */
	mchpcam::ImageProcessingParams processingParams;
	bool hasParams = false;

	/* Configure which algorithms to enable */
	processingParams.enableAGC = enableAGC_;
	processingParams.enableBLC = enableBLC_;
	processingParams.enableAWB = enableAWB_;
	processingParams.enableCCM = enableCCM_;

	/* Set AWB mode */
	processingParams.awbMode = awbMode_;

	/* Get AGC gain */
	constexpr uint32_t AUTO_GAIN_ID = 0x009819d1;
	if (metadata.contains(AUTO_GAIN_ID)) {
	processingParams.gainValue = metadata.get(AUTO_GAIN_ID).get<int32_t>();
	hasParams = true;
	}

	/* Get BLC level */
	constexpr uint32_t BLACK_LEVEL_ID = 0x009819d0;
	if (metadata.contains(BLACK_LEVEL_ID)) {
	processingParams.blackLevel = metadata.get(BLACK_LEVEL_ID).get<int32_t>();
	hasParams = true;
	}

	/* Get AWB parameters - use the correct control IDs */
	constexpr uint32_t RED_GAIN_ID = 0x009819c0;
	constexpr uint32_t BLUE_GAIN_ID = 0x009819c1;
	constexpr uint32_t GREEN_RED_GAIN_ID = 0x009819c2;
	constexpr uint32_t GREEN_BLUE_GAIN_ID = 0x009819c3;
	constexpr uint32_t RED_OFFSET_ID = 0x009819c4;
	constexpr uint32_t BLUE_OFFSET_ID = 0x009819c5;
	constexpr uint32_t GREEN_RED_OFFSET_ID = 0x009819c6;
	constexpr uint32_t GREEN_BLUE_OFFSET_ID = 0x009819c7;

	/* Extract AWB gains if available */
	if (metadata.contains(RED_GAIN_ID)) {
	processingParams.awbParams.redGain = metadata.get(RED_GAIN_ID).get<int32_t>();
	hasParams = true;
	}

	if (metadata.contains(BLUE_GAIN_ID)) {
	processingParams.awbParams.blueGain = metadata.get(BLUE_GAIN_ID).get<int32_t>();
	hasParams = true;
	}

	if (metadata.contains(GREEN_RED_GAIN_ID)) {
	processingParams.awbParams.greenRedGain = metadata.get(GREEN_RED_GAIN_ID).get<int32_t>();
	hasParams = true;
	}

	if (metadata.contains(GREEN_BLUE_GAIN_ID)) {
	processingParams.awbParams.greenBlueGain = metadata.get(GREEN_BLUE_GAIN_ID).get<int32_t>();
	hasParams = true;
	}

	/* Extract AWB offsets if available */
	if (metadata.contains(RED_OFFSET_ID)) {
	processingParams.awbParams.redOffset = metadata.get(RED_OFFSET_ID).get<int32_t>();
	hasParams = true;
	}

	if (metadata.contains(BLUE_OFFSET_ID)) {
	processingParams.awbParams.blueOffset = metadata.get(BLUE_OFFSET_ID).get<int32_t>();
	hasParams = true;
	}

	if (metadata.contains(GREEN_RED_OFFSET_ID)) {
	processingParams.awbParams.greenRedOffset = metadata.get(GREEN_RED_OFFSET_ID).get<int32_t>();
	hasParams = true;
	}

	if (metadata.contains(GREEN_BLUE_OFFSET_ID)) {
	processingParams.awbParams.greenBlueOffset = metadata.get(GREEN_BLUE_OFFSET_ID).get<int32_t>();
	hasParams = true;
	}

	/* Get CCM parameters */
	constexpr std::array<uint32_t, 9> CCM_COEFF_IDS = {
	0x009819e0, 0x009819e1, 0x009819e2,  /* CCM_COEFF_00_ID, CCM_COEFF_01_ID, CCM_COEFF_02_ID */
	0x009819e3, 0x009819e4, 0x009819e5,  /* CCM_COEFF_10_ID, CCM_COEFF_11_ID, CCM_COEFF_12_ID */
	0x009819e6, 0x009819e7, 0x009819e8	 /* CCM_COEFF_20_ID, CCM_COEFF_21_ID, CCM_COEFF_22_ID */
	};

	constexpr std::array<uint32_t, 3> CCM_OFFSET_IDS = {
	0x009819e9, 0x009819ea, 0x009819eb	/* CCM_OFFSET_R_ID, CCM_OFFSET_G_ID, CCM_OFFSET_B_ID */
	};

	/* Check if at least one CCM coefficient exists */
	if (metadata.contains(CCM_COEFF_IDS[0])) {
	/* Get all CCM coefficients */
	for (size_t i = 0; i < CCM_COEFF_IDS.size(); i++) {
	if (metadata.contains(CCM_COEFF_IDS[i])) {
	processingParams.ccmMatrix[i] = metadata.get(CCM_COEFF_IDS[i]).get<int32_t>();
	}
	}

	/* Get CCM offsets */
	for (size_t i = 0; i < CCM_OFFSET_IDS.size(); i++) {
	if (metadata.contains(CCM_OFFSET_IDS[i])) {
	processingParams.ccmOffset[i] = metadata.get(CCM_OFFSET_IDS[i]).get<int32_t>();
	}
	}

	/* Get color temperature if available */
	constexpr uint32_t SCENE_COLOR_CORRECTION_ID = 0x009819d2;
	if (metadata.contains(SCENE_COLOR_CORRECTION_ID)) {
	processingParams.colorTemperature =
	metadata.get(SCENE_COLOR_CORRECTION_ID).get<int32_t>();
	}

	hasParams = true;
	}

	/* Apply all processing to the image data if we have parameters */
	if (hasParams) {
	mchpcam::ImageProcessor::applySoftwareProcessing(
	processedData.data(), width_, height_, processingParams);
	} else {
	std::cout << "No processing parameters found in metadata" << std::endl;
	}
	}

	/* Buffer for RGB888 data (used for PNG/JPEG output) */
	std::vector<uint8_t> rgbBuffer(width_ * height_ * 3);

	if (pixelFormat_ == formats::YUYV) {
		/* Convert YUYV to RGB */
		for (unsigned int y = 0; y < height_; ++y) {
			for (unsigned int x = 0; x < width_; x += 2) {
				int y0 = planeData[y * width_ * 2 + x * 2];
				int u = planeData[y * width_ * 2 + x * 2 + 1];
				int y1 = planeData[y * width_ * 2 + x * 2 + 2];
				int v = planeData[y * width_ * 2 + x * 2 + 3];

				rgbBuffer[(y * width_ + x) * 3] = std::clamp(y0 + 1.402 * (v - 128), 0.0, 255.0);
				rgbBuffer[(y * width_ + x) * 3 + 1] = std::clamp(y0 - 0.344 * (u - 128) - 0.714 * (v - 128), 0.0, 255.0);
				rgbBuffer[(y * width_ + x) * 3 + 2] = std::clamp(y0 + 1.772 * (u - 128), 0.0, 255.0);

				rgbBuffer[(y * width_ + x + 1) * 3] = std::clamp(y1 + 1.402 * (v - 128), 0.0, 255.0);
				rgbBuffer[(y * width_ + x + 1) * 3 + 1] = std::clamp(y1 - 0.344 * (u - 128) - 0.714 * (v - 128), 0.0, 255.0);
				rgbBuffer[(y * width_ + x + 1) * 3 + 2] = std::clamp(y1 + 1.772 * (u - 128), 0.0, 255.0);
			}
		}
	} else if (pixelFormat_ == formats::RGB565) {
		/* Convert RGB565 to RGB888 */
		for (unsigned int y = 0; y < height_; ++y) {
			for (unsigned int x = 0; x < width_; ++x) {
				uint16_t pixel = (planeData[(y * width_ + x) * 2 + 1] << 8) |
					planeData[(y * width_ + x) * 2];
				rgbBuffer[(y * width_ + x) * 3] = ((pixel >> 11) & 0x1F) << 3;     /* Red */
				rgbBuffer[(y * width_ + x) * 3 + 1] = ((pixel >> 5) & 0x3F) << 2;  /* Green */
				rgbBuffer[(y * width_ + x) * 3 + 2] = (pixel & 0x1F) << 3;         /* Blue */
			}
		}
	}

	if (imageFormat_ == "jpeg") {
		saveJpeg(rgbBuffer.data(), width_, height_, filename);
	} else if (imageFormat_ == "png") {
		savePng(rgbBuffer.data(), width_, height_, filename);
	} else {
		std::cerr << "Unsupported image format: " << imageFormat_ << std::endl;
	}

	munmap(mappedMemory, plane.length);
}

void MchpCamStill::saveRaw(const unsigned char *data, size_t size, const std::string &filename)
{
	std::string outfilename = filename;

	/* Ensure .raw extension for raw files */
	if (outfilename.find(".raw") == std::string::npos) {
	outfilename += ".raw";
	}

	std::ofstream file(outfilename, std::ios::binary);
	if (!file) {
	std::cerr << "Can't open output file for raw data" << std::endl;
	return;
	}

	file.write(reinterpret_cast<const char*>(data), size);
	file.close();

	std::cout << "Saved raw data to " << outfilename
	<< " (" << size << " bytes)" << std::endl;
}

void MchpCamStill::saveJpeg(const unsigned char *data, int width, int height, const std::string &filename)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (!file) {
	std::cerr << "Can't open output file for JPEG image" << std::endl;
	return;
	}

	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, file);

	cinfo.image_width = width;
	cinfo.image_height = height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, 90, TRUE);
	jpeg_start_compress(&cinfo, TRUE);

	JSAMPROW row_pointer[1];
	while (cinfo.next_scanline < cinfo.image_height) {
	row_pointer[0] = const_cast<unsigned char*>(&data[cinfo.next_scanline * width * 3]);
	jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	jpeg_finish_compress(&cinfo);
	fclose(file);
	jpeg_destroy_compress(&cinfo);

void MchpCamStill::savePng(const unsigned char *data, int width, int height, const std::string &filename)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (!file) {
	std::cerr << "Can't open output file for PNG image" << std::endl;
	return;
	}

	png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
	if (!png) {
	std::cerr << "Failed to create PNG write struct" << std::endl;
	fclose(file);
	return;
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
	std::cerr << "Failed to create PNG info struct" << std::endl;
	png_destroy_write_struct(&png, nullptr);
	fclose(file);
	return;
	}

	if (setjmp(png_jmpbuf(png))) {
	std::cerr << "Error writing PNG file" << std::endl;
	png_destroy_write_struct(&png, &info);
	fclose(file);
	return;
	}

	png_init_io(png, file);
	png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
	png_write_info(png, info);

	for (int y = 0; y < height; ++y) {
		png_write_row(png, const_cast<unsigned char*>(&data[y * width * 3]));
	}

	png_write_end(png, nullptr);
	png_destroy_write_struct(&png, &info);
	fclose(file);
}

static MchpCamStill *app = nullptr;

static void sigHandler([[maybe_unused]]int signal)
{
	if (app)
	app->stop();
	exit(0);
}

void printUsage(const char *argv0)
{
       std::cout << "Usage: " << argv0 << " [options]" << std::endl;
       std::cout << "Options:" << std::endl;
       std::cout << "  -o, --output=FILE              Output file name (default: output.jpg)" << std::endl;
       std::cout << "  -w, --width=WIDTH              Set capture width" << std::endl;
       std::cout << "  -h, --height=HEIGHT            Set capture height" << std::endl;
       std::cout << "  -f, --format=FORMAT            Set pixel format (YUYV, RGB565)" << std::endl;
       std::cout << "  -i, --image-format=FORMAT      Set image format (jpeg, png)" << std::endl;
       std::cout << "  -c, --camera=ID                Set camera ID" << std::endl;
       std::cout << "  -b, --brightness=VALUE         Set brightness (-100 to 100)" << std::endl;
       std::cout << "  -n, --contrast=VALUE           Set contrast (0 to 100)" << std::endl;
       std::cout << "  -a, --white-balance-auto=VALUE Set white balance automatic (0 or 1)" << std::endl;
       std::cout << "  -g, --gamma=VALUE              Set gamma (0 to 500)" << std::endl;
       std::cout << "  -r, --red-gain=VALUE           Set red component gain (0 to 8191)" << std::endl;
       std::cout << "  -m, --blue-gain=VALUE          Set blue component gain (0 to 8191)" << std::endl;
       std::cout << "  -v, --green-red-gain=VALUE     Set green-red component gain (0 to 8191)" << std::endl;
       std::cout << "  -q, --green-blue-gain=VALUE    Set green-blue component gain (0 to 8191)" << std::endl;
       std::cout << "  -R, --red-offset=VALUE         Set red component offset (-4095 to 4095)" << std::endl;
       std::cout << "  -M, --blue-offset=VALUE        Set blue component offset (-4095 to 4095)" << std::endl;
       std::cout << "  -V, --green-red-offset=VALUE   Set green-red component offset (-4095 to 4095)" << std::endl;
       std::cout << "  -Q, --green-blue-offset=VALUE  Set green-blue component offset (-4095 to 4095)" << std::endl;
       std::cout << "  -H, --help                     Print this help message" << std::endl;
}

int main(int argc, char **argv)
{
	app = MchpCamStill::getInstance();
	static struct option long_options[] = {
               {"output",               required_argument, 0, 'o'},
               {"width",                required_argument, 0, 'w'},
               {"height",               required_argument, 0, 'h'},
               {"format",               required_argument, 0, 'f'},
               {"camera",               required_argument, 0, 'c'},
               {"brightness",           required_argument, 0, 'b'},
               {"contrast",             required_argument, 0, 'n'},
               {"white-balance-auto",   required_argument, 0, 'a'},
               {"gamma",                required_argument, 0, 'g'},
               {"red-gain",             required_argument, 0, 'r'},
               {"blue-gain",            required_argument, 0, 'm'},
               {"green-red-gain",       required_argument, 0, 'v'},
               {"green-blue-gain",      required_argument, 0, 'q'},
               {"red-offset",           required_argument, 0, 'R'},
               {"blue-offset",          required_argument, 0, 'M'},
               {"green-red-offset",     required_argument, 0, 'V'},
               {"green-blue-offset",    required_argument, 0, 'Q'},
               {"help",                 no_argument,       0, 'H'},
               {0, 0, 0, 0}
        };

	int option_index = 0;
	int c;
	std::string output = "output.jpg";
	std::string cameraId;
	unsigned int width = 0, height = 0;

	while ((c = getopt_long(argc, argv, "o:w:h:f:c:b:n:a:g:r:m:v:R:M:V:Q:H", long_options, &option_index)) != -1) {
		switch (c) {
		case 'o':
			output = optarg;
			break;
		case 'w':
			width = std::stoi(optarg);
			break;
		case 'h':
			height = std::stoi(optarg);
			break;
		case 'f':
			app->setFormat(optarg);
			break;
		case 'c':
			cameraId = optarg;
			break;
		case 'b':
			app->setBrightness(std::stoi(optarg));
			break;
		case 'n':
			app->setContrast(std::stoi(optarg));
			break;
		case 'a':
			app->setWhiteBalanceAutomatic(std::stoi(optarg) != 0);
			break;
		case 'g':
			app->setGamma(std::stoi(optarg));
			break;
		case 'r':
                        app->setRedGain(std::stoi(optarg));  // Min: 0, Max: 8191
                        break;
                case 'm':
                        app->setBlueGain(std::stoi(optarg));  // Min: 0, Max: 8191
                        break;
                case 'v':
                        app->setGreenRedGain(std::stoi(optarg));  // Min: 0, Max: 8191
                        break;
                case 'q':
                        app->setGreenBlueGain(std::stoi(optarg));  // Min: 0, Max: 8191
                        break;
                case 'R':
                        app->setRedOffset(std::stoi(optarg));  // Min: -4095, Max: 4095
                        break;
                case 'M':
                        app->setBlueOffset(std::stoi(optarg));  // Min: -4095, Max: 4095
                        break;
                case 'V':
                        app->setGreenRedOffset(std::stoi(optarg));  // Min: -4095, Max: 4095
                        break;
                case 'Q':
                        app->setGreenBlueOffset(std::stoi(optarg));  // Min: -4095, Max: 4095
                        break;
		case 'H':
		default:
			printUsage(argv[0]);
			if (app)
				app->stop();

			exit(0);
		}
	}

	if (width != 0 && height != 0) {
	app->setResolution(width, height);
	}

	std::cout << "Requested resolution: " << app->getWidth() << "x" << app->getHeight() << std::endl;

	if (app->init(cameraId) < 0)
	return -1;

	signal(SIGINT, sigHandler);
	app->captureStill(output);

	return 0;
}
