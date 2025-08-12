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
	void setJpegQuality(int quality) { jpeg_quality_ = std::clamp(quality, 1, 100); }
	void setPngCompression(int level) { png_compression_ = std::clamp(level, 0, 9); }

	/* New methods for algorithm control */
	void setEnableAGC(bool enable) { enableAGC_ = enable; }
	void setEnableAWB(bool enable) { enableAWB_ = enable; }
	void setEnableCCM(bool enable) { enableCCM_ = enable; }
	void setEnableAllProcessing(bool enable) {
		enableAGC_ = enableAWB_ = enableCCM_ = enable;
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
	int jpeg_quality_;
	int png_compression_;
	int awbMode_ = 0; /* Default to auto (WB_AUTO) */

	/* Algorithm control flags */
	bool enableAGC_;
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

			/* Check for AGC parameters in request metadata */
			const ControlList &metadata = requests_[0]->metadata();
			std::cout << "Metadata size: " << metadata.size() << std::endl;
			saveFrame(buffer, filename);
			break;
		}
	}

	return 0;
}

void MchpCamStill::saveFrame(const FrameBuffer *buffer, const std::string &filename)
{
	const FrameBuffer::Plane &plane = buffer->planes()[0];

	std::cout << "  Buffer debug info:" << std::endl;
	std::cout << "  Plane length: " << plane.length << " bytes" << std::endl;
	std::cout << "  Expected size: " << (width_ * height_ * 2) << " bytes (RGB565)" << std::endl;
	std::cout << "  Resolution: " << width_ << "x" << height_ << std::endl;

	// Validate buffer size
	size_t expectedSize = width_ * height_ * 2;  /* RGB565 = 2 bytes per pixel */
	if (plane.length < expectedSize) {
		std::cerr << "  Buffer too small! Expected: " << expectedSize
			<< ", got: " << plane.length << std::endl;
		return;
	}

	void *mappedMemory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), plane.offset);

	if (mappedMemory == MAP_FAILED) {
		std::cerr << " Failed to map memory: " << strerror(errno) << std::endl;
		return;
	}

	std::cout << " Memory mapped successfully" << std::endl;
	std::cout << " Hardware AWB Mode - Using ISC processed pixels directly" << std::endl;

	/* For raw capture - save directly without any processing */
	if (rawCapture_) {
		std::cout << " Saving raw data (hardware processed)" << std::endl;
		saveRaw(static_cast<const unsigned char*>(mappedMemory), plane.length, filename);
		munmap(mappedMemory, plane.length);
		return;
	}

	std::cout << " Converting format: " << pixelFormat_.toString() << " → RGB888" << std::endl;

	/* Validate pixel format */
	if (pixelFormat_ != formats::RGB565 && pixelFormat_ != formats::YUYV) {
		std::cerr << " Unsupported pixel format: " << pixelFormat_.toString() << std::endl;
		munmap(mappedMemory, plane.length);
		return;
	}

	/* Create RGB888 buffer with extra safety margin */
	size_t rgbBufferSize = width_ * height_ * 3;
	std::vector<uint8_t> rgbBuffer;

	try {
		rgbBuffer.resize(rgbBufferSize);
		std::cout << " RGB buffer allocated: " << rgbBufferSize << " bytes" << std::endl;
	} catch (const std::exception& e) {
		std::cerr << " Failed to allocate RGB buffer: " << e.what() << std::endl;
		munmap(mappedMemory, plane.length);
		return;
	}

	const uint8_t* data = static_cast<const uint8_t*>(mappedMemory);

	if (pixelFormat_ == formats::RGB565) {
		std::cout << " Converting RGB565 → RGB888 (hardware AWB already applied)" << std::endl;

		// Safe conversion with bounds checking
		for (unsigned int y = 0; y < height_; ++y) {
			for (unsigned int x = 0; x < width_; ++x) {
				/* Calculate indices with bounds checking */
				size_t pixelIndex = (y * width_ + x) * 2;
				size_t rgbIndex = (y * width_ + x) * 3;

				/* Bounds check for input buffer */
				if (pixelIndex + 1 >= plane.length) {
					std::cerr << " Input buffer overflow at pixel (" << x << "," << y << ")" << std::endl;
					munmap(mappedMemory, plane.length);
					return;
				}

				/* Bounds check for output buffer */
				if (rgbIndex + 2 >= rgbBufferSize) {
					std::cerr << " Output buffer overflow at pixel (" << x << "," << y << ")" << std::endl;
					munmap(mappedMemory, plane.length);
					return;
				}

				/* Read RGB565 pixel safely */
				uint16_t pixel = data[pixelIndex] | (data[pixelIndex + 1] << 8);

				/* Extract RGB components */
				uint8_t r = (pixel >> 11) & 0x1F;  /* 5 bits */
				uint8_t g = (pixel >> 5) & 0x3F;   /* 6 bits */
				uint8_t b = pixel & 0x1F;          /* 5 bits */

				/* Expand to 8-bit */
				r = (r << 3) | (r >> 2);  /* 5→8 bit expansion */
				g = (g << 2) | (g >> 4);  /* 6→8 bit expansion */
				b = (b << 3) | (b >> 2);  /* 5→8 bit expansion */

				/* Store safely */
				rgbBuffer[rgbIndex] = r;
				rgbBuffer[rgbIndex + 1] = g;
				rgbBuffer[rgbIndex + 2] = b;
			}

		}

	} else if (pixelFormat_ == formats::YUYV) {
		std::cout << " Converting YUYV → RGB888 (hardware AWB already applied)" << std::endl;

		/* YUYV conversion with bounds checking */
		for (unsigned int y = 0; y < height_; ++y) {
			for (unsigned int x = 0; x < width_; x += 2) {
				size_t yuvIndex = (y * width_ + x) * 2;
				size_t rgb0Index = (y * width_ + x) * 3;
				size_t rgb1Index = (y * width_ + x + 1) * 3;

				/* Bounds checks */
				if (yuvIndex + 3 >= plane.length ||
						rgb1Index + 2 >= rgbBufferSize) {
					std::cerr << " Buffer overflow in YUYV conversion" << std::endl;
					munmap(mappedMemory, plane.length);
					return;
				}

				/* Read YUYV safely */
				int y0 = data[yuvIndex];
				int u = data[yuvIndex + 1];
				int y1 = data[yuvIndex + 2];
				int v = data[yuvIndex + 3];

				/* Convert with clamping */
				auto clamp = [](float val) -> uint8_t {
					return static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, val)));
				};

				rgbBuffer[rgb0Index] = clamp(y0 + 1.402f * (v - 128));
				rgbBuffer[rgb0Index + 1] = clamp(y0 - 0.344f * (u - 128) - 0.714f * (v - 128));
				rgbBuffer[rgb0Index + 2] = clamp(y0 + 1.772f * (u - 128));

				rgbBuffer[rgb1Index] = clamp(y1 + 1.402f * (v - 128));
				rgbBuffer[rgb1Index + 1] = clamp(y1 - 0.344f * (u - 128) - 0.714f * (v - 128));
				rgbBuffer[rgb1Index + 2] = clamp(y1 + 1.772f * (u - 128));
			}
		}
	}

	std::cout << " Format conversion completed successfully" << std::endl;

	std::string outfilename = filename;

	try {
		if (imageFormat_ == "jpeg" || imageFormat_ == "jpg") {
			if (outfilename.find(".jpg") == std::string::npos &&
					outfilename.find(".jpeg") == std::string::npos) {
				outfilename += ".jpg";
			}
			std::cout << " Saving JPEG: " << outfilename << std::endl;
			saveJpeg(rgbBuffer.data(), width_, height_, outfilename);

		} else if (imageFormat_ == "png") {
			if (outfilename.find(".png") == std::string::npos) {
				outfilename += ".png";
			}
			std::cout << " Saving PNG: " << outfilename << std::endl;
			savePng(rgbBuffer.data(), width_, height_, outfilename);

		} else {
			if (outfilename.find(".png") == std::string::npos &&
					outfilename.find(".jpg") == std::string::npos &&
					outfilename.find(".jpeg") == std::string::npos) {
				outfilename += ".png";
			}
			std::cout << " Saving PNG (default): " << outfilename << std::endl;
			savePng(rgbBuffer.data(), width_, height_, outfilename);
		}

		std::cout << " File saved successfully!" << std::endl;

	} catch (const std::exception& e) {
		std::cerr << " Error during file save: " << e.what() << std::endl;
	}

	munmap(mappedMemory, plane.length);
	std::cout << " Hardware AWB test completed!" << std::endl;
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

	/* Set quality based on user preference */
	jpeg_set_quality(&cinfo, jpeg_quality_, TRUE);

	/* Set to progressive JPEG for better quality perception */
	jpeg_simple_progression(&cinfo);

	jpeg_start_compress(&cinfo, TRUE);

	JSAMPROW row_pointer[1];
	while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer[0] = const_cast<unsigned char*>(&data[cinfo.next_scanline * width * 3]);
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	jpeg_finish_compress(&cinfo);
	fclose(file);
	jpeg_destroy_compress(&cinfo);

	std::cout << "Saved JPEG image to " << filename
		  << " with quality " << jpeg_quality_ << std::endl;
}
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

	/* Set PNG compression level to user-defined value */
	png_set_compression_level(png, png_compression_);

	/* Use RGB color type for high quality */
	png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB,
		     PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
		     PNG_FILTER_TYPE_DEFAULT);

	/* Add minimal metadata */
	png_text text_ptr[1];
	char key0[] = "Software";
	char text0[] = "Microchip Camera";
	text_ptr[0].key = key0;
	text_ptr[0].text = text0;
	text_ptr[0].compression = PNG_TEXT_COMPRESSION_NONE;

	png_set_text(png, info, text_ptr, 1);
	png_write_info(png, info);

	/* Write image data row by row for memory efficiency */
	for (int y = 0; y < height; ++y) {
		png_write_row(png, const_cast<png_bytep>(&data[y * width * 3]));
	}

	png_write_end(png, nullptr);
	png_destroy_write_struct(&png, &info);
	fclose(file);

	std::cout << "Saved PNG image to " << filename
		  << " with compression level " << png_compression_ << std::endl;
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
	std::cout << "Camera options:" << std::endl;
	std::cout << "	-c, --camera=CAMERA          Camera index or name" << std::endl;
	std::cout << "	-o, --output=FILE            Output file name (default: output.jpg)" << std::endl;
	std::cout << "	-f, --format=FORMAT          Set pixel format (YUYV, RGB565)" << std::endl;
	std::cout << "	--width=WIDTH                Set capture width" << std::endl;
	std::cout << "	--height=HEIGHT              Set capture height" << std::endl;

	std::cout << "\nImage options:" << std::endl;
	std::cout << "	--jpeg-quality=VALUE         Set JPEG quality (1-100, default: 95)" << std::endl;
	std::cout << "	--png-compression=VALUE      Set PNG compression level (0-9, default: 6)" << std::endl;
	std::cout << "	--raw                        Save raw sensor data instead of processed image" << std::endl;
	std::cout << "	--image-format=FORMAT        Set output image format (jpeg, png, raw)" << std::endl;

	std::cout << "\nImage processing options:" << std::endl;
	std::cout << "	--brightness=VALUE           Set brightness (-100 to 100)" << std::endl;
	std::cout << "	--contrast=VALUE             Set contrast (0 to 100)" << std::endl;
	std::cout << "	--gamma=VALUE                Set gamma (0 to 2)" << std::endl;
	std::cout << "	--enable-processing          Enable software image processing" << std::endl;

	std::cout << "\nWhite balance options:" << std::endl;
	std::cout << "	--awb=MODE                   Set white balance mode (auto, daylight, cloudy, tungsten, fluorescent, shade, manual)" << std::endl;
	std::cout << "	--awb-enable                 Enable auto white balance" << std::endl;
	std::cout << "	--red-gain=VALUE             Set red component gain (0 to 8191)" << std::endl;
	std::cout << "	--blue-gain=VALUE            Set blue component gain (0 to 8191)" << std::endl;
	std::cout << "	--green-red-gain=VALUE       Set green-red component gain (0 to 8191)" << std::endl;
	std::cout << "	--green-blue-gain=VALUE      Set green-blue component gain (0 to 8191)" << std::endl;
	std::cout << "	--red-offset=VALUE           Set red component offset (-4095 to 4095)" << std::endl;
	std::cout << "	--blue-offset=VALUE          Set blue component offset (-4095 to 4095)" << std::endl;
	std::cout << "	--green-red-offset=VALUE     Set green-red component offset (-4095 to 4095)" << std::endl;
	std::cout << "	--green-blue-offset=VALUE    Set green-blue component offset (-4095 to 4095)" << std::endl;

	std::cout << "\nAlgorithm options:" << std::endl;
	std::cout << "	--enable-agc                 Enable Automatic Gain Control" << std::endl;
	std::cout << "	--enable-ccm                 Enable Color Correction Matrix" << std::endl;
	std::cout << "	--enable-awb                 Enable Software Auto white balance processing" << std::endl;
	std::cout << "	--enable-all                 Enable all image processing algorithms" << std::endl;

	std::cout << "\nGeneral options:" << std::endl;
	std::cout << "	-h, --help                   Print this help message" << std::endl;
}

int main(int argc, char **argv)
{
	app = MchpCamStill::getInstance();
	static struct option long_options[] = {
		{"camera",              required_argument, 0, 'c'},
		{"output",	        required_argument, 0, 'o'},
		{"format",	        required_argument, 0, 'f'},
		{"width", 	        required_argument, 0, 0},
		{"height",              required_argument, 0, 0},
		{"jpeg-quality",        required_argument, 0, 0},
		{"png-compression",     required_argument, 0, 0},
		{"raw",		        no_argument, 0, 0},
		{"image-format",        required_argument, 0, 0},
		{"brightness",	        required_argument, 0, 0},
		{"contrast",	        required_argument, 0, 0},
		{"gamma",	        required_argument, 0, 0},
		{"enable-processing",   no_argument, 0, 0},
		{"awb",		        required_argument, 0, 0},
		{"awb-enable",	        no_argument, 0, 0},
		{"red-gain",	        required_argument, 0, 0},
		{"blue-gain",	        required_argument, 0, 0},
		{"green-red-gain",      required_argument, 0, 0},
		{"green-blue-gain",     required_argument, 0, 0},
		{"red-offset",	        required_argument, 0, 0},
		{"blue-offset",	        required_argument, 0, 0},
		{"green-red-offset",    required_argument, 0, 0},
		{"green-blue-offset",   required_argument, 0, 0},
		{"enable-agc",	        no_argument, 0, 0},
		{"enable-ccm",	        no_argument, 0, 0},
		{"enable-awb",	        no_argument, 0, 0},
		{"enable-all",	        no_argument, 0, 0},
		{"help",                no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};
	int option_index = 0;
	int c;
	const char* option_name;
	std::string output = "output.jpg";	/* Default to jpeg */
	std::string cameraId;
	unsigned int width = 0, height = 0;

	while ((c = getopt_long(argc, argv, "c:o:f:h", long_options, &option_index)) != -1) {
		switch (c) {
		case 'c':
			cameraId = optarg;
			break;
		case 'o':
			output = optarg;
			break;
		case 'f':
			app->setFormat(optarg);
			break;
		case 'h':
			printUsage(argv[0]);
			if (app)
				app->stop();
			exit(0);
			break;
		case 0:
			/* Handle long options without short equivalents */
			option_name = long_options[option_index].name;

			if (strcmp(option_name, "width") == 0) {
				width = std::stoi(optarg);
			} else if (strcmp(option_name, "height") == 0) {
				height = std::stoi(optarg);
			} else if (strcmp(option_name, "jpeg-quality") == 0) {
				app->setJpegQuality(std::stoi(optarg));
			} else if (strcmp(option_name, "png-compression") == 0) {
				app->setPngCompression(std::stoi(optarg));
			} else if (strcmp(option_name, "raw") == 0) {
				app->setRawCapture(true);
			} else if (strcmp(option_name, "image-format") == 0) {
				app->setImageFormat(optarg);
			} else if (strcmp(option_name, "brightness") == 0) {
				app->setBrightness(std::stoi(optarg));
			} else if (strcmp(option_name, "contrast") == 0) {
				app->setContrast(std::stoi(optarg));
			} else if (strcmp(option_name, "gamma") == 0) {
				app->setGamma(std::stoi(optarg));
			} else if (strcmp(option_name, "enable-processing") == 0) {
				app->setEnableSoftwareProcessing(true);
			} else if (strcmp(option_name, "awb") == 0) {
				/* Handle AWB mode setting */
				std::string mode = optarg;
				if (mode == "auto")
					app->setAWBMode(0);
				else if (mode == "daylight" || mode == "sunny")
					app->setAWBMode(1);
				else if (mode == "cloudy")
					app->setAWBMode(2);
				else if (mode == "tungsten" || mode == "incandescent")
					app->setAWBMode(3);
				else if (mode == "fluorescent")
					app->setAWBMode(4);
				else if (mode == "shade")
					app->setAWBMode(5);
				else if (mode == "manual")
					app->setAWBMode(6);
				else
					std::cerr << "Unknown AWB mode: " << mode << ". Using auto." << std::endl;
			} else if (strcmp(option_name, "awb-enable") == 0) {
				app->setWhiteBalanceAutomatic(true);
			} else if (strcmp(option_name, "red-gain") == 0) {
				app->setRedGain(std::stoi(optarg));
			} else if (strcmp(option_name, "blue-gain") == 0) {
				app->setBlueGain(std::stoi(optarg));
			} else if (strcmp(option_name, "green-red-gain") == 0) {
				app->setGreenRedGain(std::stoi(optarg));
			} else if (strcmp(option_name, "green-blue-gain") == 0) {
				app->setGreenBlueGain(std::stoi(optarg));
			} else if (strcmp(option_name, "red-offset") == 0) {
				app->setRedOffset(std::stoi(optarg));
			} else if (strcmp(option_name, "blue-offset") == 0) {
				app->setBlueOffset(std::stoi(optarg));
			} else if (strcmp(option_name, "green-red-offset") == 0) {
				app->setGreenRedOffset(std::stoi(optarg));
			} else if (strcmp(option_name, "green-blue-offset") == 0) {
				app->setGreenBlueOffset(std::stoi(optarg));
			} else if (strcmp(option_name, "enable-agc") == 0) {
				app->setEnableAGC(true);
				app->setEnableSoftwareProcessing(true);
			} else if (strcmp(option_name, "enable-ccm") == 0) {
				app->setEnableCCM(true);
				app->setEnableSoftwareProcessing(true);
			} else if (strcmp(option_name, "enable-awb") == 0) {
				app->setEnableAWB(true);
				app->setEnableSoftwareProcessing(true);
			} else if (strcmp(option_name, "enable-all") == 0) {
				app->setEnableAllProcessing(true);
				app->setEnableSoftwareProcessing(true);
			}
			break;
		default:
			printUsage(argv[0]);
			if (app)
				app->stop();
			exit(0);
		}
	}

	/* Set resolution if provided */
	if (width != 0 && height != 0) {
		app->setResolution(width, height);
	}

	/* Set image format based on file extension if not explicitly specified */
	if (output.length() > 4) {
		std::string extension = output.substr(output.length() - 4);
		std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
		if (extension == ".png") {
			app->setImageFormat("png");
		} else if (extension == ".jpg" || extension == "jpeg") {
			app->setImageFormat("jpeg");
		} else if (extension == ".raw") {
			app->setImageFormat("raw");
			app->setRawCapture(true);
		}
	}

	std::cout << "Requested resolution: " << (width != 0 ? width : app->getWidth())
		  << "x" << (height != 0 ? height : app->getHeight()) << std::endl;

	if (app->init(cameraId) < 0)
		return -1;

	signal(SIGINT, sigHandler);
	app->captureStill(output);

	return 0;
}
