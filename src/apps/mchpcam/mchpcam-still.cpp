/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * mchpcam - mchpcam-still application for image capture
 */

#include "mchpcam_common.h"
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

	int captureStill(const std::string &filename);

protected:
	void saveFrame(const FrameBuffer *buffer, const std::string &filename) override;

private:
	MchpCamStill() : MchpCamCommon(), imageFormat_("jpeg") {}
	std::string imageFormat_;

	void saveJpeg(const unsigned char *data, int width, int height, const std::string &filename);
	void savePng(const unsigned char *data, int width, int height, const std::string &filename);
};

int MchpCamStill::captureStill(const std::string &filename)
{
	ControlList controls;

	auto setControl = [&](const ControlId *id, auto value) {
		if (camera_->controls().count(id)) {
			controls.set(id->id(), ControlValue(value));
		}
	};

	setControl(&controls::Brightness, brightness_);
	setControl(&controls::Contrast, contrast_);
	setControl(&controls::AwbEnable, whiteBalanceAutomatic_);
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

	std::this_thread::sleep_for(std::chrono::seconds(1));
	camera_->stop();

	const FrameBuffer *buffer = requests_[0]->buffers().begin()->second;
	if (!buffer) {
		std::cerr << "No buffer received" << std::endl;
		return -1;
	}

	std::cout << "Capturing image with resolution: " << width_ << "x" << height_ << std::endl;
	saveFrame(buffer, filename);

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

void MchpCamStill::saveJpeg(const unsigned char *data, int width, int height, const std::string &filename)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (!file) {
		std::cerr << "Can't open output file" << std::endl;
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
}

void MchpCamStill::savePng(const unsigned char *data, int width, int height, const std::string &filename)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (!file) {
		std::cerr << "Can't open output file" << std::endl;
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
	std::cout << "  -H, --help                     Print this help message" << std::endl;
}

int main(int argc, char **argv)
{
	app = MchpCamStill::getInstance();

	static struct option long_options[] = {
		{"output",   required_argument, 0, 'o'},
		{"width",    required_argument, 0, 'w'},
		{"height",   required_argument, 0, 'h'},
		{"format",   required_argument, 0, 'f'},
		{"camera",   required_argument, 0, 'c'},
		{"brightness", required_argument, 0, 'b'},
		{"contrast", required_argument, 0, 'n'},
		{"white-balance-auto", required_argument, 0, 'a'},
		{"gamma", required_argument, 0, 'g'},
		{"help",     no_argument,       0, 'H'},
		{0, 0, 0, 0}
	};

	int option_index = 0;
	int c;
	std::string output = "output.jpg";
	std::string cameraId;
	unsigned int width = 0, height = 0;

	while ((c = getopt_long(argc, argv, "o:w:h:f:c:b:n:a:g:H", long_options, &option_index)) != -1) {
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
		case 'H':
			printUsage(argv[0]);
			return 0;
		default:
			printUsage(argv[0]);
			return 1;
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
