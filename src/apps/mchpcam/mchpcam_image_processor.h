/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
 *
 * mchpcam - image processing utilities
 */
#ifndef MCHPCAM_IMAGE_PROCESSOR_H
#define MCHPCAM_IMAGE_PROCESSOR_H

#include <cstdint>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <iostream>
#include <array>
#include "mchpcam_common.h"

namespace mchpcam {
/* Structure to hold software image processing parameters */
struct ImageProcessingParams {
	/* Algorithm enable flags */
	bool enableAGC;
	bool enableBLC;
	bool enableAWB;
	bool enableCCM;

	/* AWB mode */
	int awbMode;

	/* AGC parameter */
	int gainValue;

	/* BLC parameter */
	int blackLevel;

	/* AWB parameters */
	AWBParameters awbParams;

	/* CCM parameters */
	std::array<int32_t, 9> ccmMatrix;
	std::array<int32_t, 3> ccmOffset;
	int colorTemperature;

	/* Default constructor with neutral values */
	ImageProcessingParams()
		: enableAGC(false), enableBLC(false), enableAWB(false), enableCCM(false),
		  awbMode(0), /* Default to auto */
		  gainValue(1024), blackLevel(0),
		  colorTemperature(5500)
		{
			ccmMatrix.fill(0);
			ccmMatrix[0] = ccmMatrix[4] = ccmMatrix[8] = 1024; /* Identity matrix */
			ccmOffset.fill(0);
		}
};

class ImageProcessor {
public:
	/* Apply software AGC, BLC, AWB and CCM to image data */
	static void applySoftwareProcessing(uint8_t* data, unsigned int width, unsigned int height,
					    const ImageProcessingParams& params)
	{
		std::cout << "Applying software processing:" << std::endl;

		/* Print enabled algorithms and mode */
		if (params.enableAWB) {
			std::cout << "	AWB Mode: ";
			switch (params.awbMode) {
			case 0: std::cout << "Auto"; break;
			case 1: std::cout << "Daylight"; break;
			case 2: std::cout << "Cloudy"; break;
			case 3: std::cout << "Tungsten"; break;
			case 4: std::cout << "Fluorescent"; break;
			case 5: std::cout << "Shade"; break;
			case 6: std::cout << "Manual"; break;
			default: std::cout << "Unknown (" << params.awbMode << ")"; break;
			}
			std::cout << std::endl;

			std::cout << "	AWB Parameters:" << std::endl;
			std::cout << "		Gains: R=" << (params.awbParams.redGain / 1944.0f) << "x"
				  << " GR=" << (params.awbParams.greenRedGain / 1103.0f) << "x"
				  << " B=" << (params.awbParams.blueGain / 3404.0f) << "x"
				  << " GB=" << (params.awbParams.greenBlueGain / 1619.0f) << "x" << std::endl;

			if (params.awbParams.redOffset != 7928 || params.awbParams.blueOffset != 7936 ||
			    params.awbParams.greenRedOffset != 7920 || params.awbParams.greenBlueOffset != 7920) {
				std::cout << "		Offsets: R=" << params.awbParams.redOffset
					  << " GR=" << params.awbParams.greenRedOffset
					  << " B=" << params.awbParams.blueOffset
					  << " GB=" << params.awbParams.greenBlueOffset << std::endl;
			}
		}

		if (params.enableAGC)
			std::cout << "	AGC Gain: " << params.gainValue << " (" << (params.gainValue / 1024.0f) << "x)" << std::endl;

		if (params.enableBLC)
			std::cout << "	BLC Level: " << params.blackLevel << std::endl;

		if (params.enableCCM) {
			std::cout << "	Color Correction Matrix (CCM):" << std::endl;
			for (int i = 0; i < 3; i++) {
				std::cout << "		[";
				for (int j = 0; j < 3; j++) {
					std::cout << std::fixed << std::setprecision(3)
						  << (params.ccmMatrix[i*3 + j] / 1024.0f) << " ";
				}
				std::cout << "]" << std::endl;
			}
		}

		/* Check if any processing is enabled */
		if (!params.enableAGC && !params.enableBLC && !params.enableAWB && !params.enableCCM) {
			std::cout << "No processing needed (all algorithms disabled)" << std::endl;
			return;
		}

		/* Process RGB565 format */
		/* Each pixel is 16 bits (2 bytes) */
		for (unsigned int i = 0; i < width * height; i++) {
			/* Calculate position in buffer */
			size_t pixelPos = i * 2;

			/* Extract RGB565 value (little endian) */
			uint16_t pixel = static_cast<uint16_t>(data[pixelPos]) |
				(static_cast<uint16_t>(data[pixelPos + 1]) << 8);

			/* Extract RGB components (5 bits R, 6 bits G, 5 bits B) */
			uint8_t r = (pixel >> 11) & 0x1F;
			uint8_t g = (pixel >> 5) & 0x3F;
			uint8_t b = pixel & 0x1F;

			/* Convert to 8-bit per channel */
			r = (r << 3) | (r >> 2);	/* 5 bits to 8 bits */
			g = (g << 2) | (g >> 4);	/* 6 bits to 8 bits */
			b = (b << 3) | (b >> 2);	/* 5 bits to 8 bits */

			/* Apply Black Level Correction if enabled */
			if (params.enableBLC && params.blackLevel > 0) {
				r = std::max(0, r - params.blackLevel);
				g = std::max(0, g - params.blackLevel);
				b = std::max(0, b - params.blackLevel);
			}

			/* Apply AGC if enabled */
			if (params.enableAGC && params.gainValue != 1024) {
				float gain = params.gainValue / 1024.0f;
				r = std::clamp(static_cast<int>(r * gain), 0, 255);
				g = std::clamp(static_cast<int>(g * gain), 0, 255);
				b = std::clamp(static_cast<int>(b * gain), 0, 255);
			}

			/* Apply AWB if enabled */
			if (params.enableAWB) {
				/* Apply offsets first - these affect shadows/brightness */
				/* Scale down the offsets from their native range (-8191 to 8191) to 8-bit pixel range */
				float offsetScale = 255.0f / 8191.0f;

				/* Calculate offset difference from hardware defaults */
					float rOffset = (params.awbParams.redOffset - 7928) * offsetScale;
					float gOffset = (((params.awbParams.greenRedOffset - 7920) + (params.awbParams.greenBlueOffset - 7920)) / 2.0f) * offsetScale;
					float bOffset = (params.awbParams.blueOffset - 7936) * offsetScale;

					/* Apply offsets */
					r = std::clamp(static_cast<int>(r + rOffset), 0, 255);
					g = std::clamp(static_cast<int>(g + gOffset), 0, 255);
					b = std::clamp(static_cast<int>(b + bOffset), 0, 255);

					/* Apply gains using camera's actual hardware values as the neutral point */
					float rGain = params.awbParams.redGain / 1944.0f;
					float grGain = params.awbParams.greenRedGain / 1103.0f;
					float bGain = params.awbParams.blueGain / 3404.0f;
					float gbGain = params.awbParams.greenBlueGain / 1619.0f;

					/* For green, use the average of both green gains */
					float gGain = (grGain + gbGain) / 2.0f;

					r = std::clamp(static_cast<int>(r * rGain), 0, 255);
					g = std::clamp(static_cast<int>(g * gGain), 0, 255);
					b = std::clamp(static_cast<int>(b * bGain), 0, 255);
			}

			/* Apply CCM if enabled */
			if (params.enableCCM) {
				int r_orig = r;
				int g_orig = g;
				int b_orig = b;

				r = std::clamp((params.ccmMatrix[0] * r_orig +
						params.ccmMatrix[1] * g_orig +
						params.ccmMatrix[2] * b_orig) / 1024, 0, 255);

				g = std::clamp((params.ccmMatrix[3] * r_orig +
						params.ccmMatrix[4] * g_orig +
						params.ccmMatrix[5] * b_orig) / 1024, 0, 255);

				b = std::clamp((params.ccmMatrix[6] * r_orig +
						params.ccmMatrix[7] * g_orig +
						params.ccmMatrix[8] * b_orig) / 1024, 0, 255);
			}

			/* Convert back to RGB565 format */
			r = r >> 3;  /* 8 bits to 5 bits */
			g = g >> 2;  /* 8 bits to 6 bits */
			b = b >> 3;  /* 8 bits to 5 bits */

			/* Pack back into RGB565 */
			pixel = (r << 11) | (g << 5) | b;

			/* Write back to buffer */
			data[pixelPos] = pixel & 0xFF;
			data[pixelPos + 1] = (pixel >> 8) & 0xFF;
		}

		std::cout << "Software image processing completed" << std::endl;
	}
};

} /* namespace mchpcam */
#endif /* MCHPCAM_IMAGE_PROCESSOR_H */
