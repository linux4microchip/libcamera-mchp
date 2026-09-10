/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * Software post-processing stage for the Microchip ISC pipeline handler
 */

#pragma once

#include <array>
#include <memory>
#include <vector>

#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

namespace libcamera {

class FrameBuffer;
class YamlObject;

/**
 * \class IscPostProcessor
 * \brief Software lens-shading stage for the ISC output buffer
 *
 * The ISC has no hardware lens-shading block, so the correction is applied in
 * software here, on the finished output buffer, before it is completed to the
 * application. Hosting it in the pipeline handler (rather than one
 * application) lets any client — cam, qcam, libcamerasrc, mchpcam — request it
 * through the IscStillProcess control.
 *
 * The IPA cannot host this: it is a control-plane process that only sees
 * statistics and emits parameters, never pixel buffers. Per-pixel work must
 * live on the buffer path, which is the pipeline handler.
 *
 * The soft-ISP framework (SoftwareIsp / DebayerCpu) is not applicable either:
 * it is a RAW-Bayer to RGB888 demosaicer. It only ingests Bayer input, only
 * emits packed 24/32-bit RGB, and carries corrections as value-indexed LUTs
 * (DebayerParams) with no place for a spatial gain grid; it also allocates its
 * own input/output buffer pair. The ISC demosaics in hardware and hands us a
 * finished RGB565/YUYV/UYVY buffer, so there is no Bayer input to consume, no
 * demosaic to do, and its target formats are not in that framework's output
 * set. Converter is likewise hardware-M2M only. So the correct home for
 * per-pixel correction of an already-demosaiced ISC buffer is this
 * pipeline-handler-owned stage, filtering the buffer in place.
 *
 * Lens-shading is one multiply per pixel, but the stage is gated per request
 * via IscStillProcess: it runs only on the frames the application marks and
 * completes the rest straight from the ISC output.
 */
class IscPostProcessor : public Object
{
public:
	IscPostProcessor();
	~IscPostProcessor();

	/**
	 * \brief Load the lens-shading gain map from a tuning-file module block
	 * \param[in] config The 'LensShadingCorrection' module sub-block
	 *
	 * Parses the sector grid and per-channel gains. A malformed block
	 * disables the correction rather than failing; an absent block
	 * (loadConfig() never called) leaves the stage inactive, matching the
	 * pre-calibration behaviour.
	 */
	void loadConfig(const YamlObject &config);
	/**
	 * \brief Load the auto-levels stretch parameters from a module block
	 * \param[in] config The 'AutoLevels' module sub-block
	 *
	 * An absent or disabled block leaves the stretch inactive.
	 */
	void loadAutoLevelsConfig(const YamlObject &config);
	/**
	 * \brief Configure the stage for a stream format
	 * \param[in] format Output pixel format of the ISC capture buffer
	 * \param[in] size Output frame size
	 * \param[in] stride Line stride of the output buffer, in bytes
	 * \return 0 on success
	 */
	int configure(const PixelFormat &format, const Size &size,
		       unsigned int stride);

	/** \brief Whether any filter is active for the configured stream */
	bool isEnabled() const { return enabled_; }

	/** \brief Start the worker thread if the stage is enabled */
	int start();
	/** \brief Drain and stop the worker thread, completing queued buffers */
	void stop();

	/**
	 * \brief Queue a finished buffer for the in-place still pass
	 * \param[in] buffer The ISC output buffer to filter
	 * \param[in] colourTemperature Scene CT for per-CT map selection (0 if unknown)
	 *
	 * Runs the pass on the worker thread and returns the buffer via
	 * frameProcessed. Queued only for frames marked IscStillProcess.
	 */
	void queue(FrameBuffer *buffer, unsigned int colourTemperature);

	/**
	 * \brief Emitted on the pipeline thread when a queued buffer is done
	 *
	 * The bool is true if the pass ran, false if the buffer was completed
	 * unfiltered (map/size failure, out of memory, or drained at stop()).
	 */
	Signal<FrameBuffer *, bool> frameProcessed;

private:
	/*
	 * The filter runs on a worker thread so it does not stall the pipeline
	 * thread (where all camera events are delivered). The completion signal
	 * must reach the handler on the pipeline thread, so a two-object split
	 * is used: this outer object lives on the pipeline thread and owns the
	 * inner Worker moved to the worker thread. The worker's completion
	 * signal is connected to a slot on this object, so delivery hops back
	 * to the pipeline thread automatically.
	 */
	class Worker : public Object
	{
	public:
		Worker(IscPostProcessor *parent) : parent_(parent) {}
		void process(FrameBuffer *buffer, unsigned int colourTemperature);
		Signal<FrameBuffer *, bool> bufferProcessed;

	private:
		IscPostProcessor *parent_;
		/* Log a persistent map failure once, not once per frame. */
		bool mapErrorLogged_ = false;
	};

	void workerDone(FrameBuffer *buffer, bool filtered);

	/* Applies the active filters in place on the mapped plane data. */
	void applyFilters(uint8_t *data, unsigned int colourTemperature) const;

	/*
	 * Lens-shading correction. Per-channel flat-field gain maps on a coarse
	 * sector grid (from the tuning file's 'LensShadingCorrection' module
	 * block, produced by microchip_isc_lt.py), bilinearly interpolated to
	 * full resolution and applied as one multiply per pixel.
	 */
	struct LscGrid {
		unsigned int gw = 0;
		unsigned int gh = 0;
		std::vector<float> r;
		std::vector<float> g;
		std::vector<float> b;
		bool valid = false;
	};

	/*
	 * Per-CT lens-shading maps, from the 'per_ct' shape microchip_isc_lt.py
	 * emits when flat-field captures span several illuminant colour
	 * temperatures. Sorted by CT; selectLscGrid() picks the map nearest the
	 * scene CT. Empty when the tuning file carries a single flat 'gains'
	 * block, in which case lscGrid_ is used directly and behaviour is
	 * identical to the legacy single-map path.
	 */
	struct CtLscGrid {
		unsigned int ct = 0;
		LscGrid grid;
	};

	const LscGrid &selectLscGrid(unsigned int colourTemperature) const;

	void applyLscRgb565(uint8_t *data, const LscGrid &grid) const;
	void applyLscXrgb8888(uint8_t *data, const LscGrid &grid) const;
	void applyLscYuv422(uint8_t *data, const LscGrid &grid, bool uyvy) const;

	/*
	 * Output-domain contrast stretch. The IPA only sees the pre-WB histogram;
	 * this is the only stage that sees the final output, so it expands a
	 * compressed distribution to fill the range. Anchored on luma percentiles
	 * and applied to R, G, B alike — contrast, not colour (AWB owns colour).
	 */
	struct AutoLevels {
		float loPct = 0.01f;
		float hiPct = 0.99f;
		float maxGain = 4.0f;
		unsigned int blackTarget = 62;
		unsigned int whiteTarget = 213;
		unsigned int sample = 4;
		bool enable = false;
	};

	/* Build the 256-entry stretch map from a luma histogram; false if degenerate. */
	bool buildAutoLevelsMap(const uint8_t *data,
				std::array<uint8_t, 256> &map) const;
	void applyAutoLevels(uint8_t *data) const;

	std::unique_ptr<Worker> worker_;
	Thread thread_;

	/*
	 * Buffers queued to the worker but not yet completed. Touched only on
	 * the pipeline thread (queue()/workerDone()/stop()), so it needs no
	 * lock. stop() uses it to complete any buffer whose filter never ran
	 * before the thread was joined.
	 */
	std::vector<FrameBuffer *> inFlight_;

	PixelFormat format_;
	Size size_;
	unsigned int stride_ = 0;

	LscGrid lscGrid_;
	std::vector<CtLscGrid> lscPerCt_;
	AutoLevels autoLevels_;

	bool lscActive_ = false;
	bool autoLevelsActive_ = false;
	bool enabled_ = false;
	bool running_ = false;
};

} /* namespace libcamera */
