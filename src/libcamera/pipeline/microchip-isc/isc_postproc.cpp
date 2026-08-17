/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * Software post-processing stage for the Microchip ISC pipeline handler
 */

#include "isc_postproc.h"

#include <algorithm>
#include <cstdint>

#include <libcamera/base/log.h>

#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>

#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IscPostProc)

namespace {

/* Only packed formats the filters understand are candidates for processing. */
bool isFilterableFormat(const PixelFormat &format)
{
	return format == formats::RGB565 || format == formats::YUYV ||
	       format == formats::UYVY;
}

/*
 * Bilinearly interpolate a single channel's gain map at a pixel whose grid
 * coordinates have already been resolved to the bracketing indices and
 * fractional offsets. Kept out of the per-pixel loop bodies so the RGB565 and
 * YUV paths share one implementation.
 */
inline float bilinear(const std::vector<float> &m, unsigned int gw,
		      unsigned int i0, unsigned int i1, float fx,
		      unsigned int j0, unsigned int j1, float fy)
{
	float top = m[j0 * gw + i0] + (m[j0 * gw + i1] - m[j0 * gw + i0]) * fx;
	float bot = m[j1 * gw + i0] + (m[j1 * gw + i1] - m[j1 * gw + i0]) * fx;
	return top + (bot - top) * fy;
}

} /* namespace */

IscPostProcessor::IscPostProcessor()
{
	worker_ = std::make_unique<Worker>(this);
	worker_->bufferProcessed.connect(this, &IscPostProcessor::workerDone);
	worker_->moveToThread(&thread_);
}

IscPostProcessor::~IscPostProcessor()
{
	/* Destroy the worker before its thread is gone. */
	worker_.reset();
}

namespace {

/*
 * Upper bound on a lens-shading grid dimension. The maps are a coarse sector
 * grid (the tool emits 16x12); anything approaching this is malformed. The cap
 * also keeps grid_width * grid_height well inside a 32-bit size_t so the
 * element count below cannot overflow on the 32-bit Cortex-A7.
 */
constexpr unsigned int kMaxGridDim = 256;

} /* namespace */

/*
 * Parse a flat-field lens-shading gain map from the tuning file's
 * 'LensShadingCorrection' module block: a grid_width x grid_height sector grid
 * and per-channel gain lists (r, g, b), each grid_width*grid_height entries in
 * row-major order.
 *
 * The gains live either directly under a 'gains' block (single-illuminant
 * calibration, the legacy shape) or under a 'per_ct' list of { ct, gains }
 * entries when the flats spanned several colour temperatures. The single-map
 * shape is parsed into lscGrid_; the multi-CT shape fills lscPerCt_ and
 * selectLscGrid() picks per frame.
 *
 * A malformed block leaves both stores empty, which disables the correction
 * rather than failing the stream.
 */
void IscPostProcessor::loadConfig(const YamlObject &config)
{
	lscGrid_ = LscGrid{};
	lscPerCt_.clear();

	unsigned int gw = config["grid_width"].get<uint32_t>(0);
	unsigned int gh = config["grid_height"].get<uint32_t>(0);

	if (gw < 2 || gh < 2 || gw > kMaxGridDim || gh > kMaxGridDim) {
		LOG(IscPostProc, Warning)
			<< "Lens-shading grid " << gw << "x" << gh
			<< " out of range; LSC disabled";
		return;
	}

	/* gw, gh <= kMaxGridDim, so this cannot overflow a 32-bit size_t. */
	size_t need = static_cast<size_t>(gw) * gh;

	/* Fill one LscGrid from a 'gains' mapping; false if malformed. */
	auto parseGains = [&](const YamlObject &gains, LscGrid &grid) -> bool {
		auto r = gains["r"].getList<double>();
		auto g = gains["g"].getList<double>();
		auto b = gains["b"].getList<double>();
		if (!r || !g || !b ||
		    r->size() != need || g->size() != need ||
		    b->size() != need)
			return false;
		grid.gw = gw;
		grid.gh = gh;
		grid.r.assign(r->begin(), r->end());
		grid.g.assign(g->begin(), g->end());
		grid.b.assign(b->begin(), b->end());
		grid.valid = true;
		return true;
	};

	if (config.contains("per_ct")) {
		for (const YamlObject &entry : config["per_ct"].asList()) {
			CtLscGrid ctGrid;
			ctGrid.ct = entry["ct"].get<uint32_t>(0);
			if (!parseGains(entry["gains"], ctGrid.grid)) {
				LOG(IscPostProc, Warning)
					<< "Malformed per-CT lens-shading map (expected "
					<< gw << "x" << gh << " per channel at ct="
					<< ctGrid.ct << "); LSC disabled";
				lscPerCt_.clear();
				return;
			}
			lscPerCt_.push_back(std::move(ctGrid));
		}
		std::sort(lscPerCt_.begin(), lscPerCt_.end(),
			  [](const CtLscGrid &a, const CtLscGrid &b) {
				  return a.ct < b.ct;
			  });
		LOG(IscPostProc, Info)
			<< "Loaded " << lscPerCt_.size() << " per-CT " << gw << "x"
			<< gh << " lens-shading map(s)";
		return;
	}

	if (!parseGains(config["gains"], lscGrid_)) {
		LOG(IscPostProc, Warning)
			<< "Malformed lens-shading map (expected " << gw << "x"
			<< gh << " per channel); LSC disabled";
		lscGrid_ = LscGrid{};
		return;
	}

	LOG(IscPostProc, Info)
		<< "Loaded " << gw << "x" << gh << " lens-shading map";
}

/*
 * Pick the per-CT lens-shading map nearest the scene colour temperature. With a
 * single-map calibration (lscPerCt_ empty) the one loaded map is returned
 * unchanged. Nearest-CT selection is used rather than blending two bracketing
 * maps: the CT metadata is coarse and lags the scene by the IPA's convergence
 * delay, so a blend would not be measurably more accurate than the nearest map.
 *
 * A CT of 0 means the IPA has not yet reported one (early frames, before AWB
 * converges). Matching on 0 would always pick the lowest-CT map, biasing those
 * frames warm; select against a neutral daylight reference instead until a real
 * CT arrives.
 */
const IscPostProcessor::LscGrid &
IscPostProcessor::selectLscGrid(unsigned int colourTemperature) const
{
	if (lscPerCt_.empty())
		return lscGrid_;

	constexpr unsigned int kNeutralCt = 5000;
	unsigned int ct = colourTemperature ? colourTemperature : kNeutralCt;

	const CtLscGrid *best = &lscPerCt_.front();
	unsigned int bestDelta = ct > best->ct ? ct - best->ct : best->ct - ct;
	for (const CtLscGrid &c : lscPerCt_) {
		unsigned int delta = ct > c.ct ? ct - c.ct : c.ct - ct;
		if (delta < bestDelta) {
			bestDelta = delta;
			best = &c;
		}
	}
	return best->grid;
}

int IscPostProcessor::configure(const PixelFormat &format, const Size &size,
				unsigned int stride)
{
	format_ = format;
	size_ = size;
	stride_ = stride;

	bool filterable = isFilterableFormat(format);

	/* Lens-shading runs for every role when a gain map is loaded. */
	lscActive_ = filterable && (lscGrid_.valid || !lscPerCt_.empty());

	enabled_ = lscActive_;

	LOG(IscPostProc, Debug)
		<< "configure " << format.toString() << " " << size.toString()
		<< " stride=" << stride << " lsc=" << lscActive_
		<< " enabled=" << enabled_;

	return 0;
}

int IscPostProcessor::start()
{
	if (!enabled_ || running_)
		return 0;

	thread_.start();
	running_ = true;
	return 0;
}

void IscPostProcessor::stop()
{
	if (!running_)
		return;

	thread_.exit();
	thread_.wait();

	/*
	 * The thread is joined, so every filter that started has finished and
	 * posted its workerDone() to this object; dispatch those now so their
	 * buffers complete through frameProcessed.
	 */
	Thread::current()->dispatchMessages(Message::Type::InvokeMessage, this);

	/*
	 * Any process() calls still queued on the worker never ran (the thread
	 * exited first). Drop them so they cannot fire against released buffers
	 * on the next start(), then complete the buffers they carried directly:
	 * the caller relies on frameProcessed for every queued buffer, and the
	 * pixels are valid even though the filter was skipped.
	 */
	thread_.removeMessages(worker_.get());
	for (FrameBuffer *buffer : inFlight_)
		frameProcessed.emit(buffer);
	inFlight_.clear();

	running_ = false;
}

void IscPostProcessor::queue(FrameBuffer *buffer, unsigned int colourTemperature)
{
	inFlight_.push_back(buffer);
	worker_->invokeMethod(&Worker::process, ConnectionTypeQueued, buffer,
			      colourTemperature);
}

void IscPostProcessor::workerDone(FrameBuffer *buffer)
{
	/* Runs on the pipeline thread; hand the buffer back to the handler. */
	auto it = std::find(inFlight_.begin(), inFlight_.end(), buffer);
	if (it != inFlight_.end())
		inFlight_.erase(it);
	frameProcessed.emit(buffer);
}

void IscPostProcessor::Worker::process(FrameBuffer *buffer,
				       unsigned int colourTemperature)
{
	/*
	 * The filters run in place on the ISC video device's own exported
	 * capture buffer. This is safe: the buffer is completed to the
	 * application only after this returns (via workerDone → frameProcessed),
	 * so no consumer reads it while it is being rewritten, and it is not
	 * re-queued to the ISC until the request completes. The DmaSyncer holds
	 * the buffer in ReadWrite for the whole map so the CPU sees the ISC's
	 * writes and the ISC (or a downstream DMA consumer) sees ours across the
	 * buffer ring on the non-coherent Cortex-A7.
	 */
	std::vector<DmaSyncer> syncers;
	for (const FrameBuffer::Plane &plane : buffer->planes())
		syncers.emplace_back(plane.fd, DmaSyncer::SyncType::ReadWrite);

	MappedFrameBuffer map(buffer, MappedFrameBuffer::MapFlag::ReadWrite);
	if (!map.isValid()) {
		if (!mapErrorLogged_) {
			LOG(IscPostProc, Error)
				<< "Failed to map buffer for post-processing;"
				<< " completing frames unfiltered";
			mapErrorLogged_ = true;
		}
		syncers.clear();
		bufferProcessed.emit(buffer);
		return;
	}

	/*
	 * Guard against a plane too small for the configured geometry before
	 * the filters index into it: a mismatched stride or height would
	 * otherwise walk off the mapping.
	 */
	Span<uint8_t> plane = map.planes()[0];
	size_t needed = static_cast<size_t>(parent_->stride_) *
			parent_->size_.height;
	if (plane.size() < needed) {
		LOG(IscPostProc, Error)
			<< "Mapped plane " << plane.size() << " < required "
			<< needed << " bytes; completing frame unfiltered";
		syncers.clear();
		bufferProcessed.emit(buffer);
		return;
	}

	parent_->applyFilters(plane.data(), colourTemperature);

	syncers.clear();

	bufferProcessed.emit(buffer);
}

void IscPostProcessor::applyFilters(uint8_t *data,
				    unsigned int colourTemperature) const
{
	if (lscActive_) {
		const LscGrid &grid = selectLscGrid(colourTemperature);
		if (format_ == formats::RGB565)
			applyLscRgb565(data, grid);
		else if (format_ == formats::YUYV)
			applyLscYuv422(data, grid, false);
		else if (format_ == formats::UYVY)
			applyLscYuv422(data, grid, true);
	}
}

/*
 * Lens-shading for packed little-endian RGB565. Each 5/6/5 pixel is expanded
 * to 8-bit, scaled by its per-channel interpolated gain, clamped, and repacked.
 * Pixel centres map to grid coordinates so the grid edges sit at the image
 * corners (matches the still-path map convention).
 */
void IscPostProcessor::applyLscRgb565(uint8_t *data, const LscGrid &grid) const
{
	const unsigned int w = size_.width;
	const unsigned int h = size_.height;
	const unsigned int gw = grid.gw;
	const unsigned int gh = grid.gh;

	for (unsigned int py = 0; py < h; ++py) {
		float gy = std::clamp((py + 0.5f) * gh / h - 0.5f,
				      0.0f, static_cast<float>(gh - 1));
		unsigned int j0 = static_cast<unsigned int>(gy);
		unsigned int j1 = std::min(j0 + 1, gh - 1);
		float fy = gy - j0;

		uint8_t *row = data + static_cast<size_t>(py) * stride_;
		for (unsigned int px = 0; px < w; ++px) {
			float gx = std::clamp((px + 0.5f) * gw / w - 0.5f,
					      0.0f, static_cast<float>(gw - 1));
			unsigned int i0 = static_cast<unsigned int>(gx);
			unsigned int i1 = std::min(i0 + 1, gw - 1);
			float fx = gx - i0;

			uint8_t *p = row + px * 2;
			uint16_t v = static_cast<uint16_t>(p[0] | (p[1] << 8));
			/* Expand 5/6/5 to 8-bit by replicating the high bits. */
			unsigned int r5 = (v >> 11) & 0x1f;
			unsigned int g6 = (v >> 5) & 0x3f;
			unsigned int b5 = v & 0x1f;
			float r = ((r5 << 3) | (r5 >> 2)) *
				  bilinear(grid.r, gw, i0, i1, fx, j0, j1, fy);
			float g = ((g6 << 2) | (g6 >> 4)) *
				  bilinear(grid.g, gw, i0, i1, fx, j0, j1, fy);
			float b = ((b5 << 3) | (b5 >> 2)) *
				  bilinear(grid.b, gw, i0, i1, fx, j0, j1, fy);

			unsigned int ri = static_cast<unsigned int>(
				std::clamp(r, 0.0f, 255.0f));
			unsigned int gi = static_cast<unsigned int>(
				std::clamp(g, 0.0f, 255.0f));
			unsigned int bi = static_cast<unsigned int>(
				std::clamp(b, 0.0f, 255.0f));
			uint16_t out = static_cast<uint16_t>(
				((ri >> 3) << 11) | ((gi >> 2) << 5) | (bi >> 3));
			p[0] = out & 0xff;
			p[1] = out >> 8;
		}
	}
}

/*
 * Lens-shading for packed 4:2:2 (YUYV or UYVY). Luma is corrected per Y with a
 * luma-weighted gain; the chroma pair shared by two pixels is corrected around
 * neutral (128) with the red gain on Cr and the blue gain on Cb, sampled at the
 * even pixel. For a uniform (luminance-only) map this reproduces exact RGB
 * scaling; when the channel gains differ it is a first-order colour-shading
 * approximation — acceptable because the still path (which needs the most
 * accurate colour) captures RGB, while the 4:2:2 video path is corrected for
 * vignetting brightness, its dominant artefact.
 */
void IscPostProcessor::applyLscYuv422(uint8_t *data, const LscGrid &grid,
				      bool uyvy) const
{
	const unsigned int w = size_.width;
	const unsigned int h = size_.height;
	const unsigned int gw = grid.gw;
	const unsigned int gh = grid.gh;
	const int yOff = uyvy ? 1 : 0;
	const int cOff = uyvy ? 0 : 1;

	/* Resolve the x-axis grid bracket for a column, shared by all channels. */
	unsigned int i0, i1;
	float fx;
	auto xbracket = [&](unsigned int px) {
		float gx = std::clamp((px + 0.5f) * gw / w - 0.5f,
				      0.0f, static_cast<float>(gw - 1));
		i0 = static_cast<unsigned int>(gx);
		i1 = std::min(i0 + 1, gw - 1);
		fx = gx - i0;
	};

	for (unsigned int py = 0; py < h; ++py) {
		float gy = std::clamp((py + 0.5f) * gh / h - 0.5f,
				      0.0f, static_cast<float>(gh - 1));
		unsigned int j0 = static_cast<unsigned int>(gy);
		unsigned int j1 = std::min(j0 + 1, gh - 1);
		float fy = gy - j0;

		uint8_t *row = data + static_cast<size_t>(py) * stride_;
		for (unsigned int px = 0; px < w; px += 2) {
			uint8_t *blk = row + px * 2;

			/*
			 * Chroma pair sampled at the even pixel: Cb (blue
			 * difference) takes the blue gain, Cr (red difference)
			 * the red gain, applied around neutral 128. The pair
			 * spans this pixel and its odd neighbour, so skip it
			 * when the neighbour is past the (odd) image width.
			 */
			if (px + 1 < w) {
				xbracket(px);
				float gr = bilinear(grid.r, gw, i0, i1, fx, j0, j1, fy);
				float gb = bilinear(grid.b, gw, i0, i1, fx, j0, j1, fy);
				float u = blk[cOff] - 128.0f;
				float vv = blk[cOff + 2] - 128.0f;
				blk[cOff] = static_cast<uint8_t>(
					std::clamp(128.0f + u * gb, 0.0f, 255.0f));
				blk[cOff + 2] = static_cast<uint8_t>(
					std::clamp(128.0f + vv * gr, 0.0f, 255.0f));
			}

			/* Luma corrected per pixel with a luma-weighted gain. */
			for (unsigned int k = 0; k < 2 && px + k < w; ++k) {
				xbracket(px + k);
				float rr = bilinear(grid.r, gw, i0, i1, fx, j0, j1, fy);
				float rg = bilinear(grid.g, gw, i0, i1, fx, j0, j1, fy);
				float rb = bilinear(grid.b, gw, i0, i1, fx, j0, j1, fy);
				float ly = 0.299f * rr + 0.587f * rg + 0.114f * rb;
				uint8_t *y = blk + k * 2 + yOff;
				*y = static_cast<uint8_t>(
					std::clamp(*y * ly, 0.0f, 255.0f));
			}
		}
	}
}

} /* namespace libcamera */
