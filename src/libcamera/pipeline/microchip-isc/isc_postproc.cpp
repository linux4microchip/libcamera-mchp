/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Microchip Technology Inc.
 *
 * Software post-processing stage for the Microchip ISC pipeline handler
 */

#include "isc_postproc.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <new>

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
	return format == formats::RGB565 || format == formats::XRGB8888 ||
	       format == formats::YUYV || format == formats::UYVY;
}

/* Bytes per pixel of the packed output formats the stage handles. */
unsigned int bytesPerPixel(const PixelFormat &format)
{
	return format == formats::XRGB8888 ? 4u : 2u;
}

/* BT.601 luma of an 8-bit RGB triplet. */
inline unsigned int luma(unsigned int r, unsigned int g, unsigned int b)
{
	return (77u * r + 150u * g + 29u * b) >> 8;
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
		/* A non-finite gain would poison every interpolated pixel. */
		auto finite = [](const std::vector<double> &v) {
			return std::all_of(v.begin(), v.end(), [](double x) {
				return std::isfinite(x);
			});
		};
		if (!finite(*r) || !finite(*g) || !finite(*b))
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
		LOG(IscPostProc, Debug)
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

	LOG(IscPostProc, Debug)
		<< "Loaded " << gw << "x" << gh << " lens-shading map";
}

void IscPostProcessor::loadAutoLevelsConfig(const YamlObject &config)
{
	AutoLevels al;
	al.enable = config["enable"].get<bool>(false);
	al.loPct = config["lo_percentile"].get<double>(al.loPct);
	al.hiPct = config["hi_percentile"].get<double>(al.hiPct);
	al.maxGain = config["max_gain"].get<double>(al.maxGain);
	al.blackTarget = config["black_target"].get<uint32_t>(al.blackTarget);
	al.whiteTarget = config["white_target"].get<uint32_t>(al.whiteTarget);
	al.sample = std::max(1u, config["sample"].get<uint32_t>(al.sample));

	/* Reject a nonsensical block rather than stretching to garbage. */
	if (al.hiPct <= al.loPct || al.whiteTarget <= al.blackTarget ||
	    al.maxGain < 1.0f) {
		LOG(IscPostProc, Warning) << "Invalid AutoLevels block; disabled";
		return;
	}

	autoLevels_ = al;
	LOG(IscPostProc, Debug)
		<< "AutoLevels " << (al.enable ? "enabled" : "disabled")
		<< " black=" << al.blackTarget << " white=" << al.whiteTarget
		<< " maxGain=" << al.maxGain;
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

	/* A stride below one packed line would over-read. */
	unsigned int minStride = size.width * bytesPerPixel(format);
	if (filterable && stride < minStride) {
		LOG(IscPostProc, Warning)
			<< "stride " << stride << " < " << minStride
			<< " for " << size.toString()
			<< "; post-processing disabled";
		filterable = false;
	}

	/* Active when a gain map is loaded; the pass is gated per request. */
	lscActive_ = filterable && (lscGrid_.valid || !lscPerCt_.empty());
	autoLevelsActive_ = filterable && autoLevels_.enable;

	enabled_ = lscActive_ || autoLevelsActive_;

	LOG(IscPostProc, Debug)
		<< "configure " << format.toString() << " " << size.toString()
		<< " stride=" << stride << " lsc=" << lscActive_
		<< " autolevels=" << autoLevelsActive_ << " enabled=" << enabled_;

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
		frameProcessed.emit(buffer, false);
	inFlight_.clear();

	running_ = false;
}

void IscPostProcessor::queue(FrameBuffer *buffer, unsigned int colourTemperature)
{
	inFlight_.push_back(buffer);
	worker_->invokeMethod(&Worker::process, ConnectionTypeQueued, buffer,
			      colourTemperature);
}

void IscPostProcessor::workerDone(FrameBuffer *buffer, bool filtered)
{
	/* Runs on the pipeline thread; hand the buffer back to the handler. */
	auto it = std::find(inFlight_.begin(), inFlight_.end(), buffer);
	if (it != inFlight_.end())
		inFlight_.erase(it);
	frameProcessed.emit(buffer, filtered);
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
		bufferProcessed.emit(buffer, false);
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
		bufferProcessed.emit(buffer, false);
		return;
	}

	bool filtered = true;
	try {
		parent_->applyFilters(plane.data(), colourTemperature);
	} catch (const std::bad_alloc &) {
		/* Complete the frame unfiltered rather than abort the stream. */
		LOG(IscPostProc, Error) << "Out of memory in post-processing";
		filtered = false;
	}

	syncers.clear();

	bufferProcessed.emit(buffer, filtered);
}

void IscPostProcessor::applyFilters(uint8_t *data,
				    unsigned int colourTemperature) const
{
	if (lscActive_) {
		const LscGrid &grid = selectLscGrid(colourTemperature);
		if (format_ == formats::RGB565)
			applyLscRgb565(data, grid);
		else if (format_ == formats::XRGB8888)
			applyLscXrgb8888(data, grid);
		else if (format_ == formats::YUYV)
			applyLscYuv422(data, grid, false);
		else if (format_ == formats::UYVY)
			applyLscYuv422(data, grid, true);
	}

	/* Contrast stretch runs last, on the (optionally shading-corrected) pixels. */
	if (autoLevelsActive_)
		applyAutoLevels(data);
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

/* Lens-shading for packed XRGB8888 (memory order B, G, R, X). */
void IscPostProcessor::applyLscXrgb8888(uint8_t *data, const LscGrid &grid) const
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

			uint8_t *p = row + px * 4;
			float b = p[0] * bilinear(grid.b, gw, i0, i1, fx, j0, j1, fy);
			float g = p[1] * bilinear(grid.g, gw, i0, i1, fx, j0, j1, fy);
			float r = p[2] * bilinear(grid.r, gw, i0, i1, fx, j0, j1, fy);
			p[0] = static_cast<uint8_t>(std::clamp(b, 0.0f, 255.0f));
			p[1] = static_cast<uint8_t>(std::clamp(g, 0.0f, 255.0f));
			p[2] = static_cast<uint8_t>(std::clamp(r, 0.0f, 255.0f));
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

bool IscPostProcessor::buildAutoLevelsMap(const uint8_t *data,
					  std::array<uint8_t, 256> &map) const
{
	const unsigned int w = size_.width;
	const unsigned int h = size_.height;
	const unsigned int s = autoLevels_.sample;

	/* Subsampled luma histogram: percentiles are stable well below full res. */
	std::array<uint32_t, 256> hist = {};
	uint64_t total = 0;
	for (unsigned int py = 0; py < h; py += s) {
		const uint8_t *row = data + static_cast<size_t>(py) * stride_;
		for (unsigned int px = 0; px < w; px += s) {
			unsigned int y;
			if (format_ == formats::XRGB8888) {
				const uint8_t *p = row + px * 4;
				y = luma(p[2], p[1], p[0]);
			} else if (format_ == formats::RGB565) {
				const uint8_t *p = row + px * 2;
				uint16_t v = p[0] | (p[1] << 8);
				unsigned int r5 = (v >> 11) & 0x1f;
				unsigned int g6 = (v >> 5) & 0x3f;
				unsigned int b5 = v & 0x1f;
				y = luma((r5 << 3) | (r5 >> 2),
					 (g6 << 2) | (g6 >> 4),
					 (b5 << 3) | (b5 >> 2));
			} else {
				/* 4:2:2 — Y is the first byte of YUYV, second of UYVY. */
				const uint8_t *p = row + px * 2;
				y = (format_ == formats::UYVY) ? p[1] : p[0];
			}
			hist[y]++;
			total++;
		}
	}

	if (total == 0)
		return false;

	auto percentile = [&](float frac) {
		uint64_t target = static_cast<uint64_t>(frac * total);
		uint64_t cum = 0;
		for (unsigned int i = 0; i < 256; ++i) {
			cum += hist[i];
			if (cum >= target)
				return i;
		}
		return 255u;
	};

	unsigned int lo = percentile(autoLevels_.loPct);
	unsigned int hi = percentile(autoLevels_.hiPct);
	if (hi <= lo)
		return false;

	float gain = static_cast<float>(autoLevels_.whiteTarget - autoLevels_.blackTarget) /
		     static_cast<float>(hi - lo);
	gain = std::clamp(gain, 1.0f, autoLevels_.maxGain);

	for (unsigned int i = 0; i < 256; ++i) {
		float v = autoLevels_.blackTarget +
			  (static_cast<float>(i) - lo) * gain;
		map[i] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
	}
	return true;
}

/*
 * Expand the output luma range with a single affine map applied to each colour
 * channel. Built from the frame's own luma percentiles, so a compressed
 * distribution is stretched toward the black/white targets without touching
 * the IPA-owned white balance.
 */
void IscPostProcessor::applyAutoLevels(uint8_t *data) const
{
	std::array<uint8_t, 256> map;
	if (!buildAutoLevelsMap(data, map))
		return;

	const unsigned int w = size_.width;
	const unsigned int h = size_.height;

	for (unsigned int py = 0; py < h; ++py) {
		uint8_t *row = data + static_cast<size_t>(py) * stride_;
		if (format_ == formats::XRGB8888) {
			for (unsigned int px = 0; px < w; ++px) {
				uint8_t *p = row + px * 4;
				p[0] = map[p[0]];
				p[1] = map[p[1]];
				p[2] = map[p[2]];
			}
		} else if (format_ == formats::RGB565) {
			for (unsigned int px = 0; px < w; ++px) {
				uint8_t *p = row + px * 2;
				uint16_t v = p[0] | (p[1] << 8);
				unsigned int r5 = (v >> 11) & 0x1f;
				unsigned int g6 = (v >> 5) & 0x3f;
				unsigned int b5 = v & 0x1f;
				unsigned int r = map[(r5 << 3) | (r5 >> 2)];
				unsigned int g = map[(g6 << 2) | (g6 >> 4)];
				unsigned int b = map[(b5 << 3) | (b5 >> 2)];
				uint16_t out = static_cast<uint16_t>(
					((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
				p[0] = out & 0xff;
				p[1] = out >> 8;
			}
		} else {
			/* 4:2:2 — stretch luma only, leave chroma. */
			int yOff = (format_ == formats::UYVY) ? 1 : 0;
			for (unsigned int px = 0; px < w; ++px) {
				uint8_t *y = row + px * 2 + yOff;
				*y = map[*y];
			}
		}
	}
}

} /* namespace libcamera */
