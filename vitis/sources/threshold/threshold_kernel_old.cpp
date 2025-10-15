#include "threshold/threshold_kernel.hpp"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

constexpr int tilesz = 4;

// Read data from line_buffer to buffer
void readFromLineBuffer(uint8_t input, uint8_t linebuffer[3][imWidth], uint8_t output[4], const int32_t x)
{
#pragma HLS INLINE

	if (x < imWidth)
	{
		for (int32_t i = 0; i < 3; i++)
			output[i] = linebuffer[i][x];
		output[3] = input;
	}
}

// Write data from buffer to line_buffer
void writeToLineBuffer(uint8_t input[4], uint8_t linebuffer[3][imWidth], const int32_t x)
{
#pragma HLS INLINE

	if (x < imWidth)
	{
		for (int32_t i = 0; i < 3; i++)
			linebuffer[i][x] = input[i + 1];
	}
}

// Move sliding window with replicated/constant border
void slidingWindow(uint8_t input[4], uint8_t window[4][4], const int32_t x, const int32_t y)
{
#pragma HLS INLINE
	// Sliding window (no border handling)
	for (int32_t i = 0; i < 4; i++)
	{
		for (int32_t j = 0; j < 3; j++)
			window[i][j] = window[i][j + 1];
		window[i][3] = input[i];
	}
}

void ReadFromMem(
	unsigned short width,
	unsigned short height,
	unsigned short stride,
	const unsigned char *src,
	hls::stream<uint8_t> &pixel_stream)
{
#pragma HLS INLINE

	stride = (stride / 64) * 64; // Makes compiler see that stride is a multiple of 64, enables auto-widening
	unsigned offset = 0;
	unsigned x = 0;
	//    read_image: for (int n = 0; n < height*stride; n++) {
read_image:
	for (int n = 0; n < imWidth * imHeight; n++)
	{
		uint8_t pix = src[n];
		pixel_stream.write(pix);
		//        if (x<width) pixel_stream.write( pix );
		//        if (x==(stride-1)) x=0; else x++;
	}
}

void WriteToMem(
	unsigned short width,
	unsigned short height,
	unsigned short stride,
	hls::stream<uint8_t> &pixel_stream,
	unsigned char *dst)
{
#pragma HLS INLINE
	//    assert(stride <= MAX_IMAGE_WIDTH);
	//    assert(height <= MAX_IMAGE_HEIGHT);
	//    assert(stride%64 == 0);

	//    stride = (stride/64)*64; // Makes compiler see that stride is a multiple of 64, enables auto-widening
	unsigned offset = 0;
	unsigned x = 0;
	int tw = imWidth / 4;
	int th = imHeight / 4;
write_image:
	for (int n = 0; n < tw * th; n++)
	{
		uint8_t pix = pixel_stream.read();
		//        U8 pix = (x<width) ? pixel_stream.read() : 0;
		dst[n] = pix;
		//        if (x==(stride-1)) x=0; else x++;
	}
}

struct window
{
	uint8_t pix[16];
};

// basically read pixel and put it into window buffer. a bit hacked together based on the github code
void Window2D(
	unsigned short width,
	unsigned short height,
	hls::stream<uint8_t> &pixel_stream,
	hls::stream<window> &window_stream)
{
#pragma HLS INLINE
	uint8_t LineBuffer[3][imWidth];
#pragma HLS ARRAY_PARTITION variable = LineBuffer dim = 1 complete
#pragma HLS DEPENDENCE variable = LineBuffer inter false
#pragma HLS DEPENDENCE variable = LineBuffer intra false

	window Window;

	unsigned col_ptr = 0;
	unsigned num_pixels = width * height;
	int tw = imWidth / 4;
	int th = imHeight / 4;
	int maxColIndex = tw * 4;
	int maxRowIndex = th * 4;
	unsigned num_iterations = tw * th;

	// Iterate until all pixels have been processed
	//    update_window: for (int n=0; n<num_iterations; n++)
	//    {
rowL:
	for (int ty = 0; ty < imHeight; ty++)
	{
	colL:
		for (int tx = 0; tx < imWidth; tx++)
		{

			// #pragma HLS LOOP_TRIPCOUNT max=max_iterations
#pragma HLS PIPELINE II = 1

			// Read a new pixel from the input stream
			//        U8 new_pixel = (n<num_pixels) ? pixel_stream.read() : 0;
			uint8_t new_pixel = pixel_stream.read();
			if ((tx >= maxColIndex) || (ty >= maxRowIndex))
			{
				continue;
			}

			// Shift the window and add a column of new pixels from the line buffer
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Window.pix[i * 4 + j] = Window.pix[i * 4 + j + 1];
				}
				Window.pix[i * 4 + 3] = (i < 3) ? LineBuffer[i][col_ptr] : new_pixel;
			}

			// Shift pixels in the column of pixels in the line buffer, add the newest pixel
			for (int i = 0; i < 2; i++)
			{
				LineBuffer[i][col_ptr] = LineBuffer[i + 1][col_ptr];
			}
			LineBuffer[2][col_ptr] = new_pixel;

			// Update the line buffer column pointer
			if (col_ptr == (width - 1))
			{
				col_ptr = 0;
			}
			else
			{
				col_ptr++;
			}

			// we only need the corners of every window
			if (((ty % 4) == 3) && ((tx % 4) == 3))
			{
				window_stream.write(Window);
			}
		}
	}
}

void Pool2D(
	hls::stream<window> &window_stream,
	hls::stream<uint8_t> &stream_max,
	hls::stream<uint8_t> &stream_min)
{

	int tw = imWidth / 4;
	int th = imHeight / 4;

	for (int i = 0; i < tw * th; ++i)
	{
		int min = 255;
		int max = 0;

		window w = window_stream.read();

		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				uint8_t v = w.pix[x * 4 + y];
				if (v < min)
					min = v;
				if (v > max)
					max = v;
			}
		}
		stream_max.write(max);
		stream_min.write(min);
	}
}

void step1Stream(uint8_t in[imHeight * imWidth], uint8_t *im_max, uint8_t *im_min)
{
#pragma HLS DATAFLOW
	// use streams and dataflow pipelining as in
	// https://github.com/Xilinx/Vitis-Tutorials/blob/2023.1/Hardware_Acceleration/Design_Tutorials/01-convolution-tutorial/src/filter2d_hw.cpp

	// Stream of pixels from kernel input to filter, and from filter to output
	hls::stream<uint8_t, 2> pixel_stream;
	hls::stream<window, 3> window_stream; // Set FIFO depth to 0 to minimize resources
	hls::stream<uint8_t, 2> min_stream;
	hls::stream<uint8_t, 2> max_stream;

	// Read image data from global memory over AXI4 MM, and stream pixels out
	ReadFromMem(imWidth, imHeight, imWidth, in, pixel_stream);

	// Read incoming pixels and form valid HxV windows
	Window2D(imWidth, imHeight, pixel_stream, window_stream);
	Pool2D(window_stream, max_stream, min_stream);
	WriteToMem(imWidth, imHeight, imWidth, min_stream, im_min);
	WriteToMem(imWidth, imHeight, imWidth, max_stream, im_max);
}

void step1SlidingWindow1(uint8_t in[imHeight * imWidth], uint8_t *im_max, uint8_t *im_min)
{
#pragma HLS INLINE
	//	sliding window as in hls book examples
	//	https://kastner.ucsd.edu/hlsbook/
	//	https://github.com/KastnerRG/pp4fpgas/blob/master/examples/video_2dfilter_linebuffer.c
	int w = imWidth, h = imHeight, s = imWidth;
	int tw = w / tilesz;
	int th = h / tilesz;

	uint8_t line_buffer[3][imWidth];
#pragma HLS array_partition variable = line_buffer complete dim = 1
	uint8_t window[4][4];
#pragma HLS array_partition variable = window complete dim = 0

rowL:
	for (int ty = 0; ty < th; ty++)
	{
	colL:
		for (int tx = 0; tx < tw; tx++)
		{
			uint8_t max = 0, min = 255;
		tyL:
			for (int dy = 0; dy < tilesz; dy++)
			{
			txL:
				for (int dx = 0; dx < tilesz; dx++)
				{
					for (int i = 0; i < 4; i++)
					{
						window[i][0] = window[i][1];
						window[i][1] = window[i][2];
						window[i][2] = window[i][3];
					}
					window[0][3] = line_buffer[0][tx * tilesz + dx];
					window[1][3] = (line_buffer[0][tx * tilesz + dx] = line_buffer[1][tx * tilesz + dx]);
					window[2][3] = (line_buffer[1][tx * tilesz + dx] = line_buffer[2][tx * tilesz + dx]);
					window[3][3] = (line_buffer[2][tx * tilesz + dx] = in[(ty * tilesz + dy) * s + tx * tilesz + dx]);
				}
			}
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					uint8_t v = window[i][j];
					if (v < min)
						min = v;
					if (v > max)
						max = v;
				}
			}
			im_max[ty * tw + tx] = max;
			im_min[ty * tw + tx] = min;
		}
	}
}

void step1SlidingWindow2(uint8_t in[imHeight * imWidth], uint8_t *im_max, uint8_t *im_min)
{
#pragma HLS INLINE
	//     sliding window implementation as in the teaching material
	int w = imWidth, h = imHeight, s = imWidth;
	int tw = w / tilesz;
	int th = h / tilesz;
	uint8_t line_buffer[3][imWidth];
#pragma HLS array_partition variable = line_buffer complete dim = 1
	uint8_t window[4][4];
#pragma HLS array_partition variable = window complete dim = 0

rowL:
	for (int ty = 0; ty < th; ty++)
	{
	colL:
		for (int tx = 0; tx < tw; tx++)
		{
			uint8_t max = 0, min = 255;
		// iterate over 4x4 tile, fill buffer
		tyL:
			for (int dy = 0; dy < tilesz; dy++)
			{
			txL:
				for (int dx = 0; dx < tilesz; dx++)
				{
					// #pragma HLS UNROLL factor=2

					// Internal variables
					uint8_t input = 0;
					int16_t output = 0;
					uint8_t buffer[4];
#pragma HLS array_partition variable = buffer complete dim = 0

					// Read input from memory
					input = in[(ty * tilesz + dy) * s + tx * tilesz + dx];

					// Read data from line_buffer to buffer
					readFromLineBuffer(input, line_buffer, buffer, tx * tilesz + dx);

					// Write data from buffer to line_buffer
					writeToLineBuffer(buffer, line_buffer, tx * tilesz + dx);

					// Move sliding window with replicated/constant border
					slidingWindow(buffer, window, tx * tilesz + dx, ty * tilesz + dy);
				}
			}

			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					uint8_t v = window[i][j];
					if (v < min)
						min = v;
					if (v > max)
						max = v;
				}
			}
			im_max[ty * tw + tx] = max;
			im_min[ty * tw + tx] = min;
		}
	}
}

void step1NativeApriltag(uint8_t in[imHeight * imWidth], uint8_t *im_max, uint8_t *im_min)
{
#pragma HLS INLINE
	// c implementation from apriltag
	int w = imWidth, h = imHeight, s = imWidth;
	int tw = w / tilesz;
	int th = h / tilesz;
l1:
	for (int ty = 0; ty < th; ty++)
	{
		for (int tx = 0; tx < tw; tx++)
		{
			uint8_t max = 0, min = 255;

			for (int dy = 0; dy < tilesz; dy++)
			{

				for (int dx = 0; dx < tilesz; dx++)
				{

					uint8_t v = in[(ty * tilesz + dy) * s + tx * tilesz + dx];
					if (v < min)
						min = v;
					if (v > max)
						max = v;
				}
			}

			im_max[ty * tw + tx] = max;
			im_min[ty * tw + tx] = min;
		}
	}
}

void threshold(uint8_t in[imWidth * imHeight], uint8_t out[imWidth * imHeight])
{
#pragma HLS INTERFACE axis port = in
#pragma HLS INTERFACE axis port = out

	int w = imWidth, h = imHeight, s = imWidth;
	// The idea is to find the maximum and minimum values in a
	// window around each pixel. If it's a contrast-free region
	// (max-min is small), don't try to binarize. Otherwise,
	// threshold according to (max+min)/2.
	//
	// Mark low-contrast regions with value 127 so that we can skip
	// future work on these areas too.

	// however, computing max/min around every pixel is needlessly
	// expensive. We compute max/min for tiles. To avoid artifacts
	// that arise when high-contrast features appear near a tile
	// edge (and thus moving from one tile to another results in a
	// large change in max/min value), the max/min values used for
	// any pixel are computed from all 3x3 surrounding tiles. Thus,
	// the max/min sampling area for nearby pixels overlap by at least
	// one tile.
	//
	// The important thing is that the windows be large enough to
	// capture edge transitions; the tag does not need to fit into
	// a tile.

	// the last (possibly partial) tiles along each row and column will
	// just use the min/max value from the last full tile.
	int tw = w / tilesz;
	int th = h / tilesz;

	uint8_t im_max[tw * th];
	uint8_t im_min[tw * th];

	//    first step: get max and min value for each 4x4 tile (max and min pooling)
	//	  4 different approaches for pooling
	step1Stream(in, im_max, im_min);
	//    step1SlidingWindow1(in, im_max, im_min);
	//    step1SlidingWindow2(in, im_max, im_min);
	//    step1NativeApriltag(in, im_max, im_min);

	// second, apply 3x3 max/min convolution to "blur" these values
	// over larger areas. This reduces artifacts due to abrupt changes
	// in the threshold value.
	uint8_t im_max_tmp[tw * th];
	uint8_t im_min_tmp[tw * th];

loopStep2:
	for (int ty = 0; ty < th; ty++)
	{
		for (int tx = 0; tx < tw; tx++)
		{
			uint8_t max = 0, min = 255;

			for (int dy = -1; dy <= 1; dy++)
			{
				if (ty + dy < 0 || ty + dy >= th)
					continue;
				for (int dx = -1; dx <= 1; dx++)
				{
					if (tx + dx < 0 || tx + dx >= tw)
						continue;

					uint8_t m = im_max[(ty + dy) * tw + tx + dx];
					if (m > max)
						max = m;
					m = im_min[(ty + dy) * tw + tx + dx];
					if (m < min)
						min = m;
				}
			}

			im_max_tmp[ty * tw + tx] = max;
			im_min_tmp[ty * tw + tx] = min;
		}
	}

	memcpy(im_max, im_max_tmp, (tw * th));
	memcpy(im_min, im_min_tmp, (tw * th));

	uint8_t lineBuffers[4][imWidth];
loopStep3:
	for (int ty = 0; ty < th; ty++)
	{
		memcpy(lineBuffers[0], &in[(ty * tilesz) * s], (imWidth));
		memcpy(lineBuffers[1], &in[(ty * tilesz + 1) * s], (imWidth));
		memcpy(lineBuffers[2], &in[(ty * tilesz + 2) * s], (imWidth));
		memcpy(lineBuffers[3], &in[(ty * tilesz + 3) * s], (imWidth));

		uint8_t lineBuffersOut[4][imWidth];
#pragma HLS array_partition variable = lineBuffersOut complete dim = 1
	l32:
		for (int tx = 0; tx < tw; tx++)
		{

			int min = im_min[ty * tw + tx];
			int max = im_max[ty * tw + tx];

			// low contrast region? (no edges)
			if (max - min < min_white_black_diff)
			{
				for (int dy = 0; dy < tilesz; dy++)
				{
					int y = ty * tilesz + dy;

					for (int dx = 0; dx < tilesz; dx++)
					{
						int x = tx * tilesz + dx;

						//                        out[y*s+x] = 127;
						lineBuffersOut[dy][x] = 127;
					}
				}
				continue;
			}

			// otherwise, actually threshold this tile.

			// argument for biasing towards dark; specular highlights
			// can be substantially brighter than white tag parts
			uint8_t thresh = min + (max - min) / 2;

			for (int dy = 0; dy < tilesz; dy++)
			{
				int y = ty * tilesz + dy;

				for (int dx = 0; dx < tilesz; dx++)
				{
					int x = tx * tilesz + dx;

					//                    uint8_t v = in[y*s+x];
					uint8_t v = lineBuffers[dy][x];
					uint8_t val = 0;
					if (v > thresh)
						//                    	out[y*s+x] = 255;
						//                    	lineBuffersOut[dy][x] = 255;
						val = 255;
					else
						//                    	out[y*s+x] = 0;
						//                    	lineBuffersOut[dy][x] = 0;
						val = 0;
					lineBuffersOut[dy][x] = val;
				}
			}
		}

		memcpy(&out[(ty * tilesz) * s], lineBuffersOut[0], (imWidth));
		memcpy(&out[(ty * tilesz + 1) * s], lineBuffersOut[1], (imWidth));
		memcpy(&out[(ty * tilesz + 2) * s], lineBuffersOut[2], (imWidth));
		memcpy(&out[(ty * tilesz + 3) * s], lineBuffersOut[3], (imWidth));
	}

// we skipped over the non-full-sized tiles above. Fix those now.
loopStep4:
	for (int y = 0; y < h; y++)
	{

		// what is the first x coordinate we need to process in this row?

		int x0;

		if (y >= th * tilesz)
		{
			x0 = 0; // we're at the bottom; do the whole row.
		}
		else
		{
			x0 = tw * tilesz; // we only need to do the right most part.
		}

		// compute tile coordinates and clamp.
		int ty = y / tilesz;
		if (ty >= th)
			ty = th - 1;

		for (int x = x0; x < w; x++)
		{
			int tx = x / tilesz;
			if (tx >= tw)
				tx = tw - 1;

			int max = im_max[ty * tw + tx];
			int min = im_min[ty * tw + tx];
			int thresh = min + (max - min) / 2;

			uint8_t v = in[y * s + x];
			if (v > thresh)
				out[y * s + x] = 255;
			else
				out[y * s + x] = 0;
		}
	}

	return;
}
