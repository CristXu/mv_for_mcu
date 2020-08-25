#include "imlib.h"
// to ease creating mati, matf, etc. in the future.
#define double float
#define TYPE double

#ifdef WIN32
#define free(ptr) (umm_free(ptr))
#define malloc(size) (umm_malloc(size))
#define realloc(ptr, size) (umm_realloc((ptr), (size)))
#define calloc(num, item_size) (umm_calloc((num), (item_size)))
#else
#define free(ptr) ({ umm_free(ptr); })
#define malloc(size) ({ void *_r = umm_malloc(size); if(!_r) fb_alloc_fail(); _r; })
#define realloc(ptr, size) ({ void *_r = umm_realloc((ptr), (size)); if(!_r) fb_alloc_fail(); _r; })
#define calloc(num, item_size) ({ void *_r = umm_calloc((num), (item_size)); if(!_r) fb_alloc_fail(); _r; })
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "math_util.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_TWOPI
# define M_TWOPI       6.2831853071795862319959  /* 2*pi */
#endif

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

#define to_radians(x) ( (x) * (M_PI / 180.0 ))
#define to_degrees(x) ( (x) * (180.0 / M_PI ))

#define max(A, B) (A < B ? B : A)
#define min(A, B) (A < B ? A : B)

  /* DEPRECATE, threshold meaningless without context.
static inline int dequals(double a, double b)
{
	double thresh = 1e-9;
	return (fabs(a-b) < thresh);
}
  */

static inline int dequals_mag(double a, double b, double thresh)
{
	return (fabs(a - b) < thresh);
}

static inline int isq(int v)
{
	return v * v;
}

static inline float fsq(float v)
{
	return v * v;
}

static inline double sq(double v)
{
	return v * v;
}

static inline double sgn(double v)
{
	return (v >= 0) ? 1 : -1;
}

// random number between [0, 1)
static inline float randf()
{
	return ((float)rand()) / (RAND_MAX + 1.0);
}


static inline float signed_randf()
{
	return randf() * 2 - 1;
}

/** Map vin to [0, 2*PI) **/
static inline double mod2pi_positive(double vin)
{
	return vin - M_TWOPI * floor(vin / M_TWOPI);
}

/** Map vin to [-PI, PI) **/
static inline double mod2pi(double vin)
{
	return mod2pi_positive(vin + M_PI) - M_PI;
}

/** Return vin such that it is within PI degrees of ref **/
static inline double mod2pi_ref(double ref, double vin)
{
	return ref + mod2pi(vin - ref);
}

/** Map vin to [0, 360) **/
static inline double mod360_positive(double vin)
{
	return vin - 360 * floor(vin / 360);
}

/** Map vin to [-180, 180) **/
static inline double mod360(double vin)
{
	return mod360_positive(vin + 180) - 180;
}
typedef struct image_u8 image_u8_t;
struct image_u8
{
	int32_t width;
	int32_t height;
	int32_t stride;

	uint8_t* buf;
};
/* class apriltag */
struct apriltag_quad_thresh_params
{
	// reject quads containing too few pixels
	int min_cluster_pixels;

	// how many corner candidates to consider when segmenting a group
	// of pixels into a quad.
	int max_nmaxima;

	// Reject quads where pairs of edges have angles that are close to
	// straight or close to 180 degrees. Zero means that no quads are
	// rejected. (In radians).
	float critical_rad;

	// When fitting lines to the contours, what is the maximum mean
	// squared error allowed?  This is useful in rejecting contours
	// that are far from being quad shaped; rejecting these quads "early"
	// saves expensive decoding processing.
	float max_line_fit_mse;

	// When we build our model of black & white pixels, we add an
	// extra check that the white model must be (overall) brighter
	// than the black model.  How much brighter? (in pixel values,
	// [0,255]). .
	int min_white_black_diff;

	// should the thresholded image be deglitched? Only useful for
	// very noisy images
	int deglitch;
};
/*
 * Defines a structure which acts as a resize-able array ala Java's ArrayList.
 */
typedef struct zarray zarray_t;
struct zarray
{
	size_t el_sz; // size of each element

	int size; // how many elements?
	int alloc; // we've allocated storage for how many elements?
	char* data;
};
/*
 * Defines a matrix structure for holding double-precision values with
 * data in row-major order (i.e. index = row*ncols + col).
 *
 * nrows and ncols are 1-based counts with the exception that a scalar (non-matrix)
 *   is represented with nrows=0 and/or ncols=0.
 */
typedef struct
{
	unsigned int nrows, ncols;
	double data[];
	//    double *data;
} matd_t;
struct quad
{
	float p[4][2]; // corners

	// H: tag coordinates ([-1,1] at the black corners) to pixels
	// Hinv: pixels to tag
	matd_t* H, * Hinv;
};
// Represents a detector object. Upon creating a detector, all fields
// are set to reasonable values, but can be overridden by accessing
// these fields.
typedef struct apriltag_detector apriltag_detector_t;
struct apriltag_detector
{
	///////////////////////////////////////////////////////////////
	// User-configurable parameters.

	// When non-zero, the edges of the each quad are adjusted to "snap
	// to" strong gradients nearby. This is useful when decimation is
	// employed, as it can increase the quality of the initial quad
	// estimate substantially. Generally recommended to be on (1).
	//
	// Very computationally inexpensive. Option is ignored if
	// quad_decimate = 1.
	int refine_edges;

	// when non-zero, detections are refined in a way intended to
	// increase the number of detected tags. Especially effective for
	// very small tags near the resolution threshold (e.g. 10px on a
	// side).
	int refine_decode;

	// when non-zero, detections are refined in a way intended to
	// increase the accuracy of the extracted pose. This is done by
	// maximizing the contrast around the black and white border of
	// the tag. This generally increases the number of successfully
	// detected tags, though not as effectively (or quickly) as
	// refine_decode.
	//
	// This option must be enabled in order for "goodness" to be
	// computed.
	int refine_pose;

	struct apriltag_quad_thresh_params qtp;

	///////////////////////////////////////////////////////////////
	// Statistics relating to last processed frame

	uint32_t nedges;
	uint32_t nsegments;
	uint32_t nquads;

	///////////////////////////////////////////////////////////////
	// Internal variables below

	// Not freed on apriltag_destroy; a tag family can be shared
	// between multiple users. The user should ultimately destroy the
	// tag family passed into the constructor.
	zarray_t* tag_families;
};

typedef struct unionfind unionfind_t;
struct unionfind
{
	uint32_t maxid;
	struct ufrec* data;
};
struct ufrec
{
	// the parent of this node. If a node's parent is its own index,
	// then it is a root.
	uint32_t parent;
};
struct uint64_zarray_entry
{
	uint64_t id;
	zarray_t* cluster;
	struct uint64_zarray_entry* next;
};
image_u8_t* threshold(apriltag_detector_t* td, image_u8_t* im)
{
	int w = im->width, h = im->height, s = im->stride;
	assert(w < 32768);
	assert(h < 32768);

	image_u8_t* threshim = fb_alloc(sizeof(image_u8_t));
	threshim->width = w;
	threshim->height = h;
	threshim->stride = s;
	threshim->buf = fb_alloc(w * h);
	assert(threshim->stride == s);

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

	// XXX Tunable. Generally, small tile sizes--- so long as they're
	// large enough to span a single tag edge--- seem to be a winner.
	const int tilesz = 4;

	// the last (possibly partial) tiles along each row and column will
	// just use the min/max value from the last full tile.
	int tw = w / tilesz;
	int th = h / tilesz;

	uint8_t* im_max = fb_alloc(tw * th * sizeof(uint8_t));
	uint8_t* im_min = fb_alloc(tw * th * sizeof(uint8_t));

	// first, collect min/max statistics for each tile
	for (int ty = 0; ty < th; ty++) {
		for (int tx = 0; tx < tw; tx++) {
			uint8_t max = 0, min = 255;

			for (int dy = 0; dy < tilesz; dy++) {

				for (int dx = 0; dx < tilesz; dx++) {

					uint8_t v = im->buf[(ty * tilesz + dy) * s + tx * tilesz + dx];
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

	// second, apply 3x3 max/min convolution to "blur" these values
	// over larger areas. This reduces artifacts due to abrupt changes
	// in the threshold value.
	if (1) {
		uint8_t* im_max_tmp = fb_alloc(tw * th * sizeof(uint8_t));
		uint8_t* im_min_tmp = fb_alloc(tw * th * sizeof(uint8_t));

		for (int ty = 0; ty < th; ty++) {
			for (int tx = 0; tx < tw; tx++) {
				uint8_t max = 0, min = 255;

				for (int dy = -1; dy <= 1; dy++) {
					if (ty + dy < 0 || ty + dy >= th)
						continue;
					for (int dx = -1; dx <= 1; dx++) {
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
		memcpy(im_max, im_max_tmp, tw * th * sizeof(uint8_t));
		memcpy(im_min, im_min_tmp, tw * th * sizeof(uint8_t));
		fb_free(); // im_min_tmp
		fb_free(); // im_max_tmp
	}

	for (int ty = 0; ty < th; ty++) {
		for (int tx = 0; tx < tw; tx++) {

			int min = im_min[ty * tw + tx];
			int max = im_max[ty * tw + tx];

			// low contrast region? (no edges)
			if (max - min < td->qtp.min_white_black_diff) {
				for (int dy = 0; dy < tilesz; dy++) {
					int y = ty * tilesz + dy;

					for (int dx = 0; dx < tilesz; dx++) {
						int x = tx * tilesz + dx;

						threshim->buf[y * s + x] = 127;
					}
				}
				continue;
			}

			// otherwise, actually threshold this tile.

			// argument for biasing towards dark; specular highlights
			// can be substantially brighter than white tag parts
			uint8_t thresh = min + (max - min) / 2;

			for (int dy = 0; dy < tilesz; dy++) {
				int y = ty * tilesz + dy;

				for (int dx = 0; dx < tilesz; dx++) {
					int x = tx * tilesz + dx;

					uint8_t v = im->buf[y * s + x];
					if (v > thresh)
						threshim->buf[y * s + x] = 255;
					else
						threshim->buf[y * s + x] = 0;
				}
			}
		}
	}

	// we skipped over the non-full-sized tiles above. Fix those now.
	if (1) {
		for (int y = 0; y < h; y++) {

			// what is the first x coordinate we need to process in this row?

			int x0;

			if (y >= th * tilesz) {
				x0 = 0; // we're at the bottom; do the whole row.
			}
			else {
				x0 = tw * tilesz; // we only need to do the right most part.
			}

			// compute tile coordinates and clamp.
			int ty = y / tilesz;
			if (ty >= th)
				ty = th - 1;

			for (int x = x0; x < w; x++) {
				int tx = x / tilesz;
				if (tx >= tw)
					tx = tw - 1;

				int max = im_max[ty * tw + tx];
				int min = im_min[ty * tw + tx];
				int thresh = min + (max - min) / 2;

				uint8_t v = im->buf[y * s + x];
				if (v > thresh)
					threshim->buf[y * s + x] = 255;
				else
					threshim->buf[y * s + x] = 0;
			}
		}
	}

	fb_free(); // im_min
	fb_free(); // im_max

	// this is a dilate/erode deglitching scheme that does not improve
	// anything as far as I can tell.
	if (0 || td->qtp.deglitch) {
		image_u8_t* tmp = fb_alloc(sizeof(image_u8_t));
		tmp->width = w;
		tmp->height = h;
		tmp->stride = s;
		tmp->buf = fb_alloc(w * h);

		for (int y = 1; y + 1 < h; y++) {
			for (int x = 1; x + 1 < w; x++) {
				uint8_t max = 0;
				for (int dy = -1; dy <= 1; dy++) {
					for (int dx = -1; dx <= 1; dx++) {
						uint8_t v = threshim->buf[(y + dy) * s + x + dx];
						if (v > max)
							max = v;
					}
				}
				tmp->buf[y * s + x] = max;
			}
		}

		for (int y = 1; y + 1 < h; y++) {
			for (int x = 1; x + 1 < w; x++) {
				uint8_t min = 255;
				for (int dy = -1; dy <= 1; dy++) {
					for (int dx = -1; dx <= 1; dx++) {
						uint8_t v = tmp->buf[(y + dy) * s + x + dx];
						if (v < min)
							min = v;
					}
				}
				threshim->buf[y * s + x] = min;
			}
		}

		fb_free(); // tmp->buf
		fb_free(); // tmp
	}

	return threshim;
}
static inline unionfind_t* unionfind_create(uint32_t maxid)
{
	unionfind_t* uf = (unionfind_t*)fb_alloc(sizeof(unionfind_t));
	uf->maxid = maxid;
	uf->data = (struct ufrec*) fb_alloc((maxid + 1) * sizeof(struct ufrec)); // the heap is at least h*w*sizeof(ufrec)
	for (int i = 0; i <= maxid; i++) {
		uf->data[i].parent = i;
	}
	return uf;
}
static inline void unionfind_destroy(void* ptr) {
	fb_free();
	fb_free();
}
// this one seems to be every-so-slightly faster than the recursive
// version above.
static inline uint32_t unionfind_get_representative(unionfind_t* uf, uint32_t id)
{
	uint32_t root = id;

	// chase down the root
	while (uf->data[root].parent != root) {
		root = uf->data[root].parent;
	}

	// go back and collapse the tree.
	//
	// XXX: on some of our workloads that have very shallow trees
	// (e.g. image segmentation), we are actually faster not doing
	// this...
	while (uf->data[id].parent != root) {
		uint32_t tmp = uf->data[id].parent;
		uf->data[id].parent = root;
		id = tmp;
	}

	return root;
}
static inline uint32_t unionfind_connect(unionfind_t* uf, uint32_t aid, uint32_t bid)
{
	uint32_t aroot = unionfind_get_representative(uf, aid);
	uint32_t broot = unionfind_get_representative(uf, bid);

	if (aroot != broot)
		uf->data[broot].parent = aroot;

	return aroot;
}
#define DO_UNIONFIND(dx, dy) if (im->buf[y*s + dy*s + x + dx] == v) unionfind_connect(uf, y*w + x, y*w + dy*w + x + dx);
static void do_unionfind_line(unionfind_t* uf, image_u8_t* im, int h, int w, int s, int y)
{
	assert(y + 1 < im->height);

	for (int x = 1; x < w - 1; x++) {
		uint8_t v = im->buf[y * s + x];

		if (v == 127)
			continue;

		// (dx,dy) pairs for 8 connectivity:
		//          (REFERENCE) (1, 0)
		// (-1, 1)    (0, 1)    (1, 1)
		//
		DO_UNIONFIND(1, 0);
		DO_UNIONFIND(0, 1);
		if (v == 255) {
			DO_UNIONFIND(-1, 1);
			DO_UNIONFIND(1, 1);
		}
	}
}
#undef DO_UNIONFIND
// limitation: image size must be <32768 in width and height. This is
// because we use a fixed-point 16 bit integer representation with one
// fractional bit.

static inline uint32_t u64hash_2(uint64_t x) {
	return (2654435761 * x) >> 32;
	return (uint32_t)x;
}
static inline zarray_t* zarray_create_fail_ok(size_t el_sz)
{
	assert(el_sz > 0);

	zarray_t* za = (zarray_t*)umm_calloc(1, sizeof(zarray_t));
	if (za) za->el_sz = el_sz;
	return za;
}
static inline void zarray_add_fail_ok(zarray_t* za, const void* p)
{
	assert(za != NULL);
	assert(p != NULL);

	if (za->size + 1 > za->alloc)
	{
		while (za->alloc < za->size + 1) {
			za->alloc += 8; // use less memory // *= 2;
			if (za->alloc < 8)
				za->alloc = 8;
		}

		za->data = (char*)umm_realloc(za->data, za->alloc * za->el_sz);

		if (!za->data) {
			// reset the zarray...
			za->size = 0;
			za->alloc = 0;
			return;
		}
	}

	memcpy(&za->data[za->size * za->el_sz], p, za->el_sz);
	za->size++;
}
/**
 * Retrieves the element from the supplied array located at the zero-based
 * index of 'idx' and copies its value into the variable pointed to by the pointer
 * 'p'.
 */
static inline void zarray_get(const zarray_t* za, int idx, void* p)
{
	assert(za != NULL);
	assert(p != NULL);
	assert(idx >= 0);
	assert(idx < za->size);

	memcpy(p, &za->data[idx * za->el_sz], za->el_sz);
}
/**
 * Retrieves the number of elements currently being contained by the passed
 * array, which may be different from its capacity. The index of the last element
 * in the array will be one less than the returned value.
 */
static inline int zarray_size(const zarray_t* za)
{
	assert(za != NULL);

	return za->size;
}
/**
 * Sets the value of the current element at index 'idx' by copying its value from
 * the data pointed to by 'p'. The previous value of the changed element will be
 * copied into the data pointed to by 'outp' if it is not null.
 */
static inline void zarray_set(zarray_t* za, int idx, const void* p, void* outp)
{
	assert(za != NULL);
	assert(p != NULL);
	assert(idx >= 0);
	assert(idx < za->size);

	if (outp != NULL)
		memcpy(outp, &za->data[idx * za->el_sz], za->el_sz);

	memcpy(&za->data[idx * za->el_sz], p, za->el_sz);
}
struct pt
{
	// Note: these represent 2*actual value.
	uint16_t x, y;
	float theta;
	int16_t gx, gy;
};
/**
 * Similar to zarray_get(), but returns a "live" pointer to the internal
 * storage, avoiding a memcpy. This pointer is not valid across
 * operations which might move memory around (i.e. zarray_remove_value(),
 * zarray_remove_index(), zarray_insert(), zarray_sort(), zarray_clear()).
 * 'p' should be a pointer to the pointer which will be set to the internal address.
 */
inline static void zarray_get_volatile(const zarray_t* za, int idx, void* p)
{
	assert(za != NULL);
	assert(p != NULL);
	assert(idx >= 0);
	assert(idx < za->size);

	*((void**)p) = &za->data[idx * za->el_sz];
}
static inline int imin(int a, int b)
{
	return (a < b) ? a : b;
}

static inline int imax(int a, int b)
{
	return (a > b) ? a : b;
}

static inline int64_t imin64(int64_t a, int64_t b)
{
	return (a < b) ? a : b;
}

static inline int64_t imax64(int64_t a, int64_t b)
{
	return (a > b) ? a : b;
}
struct line_fit_pt
{
	double Mx, My;
	double Mxx, Myy, Mxy;
	double W; // total weight
};
// lfps contains *cumulative* moments for N points, with
// index j reflecting points [0,j] (inclusive).
//
// fit a line to the points [i0, i1] (inclusive). i0, i1 are both [0,
// sz) if i1 < i0, we treat this as a wrap around.
void fit_line(struct line_fit_pt* lfps, int sz, int i0, int i1, double* lineparm, double* err, double* mse)
{
	assert(i0 != i1);
	assert(i0 >= 0 && i1 >= 0 && i0 < sz && i1 < sz);

	double Mx, My, Mxx, Myy, Mxy, W;
	int N; // how many points are included in the set?

	if (i0 < i1) {
		N = i1 - i0 + 1;

		Mx = lfps[i1].Mx;
		My = lfps[i1].My;
		Mxx = lfps[i1].Mxx;
		Mxy = lfps[i1].Mxy;
		Myy = lfps[i1].Myy;
		W = lfps[i1].W;

		if (i0 > 0) {
			Mx -= lfps[i0 - 1].Mx;
			My -= lfps[i0 - 1].My;
			Mxx -= lfps[i0 - 1].Mxx;
			Mxy -= lfps[i0 - 1].Mxy;
			Myy -= lfps[i0 - 1].Myy;
			W -= lfps[i0 - 1].W;
		}

	}
	else {
		// i0 > i1, e.g. [15, 2]. Wrap around.
		assert(i0 > 0);

		Mx = lfps[sz - 1].Mx - lfps[i0 - 1].Mx;
		My = lfps[sz - 1].My - lfps[i0 - 1].My;
		Mxx = lfps[sz - 1].Mxx - lfps[i0 - 1].Mxx;
		Mxy = lfps[sz - 1].Mxy - lfps[i0 - 1].Mxy;
		Myy = lfps[sz - 1].Myy - lfps[i0 - 1].Myy;
		W = lfps[sz - 1].W - lfps[i0 - 1].W;

		Mx += lfps[i1].Mx;
		My += lfps[i1].My;
		Mxx += lfps[i1].Mxx;
		Mxy += lfps[i1].Mxy;
		Myy += lfps[i1].Myy;
		W += lfps[i1].W;

		N = sz - i0 + i1 + 1;
	}

	assert(N >= 2);

	double Ex = Mx / W;
	double Ey = My / W;
	double Cxx = Mxx / W - Ex * Ex;
	double Cxy = Mxy / W - Ex * Ey;
	double Cyy = Myy / W - Ey * Ey;

	double nx, ny;

	if (1) {
		// on iOS about 5% of total CPU spent in these trig functions.
		// 85 ms per frame on 5S, example.pnm
		//
		// XXX this was using the double-precision atan2. Was there a case where
		// we needed that precision? Seems doubtful.
		double normal_theta = .5 * atan2f(-2 * Cxy, (Cyy - Cxx));
		nx = cosf(normal_theta);
		ny = sinf(normal_theta);
	}
	else {
		// 73.5 ms per frame on 5S, example.pnm
		double ty = -2 * Cxy;
		double tx = (Cyy - Cxx);
		double mag = ty * ty + tx * tx;

		if (mag == 0) {
			nx = 1;
			ny = 0;
		}
		else {
			double norm = sqrtf(ty * ty + tx * tx);
			tx /= norm;

			// ty is now sin(2theta)
			// tx is now cos(2theta). We want sin(theta) and cos(theta)

			// due to precision err, tx could still have slightly too large magnitude.
			if (tx > 1) {
				ny = 0;
				nx = 1;
			}
			else if (tx < -1) {
				ny = 1;
				nx = 0;
			}
			else {
				// half angle formula
				ny = sqrtf((1 - tx) / 2);
				nx = sqrtf((1 + tx) / 2);

				// pick a consistent branch cut
				if (ty < 0)
					ny = -ny;
			}
		}
	}

	if (lineparm) {
		lineparm[0] = Ex;
		lineparm[1] = Ey;
		lineparm[2] = nx;
		lineparm[3] = ny;
	}

	// sum of squared errors =
	//
	// SUM_i ((p_x - ux)*nx + (p_y - uy)*ny)^2
	// SUM_i  nx*nx*(p_x - ux)^2 + 2nx*ny(p_x -ux)(p_y-uy) + ny*ny*(p_y-uy)*(p_y-uy)
	//  nx*nx*SUM_i((p_x -ux)^2) + 2nx*ny*SUM_i((p_x-ux)(p_y-uy)) + ny*ny*SUM_i((p_y-uy)^2)
	//
	//  nx*nx*N*Cxx + 2nx*ny*N*Cxy + ny*ny*N*Cyy

	// sum of squared errors
	if (err)
		* err = nx * nx * N * Cxx + 2 * nx * ny * N * Cxy + ny * ny * N * Cyy;

	// mean squared error
	if (mse)
		* mse = nx * nx * Cxx + 2 * nx * ny * Cxy + ny * ny * Cyy;
}
int err_compare_descending(const void* _a, const void* _b)
{
	const double* a = _a;
	const double* b = _b;

	return ((*a) < (*b)) ? 1 : -1;
}
/*

  1. Identify A) white points near a black point and B) black points near a white point.

  2. Find the connected components within each of the classes above,
  yielding clusters of "white-near-black" and
  "black-near-white". (These two classes are kept separate). Each
  segment has a unique id.

  3. For every pair of "white-near-black" and "black-near-white"
  clusters, find the set of points that are in one and adjacent to the
  other. In other words, a "boundary" layer between the two
  clusters. (This is actually performed by iterating over the pixels,
  rather than pairs of clusters.) Critically, this helps keep nearby
  edges from becoming connected.
*/
int quad_segment_maxima(apriltag_detector_t* td, zarray_t* cluster, struct line_fit_pt* lfps, int indices[4])
{
	int sz = zarray_size(cluster);

	// ksz: when fitting points, how many points on either side do we consider?
	// (actual "kernel" width is 2ksz).
	//
	// This value should be about: 0.5 * (points along shortest edge).
	//
	// If all edges were equally-sized, that would give a value of
	// sz/8. We make it somewhat smaller to account for tags at high
	// aspects.

	// XXX Tunable. Maybe make a multiple of JPEG block size to increase robustness
	// to JPEG compression artifacts?
	int ksz = imin(20, sz / 12);

	// can't fit a quad if there are too few points.
	if (ksz < 2)
		return 0;

	//    printf("sz %5d, ksz %3d\n", sz, ksz);

	double* errs = fb_alloc(sz * sizeof(double));

	for (int i = 0; i < sz; i++) {
		fit_line(lfps, sz, (i + sz - ksz) % sz, (i + ksz) % sz, NULL, &errs[i], NULL);
	}

	// apply a low-pass filter to errs
	if (1) {
		double* y = fb_alloc(sz * sizeof(double));

		// how much filter to apply?

		// XXX Tunable
		double sigma = 1; // was 3

		// cutoff = exp(-j*j/(2*sigma*sigma));
		// log(cutoff) = -j*j / (2*sigma*sigma)
		// log(cutoff)*2*sigma*sigma = -j*j;

		// how big a filter should we use? We make our kernel big
		// enough such that we represent any values larger than
		// 'cutoff'.

		// XXX Tunable (though not super useful to change)
		double cutoff = 0.05;
		int fsz = sqrt(-log(cutoff) * 2 * sigma * sigma) + 1;
		fsz = 2 * fsz + 1;

		// For default values of cutoff = 0.05, sigma = 3,
		// we have fsz = 17.
		float* f = fb_alloc(fsz * sizeof(float));

		for (int i = 0; i < fsz; i++) {
			int j = i - fsz / 2;
			f[i] = exp(-j * j / (2 * sigma * sigma));
		}

		for (int iy = 0; iy < sz; iy++) {
			double acc = 0;

			for (int i = 0; i < fsz; i++) {
				acc += errs[(iy + i - fsz / 2 + sz) % sz] * f[i];
			}
			y[iy] = acc;
		}

		fb_free(); // f
		memcpy(errs, y, sz * sizeof(double));
		fb_free(); // y
	}

	int* maxima = fb_alloc(sz * sizeof(int));
	double* maxima_errs = fb_alloc(sz * sizeof(double));
	int nmaxima = 0;

	for (int i = 0; i < sz; i++) {
		if (errs[i] > errs[(i + 1) % sz] && errs[i] > errs[(i + sz - 1) % sz]) {
			maxima[nmaxima] = i;
			maxima_errs[nmaxima] = errs[i];
			nmaxima++;
		}
	}

	// if we didn't get at least 4 maxima, we can't fit a quad.
	if (nmaxima < 4)
		return 0;

	// select only the best maxima if we have too many
	int max_nmaxima = td->qtp.max_nmaxima;

	if (nmaxima > max_nmaxima) {
		double* maxima_errs_copy = fb_alloc(nmaxima * sizeof(double));
		memcpy(maxima_errs_copy, maxima_errs, nmaxima * sizeof(double));

		// throw out all but the best handful of maxima. Sorts descending.
		qsort(maxima_errs_copy, nmaxima, sizeof(double), err_compare_descending);

		double maxima_thresh = maxima_errs_copy[max_nmaxima];
		int out = 0;
		for (int in = 0; in < nmaxima; in++) {
			if (maxima_errs[in] <= maxima_thresh)
				continue;
			maxima[out++] = maxima[in];
		}
		nmaxima = out;

		fb_free(); // maxima_errs_copy
	}

	fb_free(); // maxima_errs
	fb_free(); // maxima
	fb_free(); // errs

	int best_indices[4];
	double best_error = HUGE_VALF;

	double err01, err12, err23, err30;
	double mse01, mse12, mse23, mse30;
	double params01[4], params12[4], params23[4], params30[4];

	// disallow quads where the angle is less than a critical value.
	double max_dot = cos(td->qtp.critical_rad); //25*M_PI/180);

	for (int m0 = 0; m0 < nmaxima - 3; m0++) {
		int i0 = maxima[m0];

		for (int m1 = m0 + 1; m1 < nmaxima - 2; m1++) {
			int i1 = maxima[m1];

			fit_line(lfps, sz, i0, i1, params01, &err01, &mse01);

			if (mse01 > td->qtp.max_line_fit_mse)
				continue;

			for (int m2 = m1 + 1; m2 < nmaxima - 1; m2++) {
				int i2 = maxima[m2];

				fit_line(lfps, sz, i1, i2, params12, &err12, &mse12);
				if (mse12 > td->qtp.max_line_fit_mse)
					continue;

				double dot = params01[2] * params12[2] + params01[3] * params12[3];
				if (fabs(dot) > max_dot)
					continue;

				for (int m3 = m2 + 1; m3 < nmaxima; m3++) {
					int i3 = maxima[m3];

					fit_line(lfps, sz, i2, i3, params23, &err23, &mse23);
					if (mse23 > td->qtp.max_line_fit_mse)
						continue;

					fit_line(lfps, sz, i3, i0, params30, &err30, &mse30);
					if (mse30 > td->qtp.max_line_fit_mse)
						continue;

					double err = err01 + err12 + err23 + err30;
					if (err < best_error) {
						best_error = err;
						best_indices[0] = i0;
						best_indices[1] = i1;
						best_indices[2] = i2;
						best_indices[3] = i3;
					}
				}
			}
		}
	}
	if (best_error == HUGE_VALF)
		return 0;

	for (int i = 0; i < 4; i++)
		indices[i] = best_indices[i];

	if (best_error / sz < td->qtp.max_line_fit_mse)
		return 1;
	return 0;
}
static inline void ptsort(struct pt* pts, int sz)
{
#define MAYBE_SWAP(arr,apos,bpos)                                   \
    if (arr[apos].theta > arr[bpos].theta) {                        \
        tmp = arr[apos]; arr[apos] = arr[bpos]; arr[bpos] = tmp;    \
    };

	if (sz <= 1)
		return;

	if (sz == 2) {
		struct pt tmp;
		MAYBE_SWAP(pts, 0, 1);
		return;
	}

	// NB: Using less-branch-intensive sorting networks here on the
	// hunch that it's better for performance.
	if (sz == 3) { // 3 element bubble sort is optimal
		struct pt tmp;
		MAYBE_SWAP(pts, 0, 1);
		MAYBE_SWAP(pts, 1, 2);
		MAYBE_SWAP(pts, 0, 1);
		return;
	}

	if (sz == 4) { // 4 element optimal sorting network.
		struct pt tmp;
		MAYBE_SWAP(pts, 0, 1); // sort each half, like a merge sort
		MAYBE_SWAP(pts, 2, 3);
		MAYBE_SWAP(pts, 0, 2); // minimum value is now at 0.
		MAYBE_SWAP(pts, 1, 3); // maximum value is now at end.
		MAYBE_SWAP(pts, 1, 2); // that only leaves the middle two.
		return;
	}

	if (sz == 5) {
		// this 9-step swap is optimal for a sorting network, but two
		// steps slower than a generic sort.
		struct pt tmp;
		MAYBE_SWAP(pts, 0, 1); // sort each half (3+2), like a merge sort
		MAYBE_SWAP(pts, 3, 4);
		MAYBE_SWAP(pts, 1, 2);
		MAYBE_SWAP(pts, 0, 1);
		MAYBE_SWAP(pts, 0, 3); // minimum element now at 0
		MAYBE_SWAP(pts, 2, 4); // maximum element now at end
		MAYBE_SWAP(pts, 1, 2); // now resort the three elements 1-3.
		MAYBE_SWAP(pts, 2, 3);
		MAYBE_SWAP(pts, 1, 2);
		return;
	}

#undef MAYBE_SWAP

	// a merge sort with temp storage.

	struct pt* tmp = fb_alloc(sizeof(struct pt) * sz);

	memcpy(tmp, pts, sizeof(struct pt) * sz);

	int asz = sz / 2;
	int bsz = sz - asz;

	struct pt* as = &tmp[0];
	struct pt* bs = &tmp[asz];

	ptsort(as, asz);
	ptsort(bs, bsz);

#define MERGE(apos,bpos)                        \
    if (as[apos].theta < bs[bpos].theta)        \
        pts[outpos++] = as[apos++];             \
    else                                        \
        pts[outpos++] = bs[bpos++];

	int apos = 0, bpos = 0, outpos = 0;
	while (apos + 8 < asz && bpos + 8 < bsz) {
		MERGE(apos, bpos); MERGE(apos, bpos); MERGE(apos, bpos); MERGE(apos, bpos);
		MERGE(apos, bpos); MERGE(apos, bpos); MERGE(apos, bpos); MERGE(apos, bpos);
	}

	while (apos < asz && bpos < bsz) {
		MERGE(apos, bpos);
	}

	if (apos < asz)
		memcpy(&pts[outpos], &as[apos], (asz - apos) * sizeof(struct pt));
	if (bpos < bsz)
		memcpy(&pts[outpos], &bs[bpos], (bsz - bpos) * sizeof(struct pt));

	fb_free(); // tmp

#undef MERGE
}
inline static void zarray_truncate(zarray_t* za, int sz)
{
	assert(za != NULL);
	assert(sz <= za->size);
	za->size = sz;
}
// return 1 if the quad looks okay, 0 if it should be discarded
int fit_quad(apriltag_detector_t* td, image_u8_t* im, zarray_t* cluster, struct quad* quad, bool overrideMode)
{
	int res = 0;

	int sz = zarray_size(cluster);
	if (sz < 4) // can't fit a quad to less than 4 points
		return 0;

	/////////////////////////////////////////////////////////////
	// Step 1. Sort points so they wrap around the center of the
	// quad. We will constrain our quad fit to simply partition this
	// ordered set into 4 groups.

	// compute a bounding box so that we can order the points
	// according to their angle WRT the center.
	int32_t xmax = 0, xmin = INT32_MAX, ymax = 0, ymin = INT32_MAX;

	for (int pidx = 0; pidx < zarray_size(cluster); pidx++) {
		struct pt* p;
		zarray_get_volatile(cluster, pidx, &p);

		xmax = imax(xmax, p->x);
		xmin = imin(xmin, p->x);

		ymax = imax(ymax, p->y);
		ymin = imin(ymin, p->y);
	}

	// add some noise to (cx,cy) so that pixels get a more diverse set
	// of theta estimates. This will help us remove more points.
	// (Only helps a small amount. The actual noise values here don't
	// matter much at all, but we want them [-1, 1]. (XXX with
	// fixed-point, should range be bigger?)
	double cx = (xmin + xmax) * 0.5 + 0.05118;
	double cy = (ymin + ymax) * 0.5 + -0.028581;

	double dot = 0;

	for (int pidx = 0; pidx < zarray_size(cluster); pidx++) {
		struct pt* p;
		zarray_get_volatile(cluster, pidx, &p);

		double dx = p->x - cx;
		double dy = p->y - cy;

		p->theta = atan2f(dy, dx);

		dot += dx * p->gx + dy * p->gy;
		//        p->theta = terrible_atan2(dy, dx);
	}

	// Ensure that the black border is inside the white border.
	if ((!overrideMode) && (dot < 0))
		return 0;

	// we now sort the points according to theta. This is a prepatory
	// step for segmenting them into four lines.
	if (1) {
		//        zarray_sort(cluster, pt_compare_theta);
		ptsort((struct pt*) cluster->data, zarray_size(cluster));

		// remove duplicate points. (A byproduct of our segmentation system.)
		if (1) {
			int outpos = 1;

			struct pt* last;
			zarray_get_volatile(cluster, 0, &last);

			for (int i = 1; i < sz; i++) {

				struct pt* p;
				zarray_get_volatile(cluster, i, &p);

				if (p->x != last->x || p->y != last->y) {

					if (i != outpos) {
						struct pt* out;
						zarray_get_volatile(cluster, outpos, &out);
						memcpy(out, p, sizeof(struct pt));
					}

					outpos++;
				}

				last = p;
			}

			cluster->size = outpos;
			sz = outpos;
		}

	}
	else {
		// This is a counting sort in which we retain at most one
		// point for every bucket; the bucket index is computed from
		// theta. Since a good quad completes a complete revolution,
		// there's reason to think that we should get a good
		// distribution of thetas.  We might "lose" a few points due
		// to collisions, but this shouldn't affect quality very much.

		// XXX tunable. Increase to reduce the likelihood of "losing"
		// points due to collisions.
		int nbuckets = 4 * sz;

#define ASSOC 2
#ifdef WIN32
		// no support the un-certain length array in common-c 
		// struct pt* v[nbuckets]
		struct pt** v = (struct pt**)win_malloc(sizeof(struct pt*) * nbuckets);
		for (int i = 0;i < ASSOC; i++) {
			v[i] = (struct pt*)win_malloc(sizeof(struct pt));
			memset(v[i], 0, sizeof(struct pt));
		}
#else
		struct pt v[nbuckets][ASSOC];
		memset(v, 0, sizeof(v));
#endif


		// put each point into a bucket.
		for (int i = 0; i < sz; i++) {
			struct pt* p;
			zarray_get_volatile(cluster, i, &p);

			assert(p->theta >= -M_PI && p->theta <= M_PI);

			int bucket = (nbuckets - 1) * (p->theta + M_PI) / (2 * M_PI);
			assert(bucket >= 0 && bucket < nbuckets);

			for (int i = 0; i < ASSOC; i++) {
				if (v[bucket][i].theta == 0) {
					v[bucket][i] = *p;
					break;
				}
			}
		}

		// collect the points from the buckets and put them back into the array.
		int outsz = 0;
		for (int i = 0; i < nbuckets; i++) {
			for (int j = 0; j < ASSOC; j++) {
				if (v[i][j].theta != 0) {
					zarray_set(cluster, outsz, &v[i][j], NULL);
					outsz++;
				}
			}
		}
#ifdef WIN32
		// free the v 
		for (int i = 0;i < ASSOC; i++)
			win_free(v[i]);
		win_free(v);
		v = NULL;
#endif
		zarray_truncate(cluster, outsz);
		sz = outsz;
	}
	if (sz < 4)
		return 0;

	/////////////////////////////////////////////////////////////
	// Step 2. Precompute statistics that allow line fit queries to be
	// efficiently computed for any contiguous range of indices.

	struct line_fit_pt* lfps = fb_alloc0(sz * sizeof(struct line_fit_pt));

	for (int i = 0; i < sz; i++) {
		struct pt* p;
		zarray_get_volatile(cluster, i, &p);

		if (i > 0) {
			memcpy(&lfps[i], &lfps[i - 1], sizeof(struct line_fit_pt));
		}

		if (0) {
			// we now undo our fixed-point arithmetic.
			double delta = 0.5;
			double x = p->x * .5 + delta;
			double y = p->y * .5 + delta;
			double W;

			for (int dy = -1; dy <= 1; dy++) {
				int iy = y + dy;

				if (iy < 0 || iy + 1 >= im->height)
					continue;

				for (int dx = -1; dx <= 1; dx++) {
					int ix = x + dx;

					if (ix < 0 || ix + 1 >= im->width)
						continue;

					int grad_x = im->buf[iy * im->stride + ix + 1] -
						im->buf[iy * im->stride + ix - 1];

					int grad_y = im->buf[(iy + 1) * im->stride + ix] -
						im->buf[(iy - 1) * im->stride + ix];

					W = sqrtf(grad_x * grad_x + grad_y * grad_y) + 1;

					//                    double fx = x + dx, fy = y + dy;
					double fx = ix + .5, fy = iy + .5;
					lfps[i].Mx += W * fx;
					lfps[i].My += W * fy;
					lfps[i].Mxx += W * fx * fx;
					lfps[i].Mxy += W * fx * fy;
					lfps[i].Myy += W * fy * fy;
					lfps[i].W += W;
				}
			}
		}
		else {
			// we now undo our fixed-point arithmetic.
			double delta = 0.5; // adjust for pixel center bias
			double x = p->x * .5 + delta;
			double y = p->y * .5 + delta;
			int ix = x, iy = y;
			double W = 1;

			if (ix > 0 && ix + 1 < im->width && iy > 0 && iy + 1 < im->height) {
				int grad_x = im->buf[iy * im->stride + ix + 1] -
					im->buf[iy * im->stride + ix - 1];

				int grad_y = im->buf[(iy + 1) * im->stride + ix] -
					im->buf[(iy - 1) * im->stride + ix];

				// XXX Tunable. How to shape the gradient magnitude?
				W = sqrt(grad_x * grad_x + grad_y * grad_y) + 1;
			}

			double fx = x, fy = y;
			lfps[i].Mx += W * fx;
			lfps[i].My += W * fy;
			lfps[i].Mxx += W * fx * fx;
			lfps[i].Mxy += W * fx * fy;
			lfps[i].Myy += W * fy * fy;
			lfps[i].W += W;
		}
	}

	int indices[4];
	if (1) {
		if (!quad_segment_maxima(td, cluster, lfps, indices))
			goto finish;
	}

	//    printf("%d %d %d %d\n", indices[0], indices[1], indices[2], indices[3]);

	if (0) {
		// no refitting here; just use those points as the vertices.
		// Note, this is useful for debugging, but pretty bad in
		// practice since this code path also omits several
		// plausibility checks that save us tons of time in quad
		// decoding.
		for (int i = 0; i < 4; i++) {
			struct pt* p;
			zarray_get_volatile(cluster, indices[i], &p);

			quad->p[i][0] = .5 * p->x; // undo fixed-point arith.
			quad->p[i][1] = .5 * p->y;
		}

		res = 1;

	}
	else {
		double lines[4][4];

		for (int i = 0; i < 4; i++) {
			int i0 = indices[i];
			int i1 = indices[(i + 1) & 3];

			if (0) {
				// if there are enough points, skip the points near the corners
				// (because those tend not to be very good.)
				if (i1 - i0 > 8) {
					int t = (i1 - i0) / 6;
					if (t < 0)
						t = -t;

					i0 = (i0 + t) % sz;
					i1 = (i1 + sz - t) % sz;
				}
			}

			double err;
			fit_line(lfps, sz, i0, i1, lines[i], NULL, &err);

			if (err > td->qtp.max_line_fit_mse) {
				res = 0;
				goto finish;
			}
		}

		for (int i = 0; i < 4; i++) {
			// solve for the intersection of lines (i) and (i+1)&3.
			// p0 + lambda0*u0 = p1 + lambda1*u1, where u0 and u1
			// are the line directions.
			//
			// lambda0*u0 - lambda1*u1 = (p1 - p0)
			//
			// rearrange (solve for lambdas)
			//
			// [u0_x   -u1_x ] [lambda0] = [ p1_x - p0_x ]
			// [u0_y   -u1_y ] [lambda1]   [ p1_y - p0_y ]
			//
			// remember that lines[i][0,1] = p, lines[i][2,3] = NORMAL vector.
			// We want the unit vector, so we need the perpendiculars. Thus, below
			// we have swapped the x and y components and flipped the y components.

			double A00 = lines[i][3], A01 = -lines[(i + 1) & 3][3];
			double A10 = -lines[i][2], A11 = lines[(i + 1) & 3][2];
			double B0 = -lines[i][0] + lines[(i + 1) & 3][0];
			double B1 = -lines[i][1] + lines[(i + 1) & 3][1];

			double det = A00 * A11 - A10 * A01;

			// inverse.
			double W00 = A11 / det, W01 = -A01 / det;
			if (fabs(det) < 0.001) {
				res = 0;
				goto finish;
			}

			// solve
			double L0 = W00 * B0 + W01 * B1;

			// compute intersection
			quad->p[i][0] = lines[i][0] + L0 * A00;
			quad->p[i][1] = lines[i][1] + L0 * A10;

			if (0) {
				// we should get the same intersection starting
				// from point p1 and moving L1*u1.
				double W10 = -A10 / det, W11 = A00 / det;
				double L1 = W10 * B0 + W11 * B1;

				double x = lines[(i + 1) & 3][0] - L1 * A10;
				double y = lines[(i + 1) & 3][1] - L1 * A11;
				assert(fabs(x - quad->p[i][0]) < 0.001 &&
					fabs(y - quad->p[i][1]) < 0.001);
			}

			res = 1;
		}
	}

	// reject quads that are too small
	if (1) {
		double area = 0;

		// get area of triangle formed by points 0, 1, 2, 0
		double length[3], p;
		for (int i = 0; i < 3; i++) {
			int idxa = i; // 0, 1, 2,
			int idxb = (i + 1) % 3; // 1, 2, 0
			length[i] = sqrt(sq(quad->p[idxb][0] - quad->p[idxa][0]) +
				sq(quad->p[idxb][1] - quad->p[idxa][1]));
		}
		p = (length[0] + length[1] + length[2]) / 2;

		area += sqrt(p * (p - length[0]) * (p - length[1]) * (p - length[2]));

		// get area of triangle formed by points 2, 3, 0, 2
		for (int i = 0; i < 3; i++) {
			int idxs[] = { 2, 3, 0, 2 };
			int idxa = idxs[i];
			int idxb = idxs[i + 1];
			length[i] = sqrt(sq(quad->p[idxb][0] - quad->p[idxa][0]) +
				sq(quad->p[idxb][1] - quad->p[idxa][1]));
		}
		p = (length[0] + length[1] + length[2]) / 2;

		area += sqrt(p * (p - length[0]) * (p - length[1]) * (p - length[2]));

		// we don't actually know the family yet (quad detection is generic.)
		// This threshold is based on a 6x6 tag (which is actually 8x8)
//        int d = fam->d + fam->black_border*2;
		int d = 8;
		if (area < d * d) {
			res = 0;
			goto finish;
		}
	}

	// reject quads whose cumulative angle change isn't equal to 2PI
	if (1) {
		double total = 0;

		for (int i = 0; i < 4; i++) {
			int i0 = i, i1 = (i + 1) & 3, i2 = (i + 2) & 3;

			double theta0 = atan2f(quad->p[i0][1] - quad->p[i1][1],
				quad->p[i0][0] - quad->p[i1][0]);
			double theta1 = atan2f(quad->p[i2][1] - quad->p[i1][1],
				quad->p[i2][0] - quad->p[i1][0]);

			double dtheta = theta0 - theta1;
			if (dtheta < 0)
				dtheta += 2 * M_PI;

			if (dtheta < td->qtp.critical_rad || dtheta >(M_PI - td->qtp.critical_rad))
				res = 0;

			total += dtheta;
		}

		// looking for 2PI
		if (total < 6.2 || total > 6.4) {
			res = 0;
			goto finish;
		}
	}

	// adjust pixel coordinates; all math up 'til now uses pixel
	// coordinates in which (0,0) is the lower left corner. But each
	// pixel actually spans from to [x, x+1), [y, y+1) the mean value of which
	// is +.5 higher than x & y.
/*    double delta = .5;
	  for (int i = 0; i < 4; i++) {
	  quad->p[i][0] += delta;
	  quad->p[i][1] += delta;
	  }
*/
finish:

	fb_free(); // lfps

	return res;
}
/**
 * Creates and returns a variable array structure capable of holding elements of
 * the specified size. It is the caller's responsibility to call zarray_destroy()
 * on the returned array when it is no longer needed.
 */
static inline zarray_t* zarray_create(size_t el_sz)
{
	assert(el_sz > 0);

	zarray_t* za = (zarray_t*)calloc(1, sizeof(zarray_t));
	za->el_sz = el_sz;
	return za;
}
/**
 * Frees all resources associated with the variable array structure which was
 * created by zarray_create(). After calling, 'za' will no longer be valid for storage.
 */
static inline void zarray_destroy(zarray_t* za)
{
	if (za == NULL)
		return;

	if (za->data != NULL)
		free(za->data);
	memset(za, 0, sizeof(zarray_t));
	free(za);
}
/**
 * Allocates enough internal storage in the supplied variable array structure to
 * guarantee that the supplied number of elements (capacity) can be safely stored.
 */
static inline void zarray_ensure_capacity(zarray_t* za, int capacity)
{
	assert(za != NULL);

	if (capacity <= za->alloc)
		return;

	while (za->alloc < capacity) {
		za->alloc += 8; // use less memory // *= 2;
		if (za->alloc < 8)
			za->alloc = 8;
	}

	za->data = (char*)realloc(za->data, za->alloc * za->el_sz);
}
/**
 * Adds a new element to the end of the supplied array, and sets its value
 * (by copying) from the data pointed to by the supplied pointer 'p'.
 * Automatically ensures that enough storage space is available for the new element.
 */
static inline void zarray_add(zarray_t* za, const void* p)
{
	assert(za != NULL);
	assert(p != NULL);

	zarray_ensure_capacity(za, za->size + 1);

	memcpy(&za->data[za->size * za->el_sz], p, za->el_sz);
	za->size++;
}
// Represents a tag family. Every tag belongs to a tag family. Tag
// families are generated by the Java tool
// april.tag.TagFamilyGenerator and can be converted to C using
// april.tag.TagToC.
typedef struct apriltag_family apriltag_family_t;
struct apriltag_family
{
	// How many codes are there in this tag family?
	uint32_t ncodes;

	// how wide (in bit-sizes) is the black border? (usually 1)
	uint32_t black_border;

	// how many bits tall and wide is it? (e.g. 36bit tag ==> 6)
	uint32_t d;

	// minimum hamming distance between any two codes. (e.g. 36h11 => 11)
	uint32_t h;

	// The codes in the family.
	uint64_t codes[];
};
apriltag_detector_t* apriltag_detector_create()
{
	apriltag_detector_t* td = (apriltag_detector_t*)calloc(1, sizeof(apriltag_detector_t));

	td->qtp.max_nmaxima = 10;
	td->qtp.min_cluster_pixels = 5;

	td->qtp.max_line_fit_mse = 10.0;
	td->qtp.critical_rad = 10 * M_PI / 180;
	td->qtp.deglitch = 0;
	td->qtp.min_white_black_diff = 5;

	td->tag_families = zarray_create(sizeof(apriltag_family_t*));

	td->refine_edges = 1;
	td->refine_pose = 0;
	td->refine_decode = 0;

	return td;
}
zarray_t* apriltag_quad_thresh(apriltag_detector_t* td, image_u8_t* im, bool overrideMode)
{
	////////////////////////////////////////////////////////
	// step 1. threshold the image, creating the edge image.

	int w = im->width, h = im->height;

	image_u8_t* threshim = threshold(td, im);
	int ts = threshim->stride;

	////////////////////////////////////////////////////////
	// step 2. find connected components.

	unionfind_t* uf = unionfind_create(w * h);

	for (int y = 0; y < h - 1; y++) {
		do_unionfind_line(uf, threshim, h, w, ts, y);
	}


    uint32_t nclustermap;
    struct uint64_zarray_entry **clustermap = fb_alloc0_all(&nclustermap);
    nclustermap /= sizeof(struct uint64_zarray_entry*);
    if (!nclustermap) fb_alloc_fail();

	for (int y = 1; y < h - 1; y++) {
		for (int x = 1; x < w - 1; x++) {

			uint8_t v0 = threshim->buf[y * ts + x];
			if (v0 == 127)
				continue;

			// XXX don't query this until we know we need it?
			uint64_t rep0 = unionfind_get_representative(uf, y * w + x);

			// whenever we find two adjacent pixels such that one is
			// white and the other black, we add the point half-way
			// between them to a cluster associated with the unique
			// ids of the white and black regions.
			//
			// We additionally compute the gradient direction (i.e., which
			// direction was the white pixel?) Note: if (v1-v0) == 255, then
			// (dx,dy) points towards the white pixel. if (v1-v0) == -255, then
			// (dx,dy) points towards the black pixel. p.gx and p.gy will thus
			// be -255, 0, or 255.
			//
			// Note that any given pixel might be added to multiple
			// different clusters. But in the common case, a given
			// pixel will be added multiple times to the same cluster,
			// which increases the size of the cluster and thus the
			// computational costs.
			//
			// A possible optimization would be to combine entries
			// within the same cluster.

#define DO_CONN(dx, dy)                                                 \
            if (1) {                                                    \
                uint8_t v1 = threshim->buf[y*ts + dy*ts + x + dx];      \
                                                                        \
                while (v0 + v1 == 255) {                                   \
                    uint64_t rep1 = unionfind_get_representative(uf, y*w + dy*w + x + dx); \
                    uint64_t clusterid;                                 \
                    if (rep0 < rep1)                                    \
                        clusterid = (rep1 << 32) + rep0;                \
                    else                                                \
                        clusterid = (rep0 << 32) + rep1;                \
                                                                        \
                    /* XXX lousy hash function */                       \
                    uint32_t clustermap_bucket = u64hash_2(clusterid) % nclustermap; \
                    struct uint64_zarray_entry *entry = clustermap[clustermap_bucket]; \
                    while (entry && entry->id != clusterid)     {       \
                        entry = entry->next;                            \
                    }                                                   \
                                                                        \
                    if (!entry) {                                       \
                        entry = umm_calloc(1 , sizeof(struct uint64_zarray_entry)); \
                        if (!entry) break;                              \
                        entry->id = clusterid;                          \
                        entry->cluster = zarray_create_fail_ok(sizeof(struct pt)); \
                        if (!entry->cluster) {                          \
                            free(entry);                                \
                            break;                                      \
                        }                                               \
                        entry->next = clustermap[clustermap_bucket];    \
                        clustermap[clustermap_bucket] = entry;          \
                    }                                                   \
                                                                        \
                    struct pt p = { .x = 2*x + dx, .y = 2*y + dy, .gx = dx*((int) v1-v0), .gy = dy*((int) v1-v0)}; \
                    zarray_add_fail_ok(entry->cluster, &p);             \
                    break;                                              \
                }                                                       \
            }

			// do 4 connectivity. NB: Arguments must be [-1, 1] or we'll overflow .gx, .gy
			DO_CONN(1, 0);
			DO_CONN(0, 1);

			// do 8 connectivity
			// DO_CONN(-1, 1);
			// DO_CONN(1, 1);
		}
	}
#undef DO_CONN

	////////////////////////////////////////////////////////
	// step 3. process each connected component.
	zarray_t* clusters = zarray_create_fail_ok(sizeof(zarray_t*)); //, uint64_zarray_hash_size(clustermap));
	if (clusters) {
		for (int i = 0; i < nclustermap; i++) {

			for (struct uint64_zarray_entry* entry = clustermap[i]; entry; entry = entry->next) {
				// XXX reject clusters here?
				zarray_add_fail_ok(clusters, &entry->cluster);
			}
		}
	}


	int sz = clusters ? zarray_size(clusters) : 0;

	if (1) {
		for (int i = 0; i < nclustermap; i++) {
			struct uint64_zarray_entry* entry = clustermap[i];
			while (entry) {
				// free any leaked cluster (zarray_add_fail_ok)
				bool leaked = true;
				for (int j = 0; j < sz; j++) {
					zarray_t* cluster;
					zarray_get(clusters, j, &cluster);
					leaked &= entry->cluster != cluster;
				}
				if (leaked) free(entry->cluster);
				struct uint64_zarray_entry* tmp = entry->next;
				free(entry);
				entry = tmp;
			}
		}
		fb_free(); // clustermap
	}

	unionfind_destroy(uf);  //uf

	fb_free(); // threshim->buf
	fb_free(); // threshim

	zarray_t* quads = zarray_create_fail_ok(sizeof(struct quad));

	if (quads) {
		for (int i = 0; i < sz; i++) {

			zarray_t* cluster;
			zarray_get(clusters, i, &cluster);

			if (zarray_size(cluster) < td->qtp.min_cluster_pixels)
				continue;

			// a cluster should contain only boundary points around the
			// tag. it cannot be bigger than the whole screen. (Reject
			// large connected blobs that will be prohibitively slow to
			// fit quads to.) A typical point along an edge is added three
			// times (because it has 3 neighbors). The maximum perimeter
			// is 2w+2h.
			if (zarray_size(cluster) > 3 * (2 * w + 2 * h)) {
				continue;
			}

			struct quad quad;
			memset(&quad, 0, sizeof(struct quad));

			if (fit_quad(td, im, cluster, &quad, overrideMode)) {

				zarray_add_fail_ok(quads, &quad);
			}
		}
	}

	//        printf("  %d %d %d %d\n", indices[0], indices[1], indices[2], indices[3]);

	// got some free error, comment these temporary
	// fixed.... without these lines, will leave some clusters un-free, caused the mem_dump....
	for (int i = 0; i < sz; i++) {
		zarray_t* cluster;
		zarray_get(clusters, i, &cluster);
		zarray_destroy(cluster);
	}

	if (clusters) zarray_destroy(clusters);


	if (!quads) {
		// we should have enough memory now
		quads = zarray_create(sizeof(struct quad));
	}
	return quads;
}
static void refine_edges(apriltag_detector_t* td, image_u8_t* im_orig, struct quad* quad)
{
	double lines[4][4]; // for each line, [Ex Ey nx ny]

	for (int edge = 0; edge < 4; edge++) {
		int a = edge, b = (edge + 1) & 3; // indices of the end points.

		// compute the normal to the current line estimate
		double nx = quad->p[b][1] - quad->p[a][1];
		double ny = -quad->p[b][0] + quad->p[a][0];
		double mag = sqrt(nx * nx + ny * ny);
		nx /= mag;
		ny /= mag;

		// we will now fit a NEW line by sampling points near
		// our original line that have large gradients. On really big tags,
		// we're willing to sample more to get an even better estimate.
		int nsamples = imax(16, mag / 8); // XXX tunable

		// stats for fitting a line...
		double Mx = 0, My = 0, Mxx = 0, Mxy = 0, Myy = 0, N = 0;

		for (int s = 0; s < nsamples; s++) {
			// compute a point along the line... Note, we're avoiding
			// sampling *right* at the corners, since those points are
			// the least reliable.
			double alpha = (1.0 + s) / (nsamples + 1);
			double x0 = alpha * quad->p[a][0] + (1 - alpha) * quad->p[b][0];
			double y0 = alpha * quad->p[a][1] + (1 - alpha) * quad->p[b][1];

			// search along the normal to this line, looking at the
			// gradients along the way. We're looking for a strong
			// response.
			double Mn = 0;
			double Mcount = 0;

			// XXX tunable: how far to search?  We want to search far
			// enough that we find the best edge, but not so far that
			// we hit other edges that aren't part of the tag. We
			// shouldn't ever have to search more than quad_decimate,
			// since otherwise we would (ideally) have started our
			// search on another pixel in the first place. Likewise,
			// for very small tags, we don't want the range to be too
			// big.
			double range = 1.0 + 1;

			// XXX tunable step size.
			for (double n = -range; n <= range; n += 0.25) {
				// Because of the guaranteed winding order of the
				// points in the quad, we will start inside the white
				// portion of the quad and work our way outward.
				//
				// sample to points (x1,y1) and (x2,y2) XXX tunable:
				// how far +/- to look? Small values compute the
				// gradient more precisely, but are more sensitive to
				// noise.
				double grange = 1;
				int x1 = x0 + (n + grange) * nx;
				int y1 = y0 + (n + grange) * ny;
				if (x1 < 0 || x1 >= im_orig->width || y1 < 0 || y1 >= im_orig->height)
					continue;

				int x2 = x0 + (n - grange) * nx;
				int y2 = y0 + (n - grange) * ny;
				if (x2 < 0 || x2 >= im_orig->width || y2 < 0 || y2 >= im_orig->height)
					continue;

				int g1 = im_orig->buf[y1 * im_orig->stride + x1];
				int g2 = im_orig->buf[y2 * im_orig->stride + x2];

				if (g1 < g2) // reject points whose gradient is "backwards". They can only hurt us.
					continue;

				double weight = (g2 - g1) * (g2 - g1); // XXX tunable. What shape for weight=f(g2-g1)?

				// compute weighted average of the gradient at this point.
				Mn += weight * n;
				Mcount += weight;
			}

			// what was the average point along the line?
			if (Mcount == 0)
				continue;

			double n0 = Mn / Mcount;

			// where is the point along the line?
			double bestx = x0 + n0 * nx;
			double besty = y0 + n0 * ny;

			// update our line fit statistics
			Mx += bestx;
			My += besty;
			Mxx += bestx * bestx;
			Mxy += bestx * besty;
			Myy += besty * besty;
			N++;
		}

		// fit a line
		double Ex = Mx / N, Ey = My / N;
		double Cxx = Mxx / N - Ex * Ex;
		double Cxy = Mxy / N - Ex * Ey;
		double Cyy = Myy / N - Ey * Ey;

		double normal_theta = .5 * atan2f(-2 * Cxy, (Cyy - Cxx));
		nx = cosf(normal_theta);
		ny = sinf(normal_theta);
		lines[edge][0] = Ex;
		lines[edge][1] = Ey;
		lines[edge][2] = nx;
		lines[edge][3] = ny;
	}

	// now refit the corners of the quad
	for (int i = 0; i < 4; i++) {

		// solve for the intersection of lines (i) and (i+1)&3.
		double A00 = lines[i][3], A01 = -lines[(i + 1) & 3][3];
		double A10 = -lines[i][2], A11 = lines[(i + 1) & 3][2];
		double B0 = -lines[i][0] + lines[(i + 1) & 3][0];
		double B1 = -lines[i][1] + lines[(i + 1) & 3][1];

		double det = A00 * A11 - A10 * A01;

		// inverse.
		if (fabs(det) > 0.001) {
			// solve
			double W00 = A11 / det, W01 = -A01 / det;

			double L0 = W00 * B0 + W01 * B1;

			// compute intersection
			quad->p[i][0] = lines[i][0] + L0 * A00;
			quad->p[i][1] = lines[i][1] + L0 * A10;
		}
		else {
			// this is a bad sign. We'll just keep the corner we had.
//            printf("bad det: %15f %15f %15f %15f %15f\n", A00, A11, A10, A01, det);
		}
	}
}
void matd_destroy(matd_t* m)
{
	if (!m)
		return;

	assert(m != NULL);
	free(m);
}
/**
 * Determines whether the supplied matrix 'a' is a scalar (positive return) or
 * not (zero return, indicating a matrix of dimensions at least 1x1).
 */
static inline int matd_is_scalar(const matd_t* a)
{
	assert(a != NULL);
	return a->ncols == 0 || a->nrows == 0;
}
matd_t* matd_create_scalar(TYPE v)
{
	matd_t* m = calloc(1, sizeof(matd_t) + sizeof(double));
	m->nrows = 0;
	m->ncols = 0;
	m->data[0] = v;

	return m;
}
/**
 * A macro to reference a specific matd_t data element given it's zero-based
 * row and column indexes. Suitable for both retrieval and assignment.
 */
#define MATD_EL(m, row, col) (m)->data[((row)*(m)->ncols + (col))]
matd_t* matd_create(int rows, int cols)
{
	assert(rows >= 0);
	assert(cols >= 0);

	if (rows == 0 || cols == 0)
		return matd_create_scalar(0);

	matd_t* m = calloc(1, sizeof(matd_t) + (rows * cols * sizeof(double)));
	m->nrows = rows;
	m->ncols = cols;

	return m;
}
matd_t* matd_create_data(int rows, int cols, const TYPE* data)
{
	if (rows == 0 || cols == 0)
		return matd_create_scalar(data[0]);

	matd_t* m = matd_create(rows, cols);
	for (int i = 0; i < rows * cols; i++)
		m->data[i] = data[i];

	return m;
}
// All square matrices (even singular ones) have a partially-pivoted
// LU decomposition such that A = PLU, where P is a permutation
// matrix, L is a lower triangular matrix, and U is an upper
// triangular matrix.
//
typedef struct
{
	// was the input matrix singular? When a zero pivot is found, this
	// flag is set to indicate that this has happened.
	int singular;

	unsigned int* piv; // permutation indices
	int pivsign; // either +1 or -1

	// The matd_plu_t object returned "owns" the enclosed LU matrix. It
	// is not expected that the returned object is itself useful to
	// users: it contains the L and U information all smushed
	// together.
	matd_t* lu; // combined L and U matrices, permuted so they can be triangular.
} matd_plu_t;
/**
 * Defines a small value which can be used in place of zero for approximating
 * calculations which are singular at zero values (i.e. inverting a matrix with
 * a zero or near-zero determinant).
 */
#define MATD_EPS 1e-8
matd_t * matd_copy(const matd_t * m)
{
	assert(m != NULL);

	matd_t* x = matd_create(m->nrows, m->ncols);
	if (matd_is_scalar(m))
		x->data[0] = m->data[0];
	else
		memcpy(x->data, m->data, sizeof(TYPE) * m->ncols * m->nrows);

	return x;
}
// returns NULL if the matrix is (exactly) singular. Caller is
// otherwise responsible for knowing how to cope with badly
// conditioned matrices.
matd_plu_t* matd_plu(const matd_t* a)
{
	unsigned int* piv = calloc(a->nrows, sizeof(unsigned int));
	int pivsign = 1;
	matd_t* lu = matd_copy(a);

	// only for square matrices.
	assert(a->nrows == a->ncols);

	matd_plu_t* mlu = calloc(1, sizeof(matd_plu_t));

	for (int i = 0; i < a->nrows; i++)
		piv[i] = i;

	for (int j = 0; j < a->ncols; j++) {
		for (int i = 0; i < a->nrows; i++) {
			int kmax = i < j ? i : j; // min(i,j)

			// compute dot product of row i with column j (up through element kmax)
			double acc = 0;
			for (int k = 0; k < kmax; k++)
				acc += MATD_EL(lu, i, k) * MATD_EL(lu, k, j);

			MATD_EL(lu, i, j) -= acc;
		}

		// find pivot and exchange if necessary.
		int p = j;
		if (1) {
			for (int i = j + 1; i < lu->nrows; i++) {
				if (fabs(MATD_EL(lu, i, j)) > fabs(MATD_EL(lu, p, j))) {
					p = i;
				}
			}
		}

		// swap rows p and j?
		if (p != j) {
#ifdef WIN32
			TYPE* tmp = win_malloc((lu->ncols) * sizeof(TYPE));
#else
			TYPE tmp[lu->ncols];
#endif
			memcpy(tmp, &MATD_EL(lu, p, 0), sizeof(TYPE) * lu->ncols);
			memcpy(&MATD_EL(lu, p, 0), &MATD_EL(lu, j, 0), sizeof(TYPE) * lu->ncols);
			memcpy(&MATD_EL(lu, j, 0), tmp, sizeof(TYPE) * lu->ncols);
#ifdef WIN32
			win_free(tmp);
			tmp = NULL;
#endif
			int k = piv[p];
			piv[p] = piv[j];
			piv[j] = k;
			pivsign = -pivsign;
		}

		double LUjj = MATD_EL(lu, j, j);

		// If our pivot is very small (which means the matrix is
		// singular or nearly singular), replace with a new pivot of the
		// right sign.
		if (fabs(LUjj) < MATD_EPS) {
			/*
						if (LUjj < 0)
							LUjj = -MATD_EPS;
						else
							LUjj = MATD_EPS;

						MATD_EL(lu, j, j) = LUjj;
			*/
			mlu->singular = 1;
		}

		if (j < lu->ncols && j < lu->nrows && LUjj != 0) {
			LUjj = 1.0 / LUjj;
			for (int i = j + 1; i < lu->nrows; i++)
				MATD_EL(lu, i, j) *= LUjj;
		}
	}

	mlu->lu = lu;
	mlu->piv = piv;
	mlu->pivsign = pivsign;

	return mlu;
}
matd_t* matd_identity(int dim)
{
	if (dim == 0)
		return matd_create_scalar(1);

	matd_t* m = matd_create(dim, dim);
	for (int i = 0; i < dim; i++)
		MATD_EL(m, i, i) = 1;

	return m;
}
// PLU = A
// Ax = B
// PLUx = B
// LUx = P'B
matd_t* matd_plu_solve(const matd_plu_t* mlu, const matd_t* b)
{
	matd_t* x = matd_copy(b);

	// permute right hand side
	for (int i = 0; i < mlu->lu->nrows; i++)
		memcpy(&MATD_EL(x, i, 0), &MATD_EL(b, mlu->piv[i], 0), sizeof(TYPE) * b->ncols);

	// solve Ly = b
	for (int k = 0; k < mlu->lu->nrows; k++) {
		for (int i = k + 1; i < mlu->lu->nrows; i++) {
			double LUik = -MATD_EL(mlu->lu, i, k);
			for (int t = 0; t < b->ncols; t++)
				MATD_EL(x, i, t) += MATD_EL(x, k, t) * LUik;
		}
	}

	// solve Ux = y
	for (int k = mlu->lu->ncols - 1; k >= 0; k--) {
		double LUkk = 1.0 / MATD_EL(mlu->lu, k, k);
		for (int t = 0; t < b->ncols; t++)
			MATD_EL(x, k, t) *= LUkk;

		for (int i = 0; i < k; i++) {
			double LUik = -MATD_EL(mlu->lu, i, k);
			for (int t = 0; t < b->ncols; t++)
				MATD_EL(x, i, t) += MATD_EL(x, k, t) * LUik;
		}
	}

	return x;
}
void matd_plu_destroy(matd_plu_t* mlu)
{
	matd_destroy(mlu->lu);
	free(mlu->piv);
	memset(mlu, 0, sizeof(matd_plu_t));
	free(mlu);
}

matd_t* matd_inverse(const matd_t* x)
{
	matd_t* m = NULL;

	assert(x != NULL);
	assert(x->nrows == x->ncols);

	if (matd_is_scalar(x)) {
		if (x->data[0] == 0)
			return NULL;

		return matd_create_scalar(1.0 / x->data[0]);
	}

	switch (x->nrows) {
	case 1: {
		double det = x->data[0];
		if (det == 0)
			return NULL;

		double invdet = 1.0 / det;

		m = matd_create(x->nrows, x->nrows);
		MATD_EL(m, 0, 0) = 1.0 * invdet;
		return m;
	}

	case 2: {
		double det = x->data[0] * x->data[3] - x->data[1] * x->data[2];
		if (det == 0)
			return NULL;

		double invdet = 1.0 / det;

		m = matd_create(x->nrows, x->nrows);
		MATD_EL(m, 0, 0) = MATD_EL(x, 1, 1) * invdet;
		MATD_EL(m, 0, 1) = -MATD_EL(x, 0, 1) * invdet;
		MATD_EL(m, 1, 0) = -MATD_EL(x, 1, 0) * invdet;
		MATD_EL(m, 1, 1) = MATD_EL(x, 0, 0) * invdet;
		return m;
	}

	default: {
		matd_plu_t* plu = matd_plu(x);

		matd_t* inv = NULL;
		if (!plu->singular) {
			matd_t* ident = matd_identity(x->nrows);
			inv = matd_plu_solve(plu, ident);
			matd_destroy(ident);
		}

		matd_plu_destroy(plu);

		return inv;
	}
	}

	return NULL; // unreachable
}
/**
 * Creates a double matrix with the Cholesky lower triangular matrix
 * of A. A must be symmetric, positive definite. It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 */
 //matd_t *matd_cholesky(const matd_t *A);

typedef struct
{
	int is_spd;
	matd_t* u;
} matd_chol_t;
// NOTE: The below implementation of Cholesky is different from the one
// used in NGV.
matd_chol_t* matd_chol(matd_t* A)
{
	assert(A->nrows == A->ncols);
	int N = A->nrows;

	// make upper right
	matd_t* U = matd_copy(A);

	// don't actually need to clear lower-left... we won't touch it.
/*    for (int i = 0; i < U->nrows; i++) {
	  for (int j = 0; j < i; j++) {
//            assert(MATD_EL(U, i, j) == MATD_EL(U, j, i));
MATD_EL(U, i, j) = 0;
}
}
*/
	int is_spd = 1; // (A->nrows == A->ncols);

	for (int i = 0; i < N; i++) {
		double d = MATD_EL(U, i, i);
		is_spd &= (d > 0);

		if (d < MATD_EPS)
			d = MATD_EPS;
		d = 1.0 / sqrt(d);

		for (int j = i; j < N; j++)
			MATD_EL(U, i, j) *= d;

		for (int j = i + 1; j < N; j++) {
			double s = MATD_EL(U, i, j);

			if (s == 0)
				continue;

			for (int k = j; k < N; k++) {
				MATD_EL(U, j, k) -= MATD_EL(U, i, k) * s;
			}
		}
	}

	matd_chol_t* chol = calloc(1, sizeof(matd_chol_t));
	chol->is_spd = is_spd;
	chol->u = U;
	return chol;
}
matd_t* matd_chol_solve(const matd_chol_t* chol, const matd_t* b)
{
	matd_t* u = chol->u;

	matd_t* x = matd_copy(b);

	// LUx = b

	// solve Ly = b ==> (U')y = b

	for (int i = 0; i < u->nrows; i++) {
		for (int j = 0; j < i; j++) {
			// b[i] -= L[i,j]*x[j]... replicated across columns of b
			//   ==> i.e., ==>
			// b[i,k] -= L[i,j]*x[j,k]
			for (int k = 0; k < b->ncols; k++) {
				MATD_EL(x, i, k) -= MATD_EL(u, j, i) * MATD_EL(x, j, k);
			}
		}
		// x[i] = b[i] / L[i,i]
		for (int k = 0; k < b->ncols; k++) {
			MATD_EL(x, i, k) /= MATD_EL(u, i, i);
		}
	}

	// solve Ux = y
	for (int k = u->ncols - 1; k >= 0; k--) {
		double LUkk = 1.0 / MATD_EL(u, k, k);
		for (int t = 0; t < b->ncols; t++)
			MATD_EL(x, k, t) *= LUkk;

		for (int i = 0; i < k; i++) {
			double LUik = -MATD_EL(u, i, k);
			for (int t = 0; t < b->ncols; t++)
				MATD_EL(x, i, t) += MATD_EL(x, k, t) * LUik;
		}
	}

	return x;
}
void matd_chol_destroy(matd_chol_t* chol)
{
	matd_destroy(chol->u);
	free(chol);
}
typedef struct
{
	matd_t* U;
	matd_t* S;
	matd_t* V;
} matd_svd_t;
// find the index of the off-diagonal element with the largest mag
static inline int max_idx(const matd_t* A, int row, int maxcol)
{
	int maxi = 0;
	double maxv = -1;

	for (int i = 0; i < maxcol; i++) {
		if (i == row)
			continue;
		double v = fabs(MATD_EL(A, row, i));
		if (v > maxv) {
			maxi = i;
			maxv = v;
		}
	}

	return maxi;
}
/** SVD 2x2.

	Computes singular values and vectors without squaring the input
	matrix. With double precision math, results are accurate to about
	1E-16.

	U = [ cos(theta) -sin(theta) ]
		[ sin(theta)  cos(theta) ]

	S = [ e  0 ]
		[ 0  f ]

	V = [ cos(phi)   -sin(phi) ]
		[ sin(phi)   cos(phi)  ]


	Our strategy is basically to analytically multiply everything out
	and then rearrange so that we can solve for theta, phi, e, and
	f. (Derivation by ebolson@umich.edu 5/2016)

   V' = [ CP  SP ]
		[ -SP CP ]

USV' = [ CT -ST ][  e*CP  e*SP ]
	   [ ST  CT ][ -f*SP  f*CP ]

	 = [e*CT*CP + f*ST*SP     e*CT*SP - f*ST*CP ]
	   [e*ST*CP - f*SP*CT     e*SP*ST + f*CP*CT ]

A00+A11 = e*CT*CP + f*ST*SP + e*SP*ST + f*CP*CT
		= e*(CP*CT + SP*ST) + f*(SP*ST + CP*CT)
		= (e+f)(CP*CT + SP*ST)
B0	    = (e+f)*cos(P-T)

A00-A11 = e*CT*CP + f*ST*SP - e*SP*ST - f*CP*CT
		= e*(CP*CT - SP*ST) - f*(-ST*SP + CP*CT)
		= (e-f)(CP*CT - SP*ST)
B1	    = (e-f)*cos(P+T)

A01+A10 = e*CT*SP - f*ST*CP + e*ST*CP - f*SP*CT
		= e(CT*SP + ST*CP) - f*(ST*CP + SP*CT)
		= (e-f)*(CT*SP + ST*CP)
B2	    = (e-f)*sin(P+T)

A01-A10 = e*CT*SP - f*ST*CP - e*ST*CP + f*SP*CT
	= e*(CT*SP - ST*CP) + f(SP*CT - ST*CP)
	= (e+f)*(CT*SP - ST*CP)
B3	= (e+f)*sin(P-T)

B0 = (e+f)*cos(P-T)
B1 = (e-f)*cos(P+T)
B2 = (e-f)*sin(P+T)
B3 = (e+f)*sin(P-T)

B3/B0 = tan(P-T)

B2/B1 = tan(P+T)
 **/
void svd22(const double A[4], double U[4], double S[2], double V[4])
{
	double A00 = A[0];
	double A01 = A[1];
	double A10 = A[2];
	double A11 = A[3];

	double B0 = A00 + A11;
	double B1 = A00 - A11;
	double B2 = A01 + A10;
	double B3 = A01 - A10;

	double PminusT = atan2(B3, B0);
	double PplusT = atan2(B2, B1);

	double P = (PminusT + PplusT) / 2;
	double T = (-PminusT + PplusT) / 2;

	double CP = cos(P), SP = sin(P);
	double CT = cos(T), ST = sin(T);

	U[0] = CT;
	U[1] = -ST;
	U[2] = ST;
	U[3] = CT;

	V[0] = CP;
	V[1] = -SP;
	V[2] = SP;
	V[3] = CP;

	// C0 = e+f. There are two ways to compute C0; we pick the one
	// that is better conditioned.
	double CPmT = cos(P - T), SPmT = sin(P - T);
	double C0 = 0;
	if (fabs(CPmT) > fabs(SPmT))
		C0 = B0 / CPmT;
	else
		C0 = B3 / SPmT;

	// C1 = e-f. There are two ways to compute C1; we pick the one
	// that is better conditioned.
	double CPpT = cos(P + T), SPpT = sin(P + T);
	double C1 = 0;
	if (fabs(CPpT) > fabs(SPpT))
		C1 = B1 / CPpT;
	else
		C1 = B2 / SPpT;

	// e and f are the singular values
	double e = (C0 + C1) / 2;
	double f = (C0 - C1) / 2;

	if (e < 0) {
		e = -e;
		U[0] = -U[0];
		U[2] = -U[2];
	}

	if (f < 0) {
		f = -f;
		U[1] = -U[1];
		U[3] = -U[3];
	}

	// sort singular values.
	if (e > f) {
		// already in big-to-small order.
		S[0] = e;
		S[1] = f;
	}
	else {
		// Curiously, this code never seems to get invoked.  Why is it
		// that S[0] always ends up the dominant vector?  However,
		// this code has been tested (flipping the logic forces us to
		// sort the singular values in ascending order).
		//
		// P = [ 0 1 ; 1 0 ]
		// USV' = (UP)(PSP)(PV')
		//      = (UP)(PSP)(VP)'
		//      = (UP)(PSP)(P'V')'
		S[0] = f;
		S[1] = e;

		// exchange columns of U and V
		double tmp[2];
		tmp[0] = U[0];
		tmp[1] = U[2];
		U[0] = U[1];
		U[2] = U[3];
		U[1] = tmp[0];
		U[3] = tmp[1];

		tmp[0] = V[0];
		tmp[1] = V[2];
		V[0] = V[1];
		V[2] = V[3];
		V[1] = tmp[0];
		V[3] = tmp[1];
	}

	/*
	double SM[4] = { S[0], 0, 0, S[1] };

	doubles_print_mat(U, 2, 2, "%20.10g");
	doubles_print_mat(SM, 2, 2, "%20.10g");
	doubles_print_mat(V, 2, 2, "%20.10g");
	printf("A:\n");
	doubles_print_mat(A, 2, 2, "%20.10g");

	double SVt[4];
	doubles_mat_ABt(SM, 2, 2, V, 2, 2, SVt, 2, 2);
	double USVt[4];
	doubles_mat_AB(U, 2, 2, SVt, 2, 2, USVt, 2, 2);

	printf("USVt\n");
	doubles_print_mat(USVt, 2, 2, "%20.10g");

	double diff[4];
	for (int i = 0; i < 4; i++)
		diff[i] = A[i] - USVt[i];

	printf("diff\n");
	doubles_print_mat(diff, 2, 2, "%20.10g");

	*/

}

#define MATD_SVD_NO_WARNINGS 1
// for the matrix [a b; b d]
void svd_sym_singular_values(double A00, double A01, double A11,
	double* Lmin, double* Lmax)
{
	double A10 = A01;

	double B0 = A00 + A11;
	double B1 = A00 - A11;
	double B2 = A01 + A10;
	double B3 = A01 - A10;

	double PminusT = atan2(B3, B0);
	double PplusT = atan2(B2, B1);

	double P = (PminusT + PplusT) / 2;
	double T = (-PminusT + PplusT) / 2;

	// C0 = e+f. There are two ways to compute C0; we pick the one
	// that is better conditioned.
	double CPmT = cos(P - T), SPmT = sin(P - T);
	double C0 = 0;
	if (fabs(CPmT) > fabs(SPmT))
		C0 = B0 / CPmT;
	else
		C0 = B3 / SPmT;

	// C1 = e-f. There are two ways to compute C1; we pick the one
	// that is better conditioned.
	double CPpT = cos(P + T), SPpT = sin(P + T);
	double C1 = 0;
	if (fabs(CPpT) > fabs(SPpT))
		C1 = B1 / CPpT;
	else
		C1 = B2 / SPpT;

	// e and f are the singular values
	double e = (C0 + C1) / 2;
	double f = (C0 - C1) / 2;

	*Lmin = fmin(e, f);
	*Lmax = fmax(e, f);
}
matd_t* matd_transpose(const matd_t* a)
{
	assert(a != NULL);

	if (matd_is_scalar(a))
		return matd_create_scalar(a->data[0]);

	matd_t* m = matd_create(a->ncols, a->nrows);

	for (int i = 0; i < a->nrows; i++) {
		for (int j = 0; j < a->ncols; j++) {
			MATD_EL(m, j, i) = MATD_EL(a, i, j);
		}
	}
	return m;
}
matd_t* matd_scale(const matd_t* a, double s)
{
	assert(a != NULL);

	if (matd_is_scalar(a))
		return matd_create_scalar(a->data[0] * s);

	matd_t* m = matd_create(a->nrows, a->ncols);

	for (int i = 0; i < m->nrows; i++) {
		for (int j = 0; j < m->ncols; j++) {
			MATD_EL(m, i, j) = s * MATD_EL(a, i, j);
		}
	}

	return m;
}
matd_t* matd_multiply(const matd_t* a, const matd_t* b)
{
	assert(a != NULL);
	assert(b != NULL);

	if (matd_is_scalar(a))
		return matd_scale(b, a->data[0]);
	if (matd_is_scalar(b))
		return matd_scale(a, b->data[0]);

	assert(a->ncols == b->nrows);
	matd_t* m = matd_create(a->nrows, b->ncols);

	for (int i = 0; i < m->nrows; i++) {
		for (int j = 0; j < m->ncols; j++) {
			TYPE acc = 0;
			for (int k = 0; k < a->ncols; k++) {
				acc += MATD_EL(a, i, k) * MATD_EL(b, k, j);
			}
			MATD_EL(m, i, j) = acc;
		}
	}

	return m;
}
matd_t* matd_add(const matd_t* a, const matd_t* b)
{
	assert(a != NULL);
	assert(b != NULL);
	assert(a->nrows == b->nrows);
	assert(a->ncols == b->ncols);

	if (matd_is_scalar(a))
		return matd_create_scalar(a->data[0] + b->data[0]);

	matd_t* m = matd_create(a->nrows, a->ncols);

	for (int i = 0; i < m->nrows; i++) {
		for (int j = 0; j < m->ncols; j++) {
			MATD_EL(m, i, j) = MATD_EL(a, i, j) + MATD_EL(b, i, j);
		}
	}

	return m;
}
matd_t* matd_subtract(const matd_t* a, const matd_t* b)
{
	assert(a != NULL);
	assert(b != NULL);
	assert(a->nrows == b->nrows);
	assert(a->ncols == b->ncols);

	if (matd_is_scalar(a))
		return matd_create_scalar(a->data[0] - b->data[0]);

	matd_t* m = matd_create(a->nrows, a->ncols);

	for (int i = 0; i < m->nrows; i++) {
		for (int j = 0; j < m->ncols; j++) {
			MATD_EL(m, i, j) = MATD_EL(a, i, j) - MATD_EL(b, i, j);
		}
	}

	return m;
}
// TODO Optimization: Some operations we could perform in-place,
// saving some memory allocation work. E.g., ADD, SUBTRACT. Just need
// to make sure that we don't do an in-place modification on a matrix
// that was an input argument!

// handle right-associative operators, greedily consuming them. These
// include transpose and inverse. This is called by the main recursion
// method.
static inline matd_t* matd_op_gobble_right(const char* expr, int* pos, matd_t* acc, matd_t** garb, int* garbpos)
{
	while (expr[*pos] != 0) {

		switch (expr[*pos]) {

		case '\'': {
			assert(acc != NULL); // either a syntax error or a math op failed, producing null
			matd_t* res = matd_transpose(acc);
			garb[*garbpos] = res;
			(*garbpos)++;
			acc = res;

			(*pos)++;
			break;
		}

				   // handle inverse ^-1. No other exponents are allowed.
		case '^': {
			assert(acc != NULL);
			assert(expr[*pos + 1] == '-');
			assert(expr[*pos + 2] == '1');

			matd_t* res = matd_inverse(acc);
			garb[*garbpos] = res;
			(*garbpos)++;
			acc = res;

			(*pos) += 3;
			break;
		}

		default:
			return acc;
		}
	}

	return acc;
}
#include "stdio.h"
// @garb, garbpos  A list of every matrix allocated during evaluation... used to assist cleanup.
// @oneterm: we should return at the end of this term (i.e., stop at a PLUS, MINUS, LPAREN).
static matd_t* matd_op_recurse(const char* expr, int* pos, matd_t* acc, matd_t** args, int* argpos,
	matd_t** garb, int* garbpos, int oneterm)
{
	while (expr[*pos] != 0) {

		switch (expr[*pos]) {

		case '(': {
			if (oneterm && acc != NULL)
				return acc;
			(*pos)++;
			matd_t* rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 0);
			rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

			if (acc == NULL) {
				acc = rhs;
			}
			else {
				matd_t* res = matd_multiply(acc, rhs);
				garb[*garbpos] = res;
				(*garbpos)++;
				acc = res;
			}

			break;
		}

		case ')': {
			if (oneterm)
				return acc;

			(*pos)++;
			return acc;
		}

		case '*': {
			(*pos)++;

			matd_t* rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
			rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

			if (acc == NULL) {
				acc = rhs;
			}
			else {
				matd_t* res = matd_multiply(acc, rhs);
				garb[*garbpos] = res;
				(*garbpos)++;
				acc = res;
			}

			break;
		}

		case 'F': {
			matd_t* rhs = args[*argpos];
			garb[*garbpos] = rhs;
			(*garbpos)++;

			(*pos)++;
			(*argpos)++;

			rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

			if (acc == NULL) {
				acc = rhs;
			}
			else {
				matd_t* res = matd_multiply(acc, rhs);
				garb[*garbpos] = res;
				(*garbpos)++;
				acc = res;
			}

			break;
		}

		case 'M': {
			matd_t* rhs = args[*argpos];

			(*pos)++;
			(*argpos)++;

			rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

			if (acc == NULL) {
				acc = rhs;
			}
			else {
				matd_t* res = matd_multiply(acc, rhs);
				garb[*garbpos] = res;
				(*garbpos)++;
				acc = res;
			}

			break;
		}

				  /*
					case 'D': {
					int rows = expr[*pos+1]-'0';
					int cols = expr[*pos+2]-'0';

					matd_t *rhs = matd_create(rows, cols);

					break;
					}
				  */
				  // a constant (SCALAR) defined inline. Treat just like M, creating a matd_t on the fly.
  //            case '0':
  //            case '1':
  //            case '2':
  //            case '3':
  //            case '4':
  //            case '5':
  //            case '6':
  //            case '7':
  //            case '8':
  //            case '9':
  //            case '.': {
  //                const char *start = &expr[*pos];
  //                char *end;
  //                double s = strtod(start, &end);
  //                (*pos) += (end - start);
  //                matd_t *rhs = matd_create_scalar(s);
  //                garb[*garbpos] = rhs;
  //                (*garbpos)++;

  //                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

  //                if (acc == NULL) {
  //                    acc = rhs;
  //                } else {
  //                    matd_t *res = matd_multiply(acc, rhs);
  //                    garb[*garbpos] = res;
  //                    (*garbpos)++;
  //                    acc = res;
  //                }

  //                break;
  //            }

		case '+': {
			if (oneterm && acc != NULL)
				return acc;

			// don't support unary plus
			assert(acc != NULL);
			(*pos)++;
			matd_t* rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
			rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

			matd_t* res = matd_add(acc, rhs);

			garb[*garbpos] = res;
			(*garbpos)++;
			acc = res;
			break;
		}

		case '-': {
			if (oneterm && acc != NULL)
				return acc;

			if (acc == NULL) {
				// unary minus
				(*pos)++;
				matd_t* rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
				rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

				matd_t* res = matd_scale(rhs, -1);
				garb[*garbpos] = res;
				(*garbpos)++;
				acc = res;
			}
			else {
				// subtract
				(*pos)++;
				matd_t* rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
				rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

				matd_t* res = matd_subtract(acc, rhs);
				garb[*garbpos] = res;
				(*garbpos)++;
				acc = res;
			}
			break;
		}

		case ' ': {
			// nothing to do. spaces are meaningless.
			(*pos)++;
			break;
		}

		default: {
			fprintf(stderr, "matd_op(): Unknown character: '%c'\n", expr[*pos]);
			assert(expr[*pos] != expr[*pos]);
		}
		}
	}
	return acc;
}
#include <stdarg.h>
// always returns a new matrix.
matd_t* matd_op(const char* expr, ...)
{
	int nargs = 0;
	int exprlen = 0;

	assert(expr != NULL);

	for (const char* p = expr; *p != 0; p++) {
		if (*p == 'M' || *p == 'F')
			nargs++;
		exprlen++;
	}

	assert(nargs > 0);

	if (!exprlen) // expr = ""
		return NULL;

	va_list ap;
	va_start(ap, expr);
#ifdef WIN32
	matd_t** args = win_malloc(nargs * sizeof(matd_t*));
#else
	matd_t* args[nargs];
#endif
	for (int i = 0; i < nargs; i++) {
		args[i] = va_arg(ap, matd_t*);
		// XXX: sanity check argument; emit warning/error if args[i]
		// doesn't look like a matd_t*.
	}

	va_end(ap);

	int pos = 0;
	int argpos = 0;
	int garbpos = 0;
#ifdef WIN32
	matd_t** garb = win_malloc(2 * exprlen * sizeof(matd_t*));
#else
	matd_t* garb[2 * exprlen]; // can't create more than 2 new result per character
							 // one result, and possibly one argument to free
#endif

	matd_t* res = matd_op_recurse(expr, &pos, NULL, args, &argpos, garb, &garbpos, 0);

	// 'res' may need to be freed as part of garbage collection (i.e. expr = "F")
	matd_t* res_copy = (res ? matd_copy(res) : NULL);

	for (int i = 0; i < garbpos; i++) {
		matd_destroy(garb[i]);
	}
#ifdef WIN32 
	win_free(args);
	args = NULL;
	win_free(garb);
	args = NULL;
#endif
	return res_copy;
}
// Computes an SVD for square or tall matrices. This code doesn't work
// for wide matrices, because the bidiagonalization results in one
// non-zero element too far to the right for us to rotate away.
//
// Caller is responsible for destroying U, S, and V.
static matd_svd_t matd_svd_tall(matd_t* A, int flags)
{
	matd_t* B = matd_copy(A);

	// Apply householder reflections on each side to reduce A to
	// bidiagonal form. Specifically:
	//
	// A = LS*B*RS'
	//
	// Where B is bidiagonal, and LS/RS are unitary.
	//
	// Why are we doing this? Some sort of transformation is necessary
	// to reduce the matrix's nz elements to a square region. QR could
	// work too. We need nzs confined to a square region so that the
	// subsequent iterative process, which is based on rotations, can
	// work. (To zero out a term at (i,j), our rotations will also
	// affect (j,i).
	//
	// We prefer bidiagonalization over QR because it gets us "closer"
	// to the SVD, which should mean fewer iterations.

	// LS: cumulative left-handed transformations
	matd_t* LS = matd_identity(A->nrows);

	// RS: cumulative right-handed transformations.
	matd_t* RS = matd_identity(A->ncols);

	for (int hhidx = 0; hhidx < A->nrows; hhidx++) {

		if (hhidx < A->ncols) {
			// We construct the normal of the reflection plane: let u
			// be the vector to reflect, x =[ M 0 0 0 ] the target
			// location for u (u') after reflection (with M = ||u||).
			//
			// The normal vector is then n = (u - x), but since we
			// could equally have the target location be x = [-M 0 0 0
			// ], we could use n = (u + x).
			//
			// We then normalize n. To ensure a reasonable magnitude,
			// we select the sign of M so as to maximize the magnitude
			// of the first element of (x +/- M). (Otherwise, we could
			// end up with a divide-by-zero if u[0] and M cancel.)
			//
			// The householder reflection matrix is then H=(I - nn'), and
			// u' = Hu.
			//
			//
			int vlen = A->nrows - hhidx;
#ifdef WIN32
			double* v = win_malloc(sizeof(double) * vlen);
#else
			double v[vlen];
#endif

			double mag2 = 0;
			for (int i = 0; i < vlen; i++) {
				v[i] = MATD_EL(B, hhidx + i, hhidx);
				mag2 += v[i] * v[i];
			}

			double oldv0 = v[0];
			if (oldv0 < 0)
				v[0] -= sqrt(mag2);
			else
				v[0] += sqrt(mag2);

			mag2 += -oldv0 * oldv0 + v[0] * v[0];

			// normalize v
			double mag = sqrt(mag2);

			// this case arises with matrices of all zeros, for example.
			if (mag == 0)
				continue;

			for (int i = 0; i < vlen; i++)
				v[i] /= mag;

			// Q = I - 2vv'
			//matd_t *Q = matd_identity(A->nrows);
			//for (int i = 0; i < vlen; i++)
			//  for (int j = 0; j < vlen; j++)
			//    MATD_EL(Q, i+hhidx, j+hhidx) -= 2*v[i]*v[j];


			// LS = matd_op("F*M", LS, Q);
			// Implementation: take each row of LS, compute dot product with n,
			// subtract n (scaled by dot product) from it.
			for (int i = 0; i < LS->nrows; i++) {
				double dot = 0;
				for (int j = 0; j < vlen; j++)
					dot += MATD_EL(LS, i, hhidx + j) * v[j];
				for (int j = 0; j < vlen; j++)
					MATD_EL(LS, i, hhidx + j) -= 2 * dot * v[j];
			}

			//  B = matd_op("M*F", Q, B); // should be Q', but Q is symmetric.
			for (int i = 0; i < B->ncols; i++) {
				double dot = 0;
				for (int j = 0; j < vlen; j++)
					dot += MATD_EL(B, hhidx + j, i) * v[j];
				for (int j = 0; j < vlen; j++)
					MATD_EL(B, hhidx + j, i) -= 2 * dot * v[j];
			}
#ifdef WIN32
			win_free(v);
#endif
		}

		if (hhidx + 2 < A->ncols) {
			int vlen = A->ncols - hhidx - 1;

#ifdef WIN32
			double* v = win_malloc(sizeof(double) * vlen);
#else
			double v[vlen];
#endif

			double mag2 = 0;
			for (int i = 0; i < vlen; i++) {
				v[i] = MATD_EL(B, hhidx, hhidx + i + 1);
				mag2 += v[i] * v[i];
			}

			double oldv0 = v[0];
			if (oldv0 < 0)
				v[0] -= sqrt(mag2);
			else
				v[0] += sqrt(mag2);

			mag2 += -oldv0 * oldv0 + v[0] * v[0];

			// compute magnitude of ([1 0 0..]+v)
			double mag = sqrt(mag2);

			// this case can occur when the vectors are already perpendicular
			if (mag == 0)
				continue;

			for (int i = 0; i < vlen; i++)
				v[i] /= mag;

			// TODO: optimize these multiplications
			// matd_t *Q = matd_identity(A->ncols);
			//  for (int i = 0; i < vlen; i++)
			//    for (int j = 0; j < vlen; j++)
			//       MATD_EL(Q, i+1+hhidx, j+1+hhidx) -= 2*v[i]*v[j];

			//  RS = matd_op("F*M", RS, Q);
			for (int i = 0; i < RS->nrows; i++) {
				double dot = 0;
				for (int j = 0; j < vlen; j++)
					dot += MATD_EL(RS, i, hhidx + 1 + j) * v[j];
				for (int j = 0; j < vlen; j++)
					MATD_EL(RS, i, hhidx + 1 + j) -= 2 * dot * v[j];
			}

			//   B = matd_op("F*M", B, Q); // should be Q', but Q is symmetric.
			for (int i = 0; i < B->nrows; i++) {
				double dot = 0;
				for (int j = 0; j < vlen; j++)
					dot += MATD_EL(B, i, hhidx + 1 + j) * v[j];
				for (int j = 0; j < vlen; j++)
					MATD_EL(B, i, hhidx + 1 + j) -= 2 * dot * v[j];
			}
#ifdef WIN32
			win_free(v);
#endif
		}
	}

	// maxiters used to be smaller to prevent us from looping forever,
	// but this doesn't seem to happen any more with our more stable
	// svd22 implementation.
	int maxiters = 1UL << 5; // 1UL << 30;
	assert(maxiters > 0); // reassure clang
	int iter;

	double maxv; // maximum non-zero value being reduced this iteration

	double tol = 1E-5; // 1E-10;

	// which method will we use to find the largest off-diagonal
	// element of B?
	const int find_max_method = 1; //(B->ncols < 6) ? 2 : 1;

	// for each of the first B->ncols rows, which index has the
	// maximum absolute value? (used by method 1)
#ifdef WIN32
	int* maxrowidx = win_malloc(B->ncols * sizeof(int));
#else
	int maxrowidx[B->ncols];
#endif
	int lastmaxi, lastmaxj;

	if (find_max_method == 1) {
		for (int i = 2; i < B->ncols; i++)
			maxrowidx[i] = max_idx(B, i, B->ncols);

		// note that we started the array at 2. That's because by setting
		// these values below, we'll recompute first two entries on the
		// first iteration!
		lastmaxi = 0, lastmaxj = 1;
	}

	for (iter = 0; iter < maxiters; iter++) {

		// No diagonalization required for 0x0 and 1x1 matrices.
		if (B->ncols < 2)
			break;

		// find the largest off-diagonal element of B, and put its
		// coordinates in maxi, maxj.
		int maxi, maxj;

		if (find_max_method == 1) {
			// method 1 is the "smarter" method which does at least
			// 4*ncols work. More work might be needed (up to
			// ncols*ncols), depending on data. Thus, this might be a
			// bit slower than the default method for very small
			// matrices.
			maxi = -1;
			maxv = -1;

			// every iteration, we must deal with the fact that rows
			// and columns lastmaxi and lastmaxj have been
			// modified. Update maxrowidx accordingly.

			// now, EVERY row also had columns lastmaxi and lastmaxj modified.
			for (int rowi = 0; rowi < B->ncols; rowi++) {

				// the magnitude of the largest off-diagonal element
				// in this row.
				double thismaxv;

				// row 'lastmaxi' and 'lastmaxj' have been completely
				// changed. compute from scratch.
				if (rowi == lastmaxi || rowi == lastmaxj) {
					maxrowidx[rowi] = max_idx(B, rowi, B->ncols);
					thismaxv = fabs(MATD_EL(B, rowi, maxrowidx[rowi]));
					goto endrowi;
				}

				// our maximum entry was just modified. We don't know
				// if it went up or down, and so we don't know if it
				// is still the maximum. We have to update from
				// scratch.
				if (maxrowidx[rowi] == lastmaxi || maxrowidx[rowi] == lastmaxj) {
					maxrowidx[rowi] = max_idx(B, rowi, B->ncols);
					thismaxv = fabs(MATD_EL(B, rowi, maxrowidx[rowi]));
					goto endrowi;
				}

				// This row is unchanged, except for columns
				// 'lastmaxi' and 'lastmaxj', and those columns were
				// not previously the largest entry...  just check to
				// see if they are now the maximum entry in their
				// row. (Remembering to consider off-diagonal entries
				// only!)
				thismaxv = fabs(MATD_EL(B, rowi, maxrowidx[rowi]));

				// check column lastmaxi. Is it now the maximum?
				if (lastmaxi != rowi) {
					double v = fabs(MATD_EL(B, rowi, lastmaxi));
					if (v > thismaxv) {
						thismaxv = v;
						maxrowidx[rowi] = lastmaxi;
					}
				}

				// check column lastmaxj
				if (lastmaxj != rowi) {
					double v = fabs(MATD_EL(B, rowi, lastmaxj));
					if (v > thismaxv) {
						thismaxv = v;
						maxrowidx[rowi] = lastmaxj;
					}
				}

				// does this row have the largest value we've seen so far?
			endrowi:
				if (thismaxv > maxv) {
					maxv = thismaxv;
					maxi = rowi;
				}
			}

			assert(maxi >= 0);
			maxj = maxrowidx[maxi];

			// save these for the next iteration.
			lastmaxi = maxi;
			lastmaxj = maxj;

			if (maxv < tol)
				break;

		}
		else if (find_max_method == 2) {
			// brute-force (reference) version.
			maxv = -1;

			// only search top "square" portion
			for (int i = 0; i < B->ncols; i++) {
				for (int j = 0; j < B->ncols; j++) {
					if (i == j)
						continue;

					double v = fabs(MATD_EL(B, i, j));

					if (v > maxv) {
						maxi = i;
						maxj = j;
						maxv = v;
					}
				}
			}

			// termination condition.
			if (maxv < tol)
				break;
		}
		else {
			assert(0);
		}

		//        printf(">>> %5d %3d, %3d %15g\n", maxi, maxj, iter, maxv);

				// Now, solve the 2x2 SVD problem for the matrix
				// [ A0 A1 ]
				// [ A2 A3 ]
		double A0 = MATD_EL(B, maxi, maxi);
		double A1 = MATD_EL(B, maxi, maxj);
		double A2 = MATD_EL(B, maxj, maxi);
		double A3 = MATD_EL(B, maxj, maxj);

		if (1) {
			double AQ[4];
			AQ[0] = A0;
			AQ[1] = A1;
			AQ[2] = A2;
			AQ[3] = A3;

			double U[4], S[2], V[4];
			svd22(AQ, U, S, V);

			/*  Reference (slow) implementation...

						// LS = LS * ROT(theta) = LS * QL
						matd_t *QL = matd_identity(A->nrows);
						MATD_EL(QL, maxi, maxi) = U[0];
						MATD_EL(QL, maxi, maxj) = U[1];
						MATD_EL(QL, maxj, maxi) = U[2];
						MATD_EL(QL, maxj, maxj) = U[3];

						matd_t *QR = matd_identity(A->ncols);
						MATD_EL(QR, maxi, maxi) = V[0];
						MATD_EL(QR, maxi, maxj) = V[1];
						MATD_EL(QR, maxj, maxi) = V[2];
						MATD_EL(QR, maxj, maxj) = V[3];

						LS = matd_op("F*M", LS, QL);
						RS = matd_op("F*M", RS, QR); // remember we'll transpose RS.
						B = matd_op("M'*F*M", QL, B, QR);

						matd_destroy(QL);
						matd_destroy(QR);
			*/

			//  LS = matd_op("F*M", LS, QL);
			for (int i = 0; i < LS->nrows; i++) {
				double vi = MATD_EL(LS, i, maxi);
				double vj = MATD_EL(LS, i, maxj);

				MATD_EL(LS, i, maxi) = U[0] * vi + U[2] * vj;
				MATD_EL(LS, i, maxj) = U[1] * vi + U[3] * vj;
			}

			//  RS = matd_op("F*M", RS, QR); // remember we'll transpose RS.
			for (int i = 0; i < RS->nrows; i++) {
				double vi = MATD_EL(RS, i, maxi);
				double vj = MATD_EL(RS, i, maxj);

				MATD_EL(RS, i, maxi) = V[0] * vi + V[2] * vj;
				MATD_EL(RS, i, maxj) = V[1] * vi + V[3] * vj;
			}

			// B = matd_op("M'*F*M", QL, B, QR);
			// The QL matrix mixes rows of B.
			for (int i = 0; i < B->ncols; i++) {
				double vi = MATD_EL(B, maxi, i);
				double vj = MATD_EL(B, maxj, i);

				MATD_EL(B, maxi, i) = U[0] * vi + U[2] * vj;
				MATD_EL(B, maxj, i) = U[1] * vi + U[3] * vj;
			}

			// The QR matrix mixes columns of B.
			for (int i = 0; i < B->nrows; i++) {
				double vi = MATD_EL(B, i, maxi);
				double vj = MATD_EL(B, i, maxj);

				MATD_EL(B, i, maxi) = V[0] * vi + V[2] * vj;
				MATD_EL(B, i, maxj) = V[1] * vi + V[3] * vj;
			}
		}
	}
#ifdef WIN32
	win_free(maxrowidx);
#endif
	if (!(flags & MATD_SVD_NO_WARNINGS) && iter == maxiters) {
		printf("WARNING: maximum iters (maximum = %d, matrix %d x %d, max=%.15f)\n",
			iter, A->nrows, A->ncols, maxv);

		//        matd_print(A, "%15f");
	}

	// them all positive by flipping the corresponding columns of
	// U/LS.
#ifdef WIN32
	int* idxs = win_malloc(sizeof(int) * A->ncols);
	double* vals = win_malloc(sizeof(double) * A->ncols);
#else
	int idxs[A->ncols];
	double vals[A->ncols];
#endif
	for (int i = 0; i < A->ncols; i++) {
		idxs[i] = i;
		vals[i] = MATD_EL(B, i, i);
	}

	// A bubble sort. Seriously.
	int changed;
	do {
		changed = 0;

		for (int i = 0; i + 1 < A->ncols; i++) {
			if (fabs(vals[i + 1]) > fabs(vals[i])) {
				int tmpi = idxs[i];
				idxs[i] = idxs[i + 1];
				idxs[i + 1] = tmpi;

				double tmpv = vals[i];
				vals[i] = vals[i + 1];
				vals[i + 1] = tmpv;

				changed = 1;
			}
		}
	} while (changed);

	matd_t* LP = matd_identity(A->nrows);
	matd_t* RP = matd_identity(A->ncols);

	for (int i = 0; i < A->ncols; i++) {
		MATD_EL(LP, idxs[i], idxs[i]) = 0; // undo the identity above
		MATD_EL(RP, idxs[i], idxs[i]) = 0;

		MATD_EL(LP, idxs[i], i) = vals[i] < 0 ? -1 : 1;
		MATD_EL(RP, idxs[i], i) = 1; //vals[i] < 0 ? -1 : 1;
	}

	// we've factored:
	// LP*(something)*RP'

	// solve for (something)
	B = matd_op("M'*F*M", LP, B, RP);

	// update LS and RS, remembering that RS will be transposed.
	LS = matd_op("F*M", LS, LP);
	RS = matd_op("F*M", RS, RP);

	matd_destroy(LP);
	matd_destroy(RP);

	matd_svd_t res;
	memset(&res, 0, sizeof(res));

	// make B exactly diagonal

	for (int i = 0; i < B->nrows; i++) {
		for (int j = 0; j < B->ncols; j++) {
			if (i != j)
				MATD_EL(B, i, j) = 0;
		}
	}
#ifdef WIN32
	win_free(idxs);
	win_free(vals);
#endif
	res.U = LS;
	res.S = B;
	res.V = RS;

	return res;
}
matd_svd_t matd_svd_flags(matd_t* A, int flags)
{
	matd_svd_t res;

	if (A->ncols <= A->nrows) {
		res = matd_svd_tall(A, flags);
	}
	else {
		matd_t* At = matd_transpose(A);

		// A =U  S  V'
		// A'=V  S' U'

		matd_svd_t tmp = matd_svd_tall(At, flags);

		memset(&res, 0, sizeof(res));
		res.U = tmp.V; //matd_transpose(tmp.V);
		res.S = matd_transpose(tmp.S);
		res.V = tmp.U; //matd_transpose(tmp.U);

		matd_destroy(tmp.S);
		matd_destroy(At);
	}

	/*
	  matd_t *check = matd_op("M*M*M'-M", res.U, res.S, res.V, A);
	  double maxerr = 0;

	  for (int i = 0; i < check->nrows; i++)
	  for (int j = 0; j < check->ncols; j++)
	  maxerr = fmax(maxerr, fabs(MATD_EL(check, i, j)));

	  matd_destroy(check);

	  if (maxerr > 1e-7) {
	  printf("bad maxerr: %15f\n", maxerr);
	  }

	  if (maxerr > 1e-5) {
	  printf("bad maxerr: %15f\n", maxerr);
	  matd_print(A, "%15f");
	  assert(0);
	  }

	*/
	return res;
}
#define HOMOGRAPHY_COMPUTE_FLAG_INVERSE 1
#define HOMOGRAPHY_COMPUTE_FLAG_SVD 0
// correspondences is a list of float[4]s, consisting of the points x
// and y concatenated. We will compute a homography such that y = Hx
matd_t * homography_compute(zarray_t * correspondences, int flags)
{
	// compute centroids of both sets of points (yields a better
	// conditioned information matrix)
	double x_cx = 0, x_cy = 0;
	double y_cx = 0, y_cy = 0;

	for (int i = 0; i < zarray_size(correspondences); i++) {
		float* c;
		zarray_get_volatile(correspondences, i, &c);

		x_cx += c[0];
		x_cy += c[1];
		y_cx += c[2];
		y_cy += c[3];
	}

	int sz = zarray_size(correspondences);
	x_cx /= sz;
	x_cy /= sz;
	y_cx /= sz;
	y_cy /= sz;

	// NB We don't normalize scale; it seems implausible that it could
	// possibly make any difference given the dynamic range of IEEE
	// doubles.

	matd_t* A = matd_create(9, 9);
	for (int i = 0; i < zarray_size(correspondences); i++) {
		float* c;
		zarray_get_volatile(correspondences, i, &c);

		// (below world is "x", and image is "y")
		double worldx = c[0] - x_cx;
		double worldy = c[1] - x_cy;
		double imagex = c[2] - y_cx;
		double imagey = c[3] - y_cy;

		double a03 = -worldx;
		double a04 = -worldy;
		double a05 = -1;
		double a06 = worldx * imagey;
		double a07 = worldy * imagey;
		double a08 = imagey;

		MATD_EL(A, 3, 3) += a03 * a03;
		MATD_EL(A, 3, 4) += a03 * a04;
		MATD_EL(A, 3, 5) += a03 * a05;
		MATD_EL(A, 3, 6) += a03 * a06;
		MATD_EL(A, 3, 7) += a03 * a07;
		MATD_EL(A, 3, 8) += a03 * a08;
		MATD_EL(A, 4, 4) += a04 * a04;
		MATD_EL(A, 4, 5) += a04 * a05;
		MATD_EL(A, 4, 6) += a04 * a06;
		MATD_EL(A, 4, 7) += a04 * a07;
		MATD_EL(A, 4, 8) += a04 * a08;
		MATD_EL(A, 5, 5) += a05 * a05;
		MATD_EL(A, 5, 6) += a05 * a06;
		MATD_EL(A, 5, 7) += a05 * a07;
		MATD_EL(A, 5, 8) += a05 * a08;
		MATD_EL(A, 6, 6) += a06 * a06;
		MATD_EL(A, 6, 7) += a06 * a07;
		MATD_EL(A, 6, 8) += a06 * a08;
		MATD_EL(A, 7, 7) += a07 * a07;
		MATD_EL(A, 7, 8) += a07 * a08;
		MATD_EL(A, 8, 8) += a08 * a08;

		double a10 = worldx;
		double a11 = worldy;
		double a12 = 1;
		double a16 = -worldx * imagex;
		double a17 = -worldy * imagex;
		double a18 = -imagex;

		MATD_EL(A, 0, 0) += a10 * a10;
		MATD_EL(A, 0, 1) += a10 * a11;
		MATD_EL(A, 0, 2) += a10 * a12;
		MATD_EL(A, 0, 6) += a10 * a16;
		MATD_EL(A, 0, 7) += a10 * a17;
		MATD_EL(A, 0, 8) += a10 * a18;
		MATD_EL(A, 1, 1) += a11 * a11;
		MATD_EL(A, 1, 2) += a11 * a12;
		MATD_EL(A, 1, 6) += a11 * a16;
		MATD_EL(A, 1, 7) += a11 * a17;
		MATD_EL(A, 1, 8) += a11 * a18;
		MATD_EL(A, 2, 2) += a12 * a12;
		MATD_EL(A, 2, 6) += a12 * a16;
		MATD_EL(A, 2, 7) += a12 * a17;
		MATD_EL(A, 2, 8) += a12 * a18;
		MATD_EL(A, 6, 6) += a16 * a16;
		MATD_EL(A, 6, 7) += a16 * a17;
		MATD_EL(A, 6, 8) += a16 * a18;
		MATD_EL(A, 7, 7) += a17 * a17;
		MATD_EL(A, 7, 8) += a17 * a18;
		MATD_EL(A, 8, 8) += a18 * a18;

		double a20 = -worldx * imagey;
		double a21 = -worldy * imagey;
		double a22 = -imagey;
		double a23 = worldx * imagex;
		double a24 = worldy * imagex;
		double a25 = imagex;

		MATD_EL(A, 0, 0) += a20 * a20;
		MATD_EL(A, 0, 1) += a20 * a21;
		MATD_EL(A, 0, 2) += a20 * a22;
		MATD_EL(A, 0, 3) += a20 * a23;
		MATD_EL(A, 0, 4) += a20 * a24;
		MATD_EL(A, 0, 5) += a20 * a25;
		MATD_EL(A, 1, 1) += a21 * a21;
		MATD_EL(A, 1, 2) += a21 * a22;
		MATD_EL(A, 1, 3) += a21 * a23;
		MATD_EL(A, 1, 4) += a21 * a24;
		MATD_EL(A, 1, 5) += a21 * a25;
		MATD_EL(A, 2, 2) += a22 * a22;
		MATD_EL(A, 2, 3) += a22 * a23;
		MATD_EL(A, 2, 4) += a22 * a24;
		MATD_EL(A, 2, 5) += a22 * a25;
		MATD_EL(A, 3, 3) += a23 * a23;
		MATD_EL(A, 3, 4) += a23 * a24;
		MATD_EL(A, 3, 5) += a23 * a25;
		MATD_EL(A, 4, 4) += a24 * a24;
		MATD_EL(A, 4, 5) += a24 * a25;
		MATD_EL(A, 5, 5) += a25 * a25;
	}

	// make symmetric
	for (int i = 0; i < 9; i++)
		for (int j = i + 1; j < 9; j++)
			MATD_EL(A, j, i) = MATD_EL(A, i, j);

	matd_t* H = matd_create(3, 3);

	if (flags & HOMOGRAPHY_COMPUTE_FLAG_INVERSE) {
		// compute singular vector by (carefully) inverting the rank-deficient matrix.

		if (1) {
			matd_t* Ainv = matd_inverse(A);
			double scale = 0;

			for (int i = 0; i < 9; i++)
				scale += sq(MATD_EL(Ainv, i, 0));
			scale = sqrt(scale);

			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					MATD_EL(H, i, j) = MATD_EL(Ainv, 3 * i + j, 0) / scale;

			matd_destroy(Ainv);
		}
		else {

			matd_t* b = matd_create_data(9, 1, (double[]) { 1, 0, 0, 0, 0, 0, 0, 0, 0 });
			matd_t* Ainv = NULL;

			if (0) {
				matd_plu_t* lu = matd_plu(A);
				Ainv = matd_plu_solve(lu, b);
				matd_plu_destroy(lu);
			}
			else {
				matd_chol_t* chol = matd_chol(A);
				Ainv = matd_chol_solve(chol, b);
				matd_chol_destroy(chol);
			}

			double scale = 0;

			for (int i = 0; i < 9; i++)
				scale += sq(MATD_EL(Ainv, i, 0));
			scale = sqrt(scale);

			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					MATD_EL(H, i, j) = MATD_EL(Ainv, 3 * i + j, 0) / scale;

			matd_destroy(b);
			matd_destroy(Ainv);
		}

	}
	else {
		// compute singular vector using SVD. A bit slower, but more accurate.
		matd_svd_t svd = matd_svd_flags(A, MATD_SVD_NO_WARNINGS);

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				MATD_EL(H, i, j) = MATD_EL(svd.U, 3 * i + j, 8);

		matd_destroy(svd.U);
		matd_destroy(svd.S);
		matd_destroy(svd.V);

	}

	matd_t* Tx = matd_identity(3);
	MATD_EL(Tx, 0, 2) = -x_cx;
	MATD_EL(Tx, 1, 2) = -x_cy;

	matd_t* Ty = matd_identity(3);
	MATD_EL(Ty, 0, 2) = y_cx;
	MATD_EL(Ty, 1, 2) = y_cy;

	matd_t* H2 = matd_op("M*M*M", Ty, H, Tx);

	matd_destroy(A);
	matd_destroy(Tx);
	matd_destroy(Ty);
	matd_destroy(H);

	return H2;
}
// returns non-zero if an error occurs (i.e., H has no inverse)
int quad_update_homographies(struct quad* quad)
{
	zarray_t* correspondences = zarray_create(sizeof(float[4]));

	for (int i = 0; i < 4; i++) {
		float corr[4];

		// At this stage of the pipeline, we have not attempted to decode the
		// quad into an oriented tag. Thus, just act as if the quad is facing
		// "up" with respect to our desired corners. We'll fix the rotation
		// later.
		// [-1, -1], [1, -1], [1, 1], [-1, 1]
		corr[0] = (i == 0 || i == 3) ? -1 : 1;
		corr[1] = (i == 0 || i == 1) ? -1 : 1;

		corr[2] = quad->p[i][0];
		corr[3] = quad->p[i][1];

		zarray_add(correspondences, &corr);
	}

	if (quad->H)
		matd_destroy(quad->H);
	if (quad->Hinv)
		matd_destroy(quad->Hinv);

	// XXX Tunable
	quad->H = homography_compute(correspondences, HOMOGRAPHY_COMPUTE_FLAG_SVD);
	quad->Hinv = matd_inverse(quad->H);
	zarray_destroy(correspondences);

	if (quad->H && quad->Hinv)
		return 0;

	return -1;
}
zarray_t* g2d_polygon_create_zeros(int sz)
{
	//  in MSC the sizeof(double) is 2 but the ARM's double is 1 instead, lead the memcpy error....
#ifdef WIN32 
	zarray_t* points = zarray_create(sizeof(float[2]));
#else
	zarray_t* points = zarray_create(sizeof(double[2]));
#endif

	double z[2] = { 0, 0 };

	for (int i = 0; i < sz; i++)
		zarray_add(points, z);

	return points;
}
typedef struct
{
	// Internal representation: a point that the line goes through (p) and
	// the direction of the line (u).
	double p[2];
	double u[2]; // always a unit vector
} g2d_line_t;
////////////////////////////////////////////////////////////////////
// Line Segments. line.p is always one endpoint; p1 is the other
// endpoint.
typedef struct
{
	g2d_line_t line;
	double p1[2];
} g2d_line_segment_t;
void g2d_line_init_from_points(g2d_line_t* line, const double p0[2], const double p1[2])
{
	line->p[0] = p0[0];
	line->p[1] = p0[1];
	line->u[0] = p1[0] - p0[0];
	line->u[1] = p1[1] - p0[1];
	double mag = sqrtf(sq(line->u[0]) + sq(line->u[1]));

	line->u[0] /= mag;
	line->u[1] /= mag;
}
void g2d_line_segment_init_from_points(g2d_line_segment_t* seg, const double p0[2], const double p1[2])
{
	g2d_line_init_from_points(&seg->line, p0, p1);
	seg->p1[0] = p1[0];
	seg->p1[1] = p1[1];
}
// Compute intersection of two line segments. If they intersect,
// result is stored in p and 1 is returned. Otherwise, zero is
// returned. p may be NULL.
int g2d_line_intersect_line(const g2d_line_t* linea, const g2d_line_t* lineb, double* p)
{
	// this implementation is many times faster than the original,
	// mostly due to avoiding a general-purpose LU decomposition in
	// Matrix.inverse().
	double m00, m01, m10, m11;
	double i00, i01;
	double b00, b10;

	m00 = linea->u[0];
	m01 = -lineb->u[0];
	m10 = linea->u[1];
	m11 = -lineb->u[1];

	// determinant of m
	double det = m00 * m11 - m01 * m10;

	// parallel lines?
	if (fabs(det) < 0.00000001)
		return 0;

	// inverse of m
	i00 = m11 / det;
	i01 = -m01 / det;

	b00 = lineb->p[0] - linea->p[0];
	b10 = lineb->p[1] - linea->p[1];

	double x00; //, x10;
	x00 = i00 * b00 + i01 * b10;

	if (p != NULL) {
		p[0] = linea->u[0] * x00 + linea->p[0];
		p[1] = linea->u[1] * x00 + linea->p[1];
	}

	return 1;
}
double g2d_line_get_coordinate(const g2d_line_t* line, const double q[2])
{
	return (q[0] - line->p[0]) * line->u[0] + (q[1] - line->p[1]) * line->u[1];
}
// Compute intersection of two line segments. If they intersect,
// result is stored in p and 1 is returned. Otherwise, zero is
// returned. p may be NULL.
int g2d_line_segment_intersect_segment(const g2d_line_segment_t* sega, const g2d_line_segment_t* segb, double* p)
{
	double tmp[2];

	if (!g2d_line_intersect_line(&sega->line, &segb->line, tmp))
		return 0;

	double a = g2d_line_get_coordinate(&sega->line, sega->line.p);
	double b = g2d_line_get_coordinate(&sega->line, sega->p1);
	double c = g2d_line_get_coordinate(&sega->line, tmp);

	// does intersection lie on the first line?
	if ((c < a && c < b) || (c > a && c > b))
		return 0;

	a = g2d_line_get_coordinate(&segb->line, segb->line.p);
	b = g2d_line_get_coordinate(&segb->line, segb->p1);
	c = g2d_line_get_coordinate(&segb->line, tmp);

	// does intersection lie on second line?
	if ((c < a && c < b) || (c > a && c > b))
		return 0;

	if (p != NULL) {
		p[0] = tmp[0];
		p[1] = tmp[1];
	}

	return 1;
}

// do the edges of polya and polyb collide? (Does NOT test for containment).
int g2d_polygon_intersects_polygon(const zarray_t* polya, const zarray_t* polyb)
{
	// do any of the line segments collide? If so, the answer is no.

	// dumb N^2 method.
	for (int ia = 0; ia < zarray_size(polya); ia++) {
		double pa0[2], pa1[2];
		zarray_get(polya, ia, pa0);
		zarray_get(polya, (ia + 1) % zarray_size(polya), pa1);

		g2d_line_segment_t sega;
		g2d_line_segment_init_from_points(&sega, pa0, pa1);

		for (int ib = 0; ib < zarray_size(polyb); ib++) {
			double pb0[2], pb1[2];
			zarray_get(polyb, ib, pb0);
			zarray_get(polyb, (ib + 1) % zarray_size(polyb), pb1);

			g2d_line_segment_t segb;
			g2d_line_segment_init_from_points(&segb, pb0, pb1);

			if (g2d_line_segment_intersect_segment(&sega, &segb, NULL))
				return 1;
		}
	}

	return 0;
}
// compute a point that is inside the polygon. (It may not be *far* inside though)
void g2d_polygon_get_interior_point(const zarray_t* poly, double* p)
{
	// take the first three points, which form a triangle. Find the middle point
	double a[2], b[2], c[2];

	zarray_get(poly, 0, a);
	zarray_get(poly, 1, b);
	zarray_get(poly, 2, c);

	p[0] = (a[0] + b[0] + c[0]) / 3;
	p[1] = (a[1] + b[1] + c[1]) / 3;
}
int g2d_polygon_contains_point_ref(const zarray_t* poly, double q[2])
{
	// use winding. If the point is inside the polygon, we'll wrap
	// around it (accumulating 6.28 radians). If we're outside the
	// polygon, we'll accumulate zero.
	int psz = zarray_size(poly);

	double acc_theta = 0;

	double last_theta;

	for (int i = 0; i <= psz; i++) {
		double p[2];

		zarray_get(poly, i % psz, &p);

		double this_theta = atan2(q[1] - p[1], q[0] - p[0]);

		if (i != 0)
			acc_theta += mod2pi(this_theta - last_theta);

		last_theta = this_theta;
	}

	return acc_theta > M_PI;
}
int g2d_polygon_contains_point(const zarray_t* poly, double q[2])
{
	// use winding. If the point is inside the polygon, we'll wrap
	// around it (accumulating 6.28 radians). If we're outside the
	// polygon, we'll accumulate zero.
	int psz = zarray_size(poly);
	assert(psz > 0);

	int last_quadrant;
	int quad_acc = 0;

	for (int i = 0; i <= psz; i++) {
		double* p;

		zarray_get_volatile(poly, i % psz, &p);

		// p[0] < q[0]       p[1] < q[1]    quadrant
		//     0                 0              0
		//     0                 1              3
		//     1                 0              1
		//     1                 1              2

		// p[1] < q[1]       p[0] < q[0]    quadrant
		//     0                 0              0
		//     0                 1              1
		//     1                 0              3
		//     1                 1              2

		int quadrant;
		if (p[0] < q[0])
			quadrant = (p[1] < q[1]) ? 2 : 1;
		else
			quadrant = (p[1] < q[1]) ? 3 : 0;

		if (i > 0) {
			int dquadrant = quadrant - last_quadrant;

			// encourage a jump table by mapping to small positive integers.
			switch (dquadrant) {
			case -3:
			case 1:
				quad_acc++;
				break;
			case -1:
			case 3:
				quad_acc--;
				break;
			case 0:
				break;
			case -2:
			case 2:
			{
				// get the previous point.
				double* p0;
				zarray_get_volatile(poly, i - 1, &p0);

				// Consider the points p0 and p (the points around the
				//polygon that we are tracing) and the query point q.
				//
				// If we've moved diagonally across quadrants, we want
				// to measure whether we have rotated +PI radians or
				// -PI radians. We can test this by computing the dot
				// product of vector (p0-q) with the vector
				// perpendicular to vector (p-q)
				double nx = p[1] - q[1];
				double ny = -p[0] + q[0];

				double dot = nx * (p0[0] - q[0]) + ny * (p0[1] - q[1]);
				if (dot < 0)
					quad_acc -= 2;
				else
					quad_acc += 2;

				break;
			}
			}
		}

		last_quadrant = quadrant;
	}

	int v = (quad_acc >= 2) || (quad_acc <= -2);

	if (0 && v != g2d_polygon_contains_point_ref(poly, q)) {
		printf("FAILURE %d %d\n", v, quad_acc);
		exit(-1);
	}

	return v;
}
int g2d_polygon_overlaps_polygon(const zarray_t* polya, const zarray_t* polyb)
{
	// do any of the line segments collide? If so, the answer is yes.
	if (g2d_polygon_intersects_polygon(polya, polyb))
		return 1;

	// if none of the edges cross, then the polygon is either fully
	// contained or fully outside.
	double p[2];
	g2d_polygon_get_interior_point(polyb, p);

	if (g2d_polygon_contains_point(polya, p))
		return 1;

	g2d_polygon_get_interior_point(polya, p);

	if (g2d_polygon_contains_point(polyb, p))
		return 1;

	return 0;
}
int prefer_smaller(int pref, double q0, double q1)
{
	if (pref)     // already prefer something? exit.
		return pref;

	if (q0 < q1)
		return -1; // we now prefer q0
	if (q1 < q0)
		return 1; // we now prefer q1

	// no preference
	return 0;
}
/**
 * Removes the entry at index 'idx'.
 * If shuffle is true, the last element in the array will be placed in
 * the newly-open space; if false, the zarray is compacted.
 */
static inline void zarray_remove_index(zarray_t* za, int idx, int shuffle)
{
	assert(za != NULL);
	assert(idx >= 0);
	assert(idx < za->size);

	if (shuffle) {
		if (idx < za->size - 1)
			memcpy(&za->data[idx * za->el_sz], &za->data[(za->size - 1) * za->el_sz], za->el_sz);
		za->size--;
		return;
	}
	else {
		// size = 10, idx = 7. Should copy 2 entries (at idx=8 and idx=9).
		// size = 10, idx = 9. Should copy 0 entries.
		int ncopy = za->size - idx - 1;
		if (ncopy > 0)
			memmove(&za->data[idx * za->el_sz], &za->data[(idx + 1) * za->el_sz], ncopy * za->el_sz);
		za->size--;
		return;
	}
}
/**
 * Removes all elements from the array and sets its size to zero. Pointers to
 * any data elements obtained i.e. by zarray_get_volatile() will no longer be
 * valid.
 */
static inline void zarray_clear(zarray_t* za)
{
	assert(za != NULL);
	za->size = 0;
}
void apriltag_detector_clear_families(apriltag_detector_t* td)
{
	zarray_clear(td->tag_families);
}
void apriltag_detector_destroy(apriltag_detector_t* td)
{
	apriltag_detector_clear_families(td);

	zarray_destroy(td->tag_families);
	free(td);
}
void imlib_find_rects(list_t* out, image_t* ptr, rectangle_t* roi, uint32_t threshold)
{
	// Frame Buffer Memory Usage...
	// -> GRAYSCALE Input Image = w*h*1
	// -> GRAYSCALE Threhsolded Image = w*h*1
	// -> UnionFind = w*h*4 (+w*h*2 for hash table)
	size_t resolution = roi->w * roi->h;
	size_t fb_alloc_need = resolution * (1 + 1 + 4 + 2); // read above...
	umm_init_x(((fb_avail() - fb_alloc_need) / resolution) * resolution);
	apriltag_detector_t* td = apriltag_detector_create();

	uint8_t* grayscale_image = fb_alloc(roi->w * roi->h);
	uint8_t* grayscale_image_to_free = grayscale_image;

	image_u8_t im;
	im.width = roi->w;
	im.height = roi->h;
	im.stride = roi->w;
	im.buf = grayscale_image;

	switch (ptr->bpp) {
	case IMAGE_BPP_BINARY: {
		for (int y = roi->y, yy = roi->y + roi->h; y < yy; y++) {
			uint32_t* row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(ptr, y);
			for (int x = roi->x, xx = roi->x + roi->w; x < xx; x++) {
				*(grayscale_image++) = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, x));
			}
		}
		break;
	}
	case IMAGE_BPP_GRAYSCALE: {
		for (int y = roi->y, yy = roi->y + roi->h; y < yy; y++) {
			uint8_t* row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(ptr, y);
			for (int x = roi->x, xx = roi->x + roi->w; x < xx; x++) {
				*(grayscale_image++) = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, x);
			}
		}
		break;
	}
	case IMAGE_BPP_RGB565: {
		for (int y = roi->y, yy = roi->y + roi->h; y < yy; y++) {
			uint16_t* row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(ptr, y);
			for (int x = roi->x, xx = roi->x + roi->w; x < xx; x++) {
				*(grayscale_image++) = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, x));
			}
		}
		break;
	}
	default: {
		memset(grayscale_image, 0, roi->w * roi->h);
		break;
	}
	}

	///////////////////////////////////////////////////////////
	// Detect quads according to requested image decimation
	// and blurring parameters.

//    zarray_t *detections = apriltag_quad_gradient(td, &im, true);
	// int the _quad_thresh func, we set nclustermap as a default value, to avoid using the fb_alloc_all....
	zarray_t* detections = apriltag_quad_thresh(td, &im, true);

	td->nquads = zarray_size(detections);

	////////////////////////////////////////////////////////////////
	// Decode tags from each quad.
	if (1) {
		for (int i = 0; i < zarray_size(detections); i++) {
			struct quad* quad_original;
			zarray_get_volatile(detections, i, &quad_original);

			// refine edges is not dependent upon the tag family, thus
			// apply this optimization BEFORE the other work.
			//if (td->quad_decimate > 1 && td->refine_edges) {
			if (td->refine_edges) {
				refine_edges(td, &im, quad_original);
			}

			// make sure the homographies are computed...
			if (quad_update_homographies(quad_original))
				continue;
		}
	}

	////////////////////////////////////////////////////////////////
	// Reconcile detections--- don't report the same tag more
	// than once. (Allow non-overlapping duplicate detections.)
	if (1) {
		zarray_t* poly0 = g2d_polygon_create_zeros(4);
		zarray_t* poly1 = g2d_polygon_create_zeros(4);

		for (int i0 = 0; i0 < zarray_size(detections); i0++) {

			struct quad* det0;
			zarray_get_volatile(detections, i0, &det0);

			for (int k = 0; k < 4; k++)
				zarray_set(poly0, k, det0->p[k], NULL);

			for (int i1 = i0 + 1; i1 < zarray_size(detections); i1++) {

				struct quad* det1;
				zarray_get_volatile(detections, i1, &det1);

				for (int k = 0; k < 4; k++)
					zarray_set(poly1, k, det1->p[k], NULL);

				if (g2d_polygon_overlaps_polygon(poly0, poly1)) {
					// the tags overlap. Delete one, keep the other.

					int pref = 0; // 0 means undecided which one we'll keep.

					// if we STILL don't prefer one detection over the other, then pick
					// any deterministic criterion.
					for (int i = 0; i < 4; i++) {
						pref = prefer_smaller(pref, det0->p[i][0], det1->p[i][0]);
						pref = prefer_smaller(pref, det0->p[i][1], det1->p[i][1]);
					}

					if (pref == 0) {
						// at this point, we should only be undecided if the tag detections
						// are *exactly* the same. How would that happen?
						printf("uh oh, no preference for overlappingdetection\n");
					}

					if (pref < 0) {
						// keep det0, destroy det1
						matd_destroy(det1->H);
						matd_destroy(det1->Hinv);
						zarray_remove_index(detections, i1, 1);
						i1--; // retry the same index
						goto retry1;
					}
					else {
						// keep det1, destroy det0
						matd_destroy(det0->H);
						matd_destroy(det0->Hinv);
						zarray_remove_index(detections, i0, 1);
						i0--; // retry the same index.
						goto retry0;
					}
				}

			retry1:;
			}

		retry0:;
		}

		zarray_destroy(poly0);
		zarray_destroy(poly1);
	}

	list_init(out, sizeof(find_rects_list_lnk_data_t));
#ifdef WIN32
#define fast_roundf(x) (int)roundf(x)
#define fast_sqrtf(x) sqrtf(x)
#endif
	const int r_diag_len = fast_roundf(fast_sqrtf((roi->w * roi->w) + (roi->h * roi->h))) * 2;
	int* theta_buffer = fb_alloc(sizeof(int) * r_diag_len);
	uint32_t* mag_buffer = fb_alloc(sizeof(uint32_t) * r_diag_len);
	point_t* point_buffer = fb_alloc(sizeof(point_t) * r_diag_len);

	for (int i = 0, j = zarray_size(detections); i < j; i++) {
		struct quad* det;
		zarray_get_volatile(detections, i, &det);
		line_t lines[4];
		lines[0].x1 = fast_roundf(det->p[0][0]) + roi->x; lines[0].y1 = fast_roundf(det->p[0][1]) + roi->y;
		lines[0].x2 = fast_roundf(det->p[1][0]) + roi->x; lines[0].y2 = fast_roundf(det->p[1][1]) + roi->y;
		lines[1].x1 = fast_roundf(det->p[1][0]) + roi->x; lines[1].y1 = fast_roundf(det->p[1][1]) + roi->y;
		lines[1].x2 = fast_roundf(det->p[2][0]) + roi->x; lines[1].y2 = fast_roundf(det->p[2][1]) + roi->y;
		lines[2].x1 = fast_roundf(det->p[2][0]) + roi->x; lines[2].y1 = fast_roundf(det->p[2][1]) + roi->y;
		lines[2].x2 = fast_roundf(det->p[3][0]) + roi->x; lines[2].y2 = fast_roundf(det->p[3][1]) + roi->y;
		lines[3].x1 = fast_roundf(det->p[3][0]) + roi->x; lines[3].y1 = fast_roundf(det->p[3][1]) + roi->y;
		lines[3].x2 = fast_roundf(det->p[0][0]) + roi->x; lines[3].y2 = fast_roundf(det->p[0][1]) + roi->y;

		uint32_t magnitude = 0;

		for (int i = 0; i < 4; i++) {
			if (!lb_clip_line(&lines[i], 0, 0, ptr->w, ptr->h)) {
				continue;
			}

			size_t index = trace_line(ptr, &lines[i], theta_buffer, mag_buffer, point_buffer);

			for (int j = 0; j < index; j++) {
				magnitude += mag_buffer[j];
			}
		}

		if (magnitude < threshold) {
			continue;
		}

		find_rects_list_lnk_data_t lnk_data;
		rectangle_init(&(lnk_data.rect), fast_roundf(det->p[0][0]) + roi->x, fast_roundf(det->p[0][1]) + roi->y, 0, 0);

		for (size_t k = 1, l = (sizeof(det->p) / sizeof(det->p[0])); k < l; k++) {
			rectangle_t temp;
			rectangle_init(&temp, fast_roundf(det->p[k][0]) + roi->x, fast_roundf(det->p[k][1]) + roi->y, 0, 0);
			rectangle_united(&(lnk_data.rect), &temp);
		}

		// Add corners...
		lnk_data.corners[0].x = fast_roundf(det->p[3][0]) + roi->x; // top-left
		lnk_data.corners[0].y = fast_roundf(det->p[3][1]) + roi->y; // top-left
		lnk_data.corners[1].x = fast_roundf(det->p[2][0]) + roi->x; // top-right
		lnk_data.corners[1].y = fast_roundf(det->p[2][1]) + roi->y; // top-right
		lnk_data.corners[2].x = fast_roundf(det->p[1][0]) + roi->x; // bottom-right
		lnk_data.corners[2].y = fast_roundf(det->p[1][1]) + roi->y; // bottom-right
		lnk_data.corners[3].x = fast_roundf(det->p[0][0]) + roi->x; // bottom-left
		lnk_data.corners[3].y = fast_roundf(det->p[0][1]) + roi->y; // bottom-left

		lnk_data.magnitude = magnitude;

		list_push_back(out, &lnk_data);
	}
#ifdef WIN32
#undef fast_roundf(x) // un define the fast_roundf, the MSC without thesre two funcs
#undef fast_sqrtf(x) //un define the fast_sqrtf
#endif
	fb_free(); // point_buffer
	fb_free(); // mag_buffer
	fb_free(); // theta_buffer

	zarray_destroy(detections);
	fb_free(); // grayscale_image;
	apriltag_detector_destroy(td);
	fb_free(); // umm_init_x();
}
