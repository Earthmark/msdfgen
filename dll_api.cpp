#include "msdfgen.h"
#include "msdfgen-ext.h"
#include "dll_exports.h"

using namespace msdfgen;

template <int N>
static void invertColor(const BitmapRef<float, N>& bitmap) {
	const float* end = bitmap.pixels + N * bitmap.width * bitmap.height;
	for (float* p = bitmap.pixels; p < end; ++p)
		*p = 1.f - *p;
}

DLL_EXPORT Shape* DLL_API create_shape()
{
	Shape* shape = new Shape();
	return shape;
}

DLL_EXPORT void DLL_API free_shape(Shape* shape)
{
	if (shape)
		delete shape;
}

DLL_EXPORT void DLL_API shape_bounds(Shape* shape, double& left, double& bottom, double& right, double& top)
{
	shape->bound(left, bottom, right, top);
}

DLL_EXPORT void DLL_API shape_edge_coloring_simple(Shape* shape, double angleThreshold, unsigned long long seed)
{
	edgeColoringSimple(*shape, angleThreshold, seed);
}

DLL_EXPORT Contour* DLL_API shape_add_contour(Shape* shape)
{
	return &shape->addContour();
}

DLL_EXPORT void DLL_API contour_add_line(Contour* contour, double fromX, double fromY, double toX, double toY)
{
	contour->addEdge(new LinearSegment(Point2(fromX, fromY), Point2(toX, toY)));
}

DLL_EXPORT void DLL_API contour_add_conic(Contour* contour, double fromX, double fromY,
	double controlX, double controlY, double toX, double toY)
{
	contour->addEdge(new QuadraticSegment(Point2(fromX, fromY), Point2(controlX, controlY), Point2(toX, toY)));
}

DLL_EXPORT void DLL_API contour_add_cubic(Contour* contour, double fromX, double fromY,
	double control0X, double control0Y, double control1X, double control1Y, double toX, double toY)
{
	contour->addEdge(new CubicSegment(Point2(fromX, fromY), Point2(control0X, control0Y), Point2(control1X, control1Y), Point2(toX, toY)));
}

DLL_EXPORT void DLL_API shape_generateMSDF(float* pixels, int width, int height,
	Shape* shape,
	double range, double scaleX, double scaleY, double offsetX, double offsetY,
	double edgeThreshold)
{
	shape->normalize();
	shape->inverseYAxis = true;

	BitmapRef<float, 3> msdf = BitmapRef<float, 3>(pixels, width, height);

	Vector2 scale = Vector2(scaleX, scaleY);
	Vector2 translate = Vector2(offsetX, offsetY);

	generateMSDF(msdf, *shape, range, scale, translate);

	// Auto fix shapes with reversed winding
	Point2 p(-100000, -100000);
	double dummy;
	SignedDistance minDistance;
	for (std::vector<Contour>::const_iterator contour = shape->contours.begin(); contour != shape->contours.end(); ++contour)
		for (std::vector<EdgeHolder>::const_iterator edge = contour->edges.begin(); edge != contour->edges.end(); ++edge) {
			SignedDistance distance = (*edge)->signedDistance(p, dummy);
			if (distance < minDistance)
				minDistance = distance;
		}

	if (minDistance.distance > 0)
		invertColor<3>(msdf);

	distanceSignCorrection(msdf, *shape, scale, translate, FILL_NONZERO);
}