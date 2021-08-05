/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once

#include <memory>

#include <string>

#include "EbsdLib/EbsdLib.h"

namespace EbsdLib
{
/*
 * @class GeometryMath GeometryMath.h DREAM3DLib/Common/GeometryMath.h
 * @brief This class performs Crystallographic Misorientation Calculations
 * @author Michael A. Jackson (BlueQuartz Software)
 * @author Michael A. Groeber (US Air Force Research Laboratory)
 * @date Feb 19, 2011
 * @version 1.0
 */
namespace GeometryMath
{
/**
 * @brief Calculates the Cosine of the angle between 2 vectors. To get the actual angle the programmer should
 * use the following form: float radians = acos(GeometryMath::CosThetaBetweenVectors(a, b));
 * @param a 1x3 Vector
 * @param b 1x3 Vector
 * @return
 */
EbsdLib_EXPORT float CosThetaBetweenVectors(const float a[3], const float b[3]);
EbsdLib_EXPORT double CosThetaBetweenVectors(const double a[3], const double b[3]);

/**
 * @brief Computes the angle in RADIANS between 2 vectors.
 * @param a
 * @param b
 * @return
 */
EbsdLib_EXPORT float AngleBetweenVectors(const float a[3], const float b[3]);

/**
 * @brief Computes the area of a triangle defined by 3 points.
 * @param a
 * @param b
 * @param distance
 * @return
 */
EbsdLib_EXPORT void FindDistanceBetweenPoints(const float a[3], const float b[3], float& distance);

/**
 * @brief Computes the area of a triangle defined by 3 points.
 * @param a
 * @param b
 * @param c
 * @param area
 * @return
 */
EbsdLib_EXPORT void FindTriangleArea(const float a[3], const float b[3], const float c[3], float& area);

/**
 * @brief Computes the volume of a tetrahedron defined by 4 points.
 * @param a
 * @param b
 * @param c
 * @param d
 * @param volume
 * @return
 */
EbsdLib_EXPORT void FindTetrahedronVolume(const float a[3], const float b[3], const float c[3], const float d[3], float& volume);

/**
 * @brief Generic function to compute the normal of a planar polygon embedded in 3D.
 * This functions will consistently handle concave polygons.
 * @param vertices
 * @param numVerts
 * @param n
 */
EbsdLib_EXPORT void FindPolygonNormal(const float* vertices, const int64_t numVerts, float n[3]);

/**
 * @brief Computes the normal vector to a plane containing 3 points (AB X AC).
 * @param a
 * @param b
 * @param c
 * @param n
 * @return
 */
EbsdLib_EXPORT void FindPlaneNormalVector(const float a[3], const float b[3], const float c[3], float n[3]);

/**
 * @brief Computes the normal vector to a plane containing 3 points (AB X AC).
 * @param a
 * @param b
 * @param c
 * @param n
 * @return
 */
EbsdLib_EXPORT void FindPlaneNormalVector(const double a[3], const double b[3], const double c[3], double n[3]);

/**
 * @brief Computes the coefficients of a plane containing 3 points (AB X AC).
 * @param a
 * @param b
 * @param c
 * @param n 1x3 Vector
 * @param d float
 * @return
 */
EbsdLib_EXPORT void FindPlaneCoefficients(const float a[3], const float b[3], const float c[3], float n[3], float& d);

/**
 * @brief Computes the distance of a point from the centroid of a triangle defined by 3 points
 * @param q
 * @param a
 * @param b
 * @param c
 * @param distance
 * @return
 */
EbsdLib_EXPORT void FindDistanceToTriangleCentroid(const float* a, const float* b, const float* c, const float* q, float& distance);

/**
 * @brief Computes the distance of a point from a plane.
 * @param q
 * @param n 1x3 Vector
 * @param d float
 * @param distance float
 * @return
 */
EbsdLib_EXPORT void FindDistanceFromPlane(const float* q, float n[3], float d, float& distance);

/**
 * @brief Determines if a point is inside of a box defined by the lower left and upper right corners
 * @param p
 * @param ll
 * @param ur
 * @return
 */
EbsdLib_EXPORT bool PointInBox(const float p[3], const float ll[3], const float ur[3]);

/**
 * @brief Determines if a point is inside of a triangle defined by 3 points
 * @param a
 * @param b
 * @param c
 * @param m int
 * @param p
 * @return
 */
EbsdLib_EXPORT char PointInTriangle3D(const float* a, const float* b, const float* c, int m, const float* p);

/**
 * @brief Determines if a point is inside of a triangle defined by 3 points
 * @param a
 * @param b
 * @param c
 * @param p
 * @return
 */
EbsdLib_EXPORT char PointInTriangle2D(const float* a, const float* b, const float* c, const float* p);

/**
 * @brief Determines if a segment between two points intersects a box defined by the lower left and upper right corners
 * @param p
 * @param q
 * @param lowerLeft
 * @param upperRight
 * @return
 */
EbsdLib_EXPORT bool RayIntersectsBox(const float* p, const float* q, const float* lowerLeft, const float* upperRight);

/**
 * @brief Determines length of a segment between two points that lies within a box defined by the lower left and upper right corners
 * @param p
 * @param q
 * @param lowerLeft
 * @param upperRight
 * @return
 */
EbsdLib_EXPORT float LengthOfRayInBox(const float* p, const float* q, const float* lowerLeft, const float* upperRight);

/**
 * @brief Creates a randomly oriented ray of given length
 * @param length float
 * @param ray 1x3 Vector
 * @return
 */
EbsdLib_EXPORT void GenerateRandomRay(float length, float ray[3]);

/**
 * @brief Determines if a segment between two points intersects a triangle defined by 3 points
 * @param q
 * @param r
 * @param a
 * @param b
 * @param c
 * @param p
 * @return
 */
EbsdLib_EXPORT char RayIntersectsTriangle(const float* a, const float* b, const float* c, const float* q, const float* r, float* p);

/**
 * @brief Determines if a segment between two points crosses a triangle defined by 3 points
 * @param q
 * @param r
 * @param a
 * @param b
 * @param c
 * @return
 */
EbsdLib_EXPORT char RayCrossesTriangle(const float* a, const float* b, const float* c, const float* q, const float* r);

/**
 * @brief Determines if a segment between two points intersects a plane defined by 3 points
 * @param q
 * @param r
 * @param a
 * @param b
 * @param c
 * @param p
 * @return
 */
EbsdLib_EXPORT char RayIntersectsPlane(const float* a, const float* b, const float* c, const float* q, const float* r, float* p, int& m);

/**
 * @brief Determines if a segment between two points intersects a plane defined by a normal and distance
 * @param n
 * @param d
 * @param q
 * @param r
 * @param p
 * @return
 */
EbsdLib_EXPORT char RayIntersectsPlane(const float* n, const float d, const float* q, const float* r, float* p);
} // namespace GeometryMath
} // namespace EbsdLib
