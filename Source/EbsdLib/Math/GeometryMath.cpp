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
 * The code contained herein was partially funded by the followig contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "GeometryMath.h"

#include <algorithm>
#include <chrono>
#include <random>

#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/EbsdMatrixMath.h"

namespace EbsdLib
{
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
float GeometryMath::CosThetaBetweenVectors(const float a[3], const float b[3])
{
  float norm1 = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  float norm2 = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  if(norm1 == 0 || norm2 == 0)
  {
    return 1.0;
  }
  return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (norm1 * norm2);
}
double GeometryMath::CosThetaBetweenVectors(const double a[3], const double b[3])
{
  double norm1 = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  double norm2 = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  if(norm1 == 0 || norm2 == 0)
  {
    return 1.0;
  }
  return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (norm1 * norm2);
}
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
float GeometryMath::AngleBetweenVectors(const float a[3], const float b[3])
{
  float norm1 = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  float norm2 = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  float cosAng = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (norm1 * norm2);
  EbsdLibMath::bound(cosAng, -1.0f, 1.0f);
  return acos(cosAng);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool GeometryMath::PointInBox(const float p[3], const float ll[3], const float ur[3])
{
  return (ll[0] <= p[0]) && (p[0] <= ur[0]) && (ll[1] <= p[1]) && (p[1] <= ur[1]) && (ll[2] <= p[2]) && (p[2] <= ur[2]);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool GeometryMath::RayIntersectsBox(const float* p, const float* q, const float* ll, const float* ur)
{
  if((ll[0] > p[0]) && (ll[0] > q[0]))
  {
    return false;
  }
  if((ur[0] < p[0]) && (ur[0] < q[0]))
  {
    return false;
  }
  if((ll[1] > p[1]) && (ll[1] > q[1]))
  {
    return false;
  }
  if((ur[1] < p[1]) && (ur[1] < q[1]))
  {
    return false;
  }
  else if((ll[2] > p[2]) && (ll[2] > q[2]))
  {
    return false;
  }
  else if((ur[2] < p[2]) && (ur[2] < q[2]))
  {
    return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
float GeometryMath::LengthOfRayInBox(const float* p, const float* q, const float* ll, const float* ur)
{
  float length = 0.0;
  float frac = 0.0;

  float x1 = p[0];
  float y1 = p[1];
  float z1 = p[2];
  float x2 = q[0];
  float y2 = q[1];
  float z2 = q[2];

  float delX = x2 - x1;
  float delY = y2 - y1;
  float delZ = z2 - z1;

  float c1x = ll[0];
  float c1y = ll[1];
  float c1z = ll[2];
  float c2x = ur[0];
  float c2y = ur[1];
  float c2z = ur[2];

  // clip ray by min x face of box
  if(x1 < c1x && x2 > c1x)
  {
    frac = ((c1x - x1) / delX);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(x1 > c1x && x2 < c1x)
  {
    frac = ((c1x - x2) / delX);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(x1 < c1x && x2 < c1x)
  {
    return 0.0;
  }
  // clip ray by min y face of box
  if(y1 < c1y && y2 > c1y)
  {
    frac = ((c1y - y1) / delY);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(y1 > c1y && y2 < c1y)
  {
    frac = ((c1y - y2) / delY);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(y1 < c1y && y2 < c1y)
  {
    return 0.0;
  }
  // clip ray by min z face of box
  if(z1 < c1z && z2 > c1z)
  {
    frac = ((c1z - z1) / delZ);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(z1 > c1z && z2 < c1z)
  {
    frac = ((c1z - z2) / delZ);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(z1 < c1z && z2 < c1z)
  {
    return 0.0;
  }
  // clip ray by max x face of box
  if(x1 > c2x && x2 < c2x)
  {
    frac = ((c2x - x1) / delX);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(x1 < c2x && x2 > c2x)
  {
    frac = ((c2x - x2) / delX);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(x1 > c2x && x2 > c2x)
  {
    return 0.0;
  }
  // clip ray by max y face of box
  if(y1 > c2y && y2 < c2y)
  {
    frac = ((c2y - y1) / delY);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(y1 < c2y && y2 > c2y)
  {
    frac = ((c2y - y2) / delY);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(y1 > c2y && y2 > c2y)
  {
    return 0.0;
  }
  // clip ray by max z face of box
  if(z1 > c2z && z2 < c2z)
  {
    frac = ((c2z - z1) / delZ);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(z1 < c2z && z2 > c2z)
  {
    frac = ((c2z - z2) / delZ);
    x1 = x1 + (frac * delX);
    y1 = y1 + (frac * delY);
    z1 = z1 + (frac * delZ);
  }
  else if(z1 > c2z && z2 > c2z)
  {
    return 0.0;
  }

  length = ((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1));
  length = sqrt(length);

  return length;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::GenerateRandomRay(float length, float* ray)
{
  std::mt19937_64::result_type seed = static_cast<std::mt19937_64::result_type>(std::chrono::steady_clock::now().time_since_epoch().count());
  std::mt19937_64 generator(seed);
  std::uniform_real_distribution<> distribution(0.0, 1.0);

  float rand1 = distribution(generator);
  float rand2 = distribution(generator);

  ray[2] = (2.0f * rand1) - 1.0f;
  float t = static_cast<float>(EbsdLib::Constants::k_2Pi) * rand2;
  float w = std::sqrt(1.0f - (ray[2] * ray[2]));
  ray[0] = w * std::cos(t);
  ray[1] = w * std::sin(t);
  ray[0] *= length;
  ray[1] *= length;
  ray[2] *= length;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindPolygonNormal(const float* vertices, const int64_t numVerts, float n[3])
{
  // Return immediately if the  number of vertices cannot form a polygon
  if(numVerts < 3)
  {
    return;
  }

  // If the polygon is a triangle, then just compute its normal using simple cross product
  if(numVerts == 3)
  {
    GeometryMath::FindPlaneNormalVector(&vertices[0], &vertices[3], &vertices[6], n);
    return;
  }

  // If the polygon is not a triangle, then it may be concave; robustly
  // determine the normal by accumulating cross product
  std::vector<float> verts(3 * size_t(numVerts), 0.0f);
  std::copy(vertices, vertices + (3 * numVerts), verts.begin());
  float* vertPtr = verts.data();
  float array[3][3];
  float* a = array[0];
  float* b = array[1];
  float* c = array[2];
  float* tmp;
  std::copy(vertPtr, vertPtr + 3, b);
  std::copy(vertPtr + 3, vertPtr + 6, c);
  float vec0[3] = {0.0f, 0.0f, 0.0f};
  float vec1[3] = {0.0f, 0.0f, 0.0f};
  float tmpNormal[3] = {0.0f, 0.0f, 0.0f};

  // Cycle through point triplets, computing cross productes and accumulating
  // Two vertex positions remain the same (initialized above as a and b),
  // while the third is cycled to create each unique triplet
  for(int64_t i = 0; i < numVerts; i++)
  {
    tmp = a;
    a = b;
    b = c;
    c = tmp;

    c[0] = vertPtr[3 * ((i + 2) % numVerts) + 0];
    c[1] = vertPtr[3 * ((i + 2) % numVerts) + 1];
    c[2] = vertPtr[3 * ((i + 2) % numVerts) + 2];

    vec0[0] = c[0] - b[0];
    vec0[1] = c[1] - b[1];
    vec0[2] = c[2] - b[2];

    vec1[0] = a[0] - b[0];
    vec1[1] = a[1] - b[1];
    vec1[2] = a[2] - b[2];

    EbsdMatrixMath::CrossProduct(vec0, vec1, tmpNormal);
    std::transform(n, n + 3, tmpNormal, n, std::plus<float>());
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindPlaneNormalVector(const float a[3], const float b[3], const float c[3], float n[3])
{
  float ab[3], ac[3];

  ab[0] = b[0] - a[0];
  ab[1] = b[1] - a[1];
  ab[2] = b[2] - a[2];

  ac[0] = c[0] - a[0];
  ac[1] = c[1] - a[1];
  ac[2] = c[2] - a[2];

  EbsdMatrixMath::CrossProduct(ab, ac, n);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindPlaneNormalVector(const double a[3], const double b[3], const double c[3], double n[3])
{
  double ab[3], ac[3];

  ab[0] = b[0] - a[0];
  ab[1] = b[1] - a[1];
  ab[2] = b[2] - a[2];

  ac[0] = c[0] - a[0];
  ac[1] = c[1] - a[1];
  ac[2] = c[2] - a[2];

  EbsdMatrixMath::CrossProduct(ab, ac, n);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindPlaneCoefficients(const float a[3], const float b[3], const float c[3], float n[3], float& d)
{
  FindPlaneNormalVector(a, b, c, n);

  d = (a[0] * n[0]) + (a[1] * n[1]) + (a[2] * n[2]);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindDistanceToTriangleCentroid(const float* a, const float* b, const float* c, const float* q, float& distance)
{
  float centroid[3] = {0.0f, 0.0f, 0.0f};
  centroid[0] = (a[0] + b[0] + c[0]) / 3.0f;
  centroid[1] = (a[1] + b[1] + c[1]) / 3.0f;
  centroid[2] = (a[2] + b[2] + c[2]) / 3.0f;

  FindDistanceBetweenPoints(centroid, q, distance);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindDistanceFromPlane(const float* q, float n[3], float d, float& distance)
{
  distance = (q[0] * n[0]) + (q[1] * n[1]) + (q[2] * n[2]) - d;
  distance /= sqrtf((n[0] * n[0]) + (n[1] * n[1]) + (n[2] * n[2]));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindDistanceBetweenPoints(const float a[3], const float b[3], float& distance)
{
  float dx = b[0] - a[0];
  float dy = b[1] - a[1];
  float dz = b[2] - a[2];
  distance = sqrtf((dx * dx) + (dy * dy) + (dz * dz));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindTriangleArea(const float a[3], const float b[3], const float c[3], float& area)
{
  area = ((b[0] - a[0]) * (c[1] - a[1])) - ((c[0] - a[0]) * (b[1] - a[1]));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeometryMath::FindTetrahedronVolume(const float a[3], const float b[3], const float c[3], const float d[3], float& volume)
{
  float axdx, aydy, azdz, bxdx, bydy, bzdz, cxdx, cydy, czdz;

  axdx = a[0] - d[0];
  aydy = a[1] - d[1];
  azdz = a[2] - d[2];
  bxdx = b[0] - d[0];
  bydy = b[1] - d[1];
  bzdz = b[2] - d[2];
  cxdx = c[0] - d[0];
  cydy = c[1] - d[1];
  czdz = c[2] - d[2];

  volume = (azdz * ((bxdx * cydy) - (bydy * cxdx))) + (aydy * ((bzdz * cxdx) - (bxdx * czdz))) + (axdx * ((bydy * czdz) - (bzdz * cydy)));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
char GeometryMath::RayIntersectsTriangle(const float* a, const float* b, const float* c, const float* q, const float* r, float* p)
{
  char code = '?';
  int m = -1;

  code = RayIntersectsPlane(a, b, c, q, r, p, m);

  if(code == '0')
  {
    return '0';
  }
  if(code == 'q')
  {
    return PointInTriangle3D(a, b, c, m, q);
  }
  if(code == 'r')
  {
    return PointInTriangle3D(a, b, c, m, r);
  }
  if(code == 'p')
  {
    return 'p';
  }
  else if(code == '1')
  {
    return RayCrossesTriangle(a, b, c, q, r);
  }
  else
  {
    return code;
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
char GeometryMath::RayIntersectsPlane(const float* a, const float* b, const float* c, const float* q, const float* r, float* p, int& m)
{
  float n[3];
  float rq[3];
  float d, num, denom, t;

  FindPlaneCoefficients(a, b, c, n, d);

  num = d - ((q[0] * n[0]) + (q[1] * n[1]) + (q[2] * n[2]));
  rq[0] = r[0] - q[0];
  rq[1] = r[1] - q[1];
  rq[2] = r[2] - q[2];
  denom = (rq[0] * n[0]) + (rq[1] * n[1]) + (rq[2] * n[2]);
  m = EbsdMatrixMath::FindIndexOfMaxVal3x1(n);

  if(denom == 0.0)
  {
    if(num == 0.0)
    {
      return 'p';
    }

    return '0';
  }

  t = num / denom;
  for(int i = 0; i < 3; i++)
  {
    p[i] = q[i] + (t * (r[i] - q[i]));
  }
  if(t > 0.0 && t < 1.0)
  {
    return '1';
  }
  if(num == 0.0)
  {
    return 'q';
  }
  if(num == denom)
  {
    return 'r';
  }

  return '0';
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
char GeometryMath::RayIntersectsPlane(const float* n, const float d, const float* q, const float* r, float* p)
{
  float rq[3];
  float num, denom, t;

  num = d - ((q[0] * n[0]) + (q[1] * n[1]) + (q[2] * n[2]));
  rq[0] = r[0] - q[0];
  rq[1] = r[1] - q[1];
  rq[2] = r[2] - q[2];
  denom = (rq[0] * n[0]) + (rq[1] * n[1]) + (rq[2] * n[2]);

  if(denom == 0.0)
  {
    if(num == 0.0)
    {
      return 'p';
    }

    return '0';
  }

  t = num / denom;
  for(int i = 0; i < 3; i++)
  {
    p[i] = q[i] + (t * (r[i] - q[i]));
  }
  if(t > 0.0 && t < 1.0)
  {
    return '1';
  }
  if(num == 0.0)
  {
    return 'q';
  }
  if(num == denom)
  {
    return 'r';
  }

  return '0';
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
char GeometryMath::PointInTriangle3D(const float* a, const float* b, const float* c, int m, const float* p)
{
  float pp[3], aP[3], bP[3], cP[3];

  int j = 0;
  for(int i = 0; i < 3; i++)
  {
    if(i != m)
    {
      pp[j] = p[i];
      aP[j] = a[i];
      bP[j] = b[i];
      cP[j] = c[i];
      j++;
    }
  }
  return PointInTriangle2D(aP, bP, cP, pp);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
char GeometryMath::PointInTriangle2D(const float* a, const float* b, const float* c, const float* p)
{
  float area0, area1, area2;

  FindTriangleArea(p, a, b, area0);
  FindTriangleArea(p, b, c, area1);
  FindTriangleArea(p, c, a, area2);

  if((area0 == 0 && area1 > 0 && area2 > 0) || (area1 == 0 && area0 > 0 && area2 > 0) || (area2 == 0 && area0 > 0 && area1 > 0))
  {
    return 'E';
  }
  if((area0 == 0 && area1 < 0 && area2 < 0) || (area1 == 0 && area0 < 0 && area2 < 0) || (area2 == 0 && area0 < 0 && area1 < 0))
  {
    return 'E';
  }
  if((area0 > 0 && area1 > 0 && area2 > 0) || (area0 < 0 && area1 < 0 && area2 < 0))
  {
    return 'F';
  }
  if((area0 == 0 && area1 == 0 && area2 == 0))
  {
    return '?';
  }
  else if((area0 == 0 && area1 == 0) || (area0 == 0 && area2 == 0) || (area1 == 0 && area2 == 0))
  {
    return 'V';
  }
  else
  {
    return '0';
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
char GeometryMath::RayCrossesTriangle(const float* a, const float* b, const float* c, const float* q, const float* r)
{
  float vol0, vol1, vol2;

  FindTetrahedronVolume(q, a, b, r, vol0);
  FindTetrahedronVolume(q, b, c, r, vol1);
  FindTetrahedronVolume(q, c, a, r, vol2);

  if((vol0 > 0 && vol1 > 0 && vol2 > 0) || (vol0 < 0 && vol1 < 0 && vol2 < 0))
  {
    return 'f';
  }
  if((vol0 > 0 || vol1 > 0 || vol2 > 0) && (vol0 < 0 || vol1 < 0 || vol2 < 0))
  {
    return '0';
  }
  if((vol0 == 0 && vol1 == 0 && vol2 == 0))
  {
    return '?';
  }
  if((vol0 == 0 && vol1 == 0) || (vol0 == 0 && vol2 == 0) || (vol1 == 0 && vol2 == 0))
  {
    return 'v';
  }
  else if(vol0 == 0 || vol1 == 0 || vol2 == 0)
  {
    return 'e';
  }
  else
  {
    return '?';
  }
}
} // namespace EbsdLib
