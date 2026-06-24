#include "EbsdLib/Utilities/PoleFigureProjection.h"

namespace ebsdlib
{
std::array<float, 2> StereographicProjectUpperHemisphere(float x, float y, float z)
{
  if(z < 0.0f)
  {
    x = -x;
    y = -y;
    z = -z;
  }
  const float denom = 1.0f + z; // z >= 0 here, so denom >= 1 (never zero)
  return {x / denom, y / denom};
}
} // namespace ebsdlib
