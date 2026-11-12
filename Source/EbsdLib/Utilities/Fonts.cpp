#include "Fonts.hpp"

#include "FiraSansRegular.hpp"
#include "LatoBold.hpp"
#include "LatoRegular.hpp"

#include <canvas_ity.hpp>

#include <cassert>
#include <cstddef>
#include <limits>
#include <vector>

namespace ebsdlib::fonts
{

const std::vector<unsigned char>& GetFiraSansRegular()
{
  static const std::vector<unsigned char> fontData = [] {
    std::vector<unsigned char> data;
    Base64Decode(fonts::k_FiraSansRegularBase64, data);
    return data;
  }();
  return fontData;
}

const std::vector<unsigned char>& GetLatoRegular()
{
  static const std::vector<unsigned char> fontData = [] {
    std::vector<unsigned char> data;
    Base64Decode(fonts::k_LatoRegularBase64, data);
    return data;
  }();
  return fontData;
}

const std::vector<unsigned char>& GetLatoBold()
{
  static const std::vector<unsigned char> fontData = [] {
    std::vector<unsigned char> data;
    Base64Decode(fonts::k_LatoBoldBase64, data);
    return data;
  }();
  return fontData;
}

bool SetFont(canvas_ity::canvas& context, const std::vector<unsigned char>& data, float size)
{
  // Empty data would otherwise resize the previous font and report success.
  if(data.empty() || data.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
  {
    return false;
  }
  return context.set_font(data.data(), static_cast<int>(data.size()), size);
}

void LoadEmbeddedFont(canvas_ity::canvas& context, const std::vector<unsigned char>& data, float size)
{
  if(!SetFont(context, data, size))
  {
    assert(false && "Failed to load embedded font; check embedded TTF data");
  }
}

} // namespace ebsdlib::fonts
