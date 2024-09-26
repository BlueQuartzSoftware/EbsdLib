#include "Fonts.hpp"

#include "FiraSansRegular.hpp"
#include "LatoBold.hpp"
#include "LatoRegular.hpp"

namespace EbsdLib::fonts
{

std::vector<unsigned char> GetFiraSansRegular()
{
  static std::vector<unsigned char> fontData;
  Base64Decode(fonts::k_FiraSansRegularBase64, fontData);
  return fontData;
}

std::vector<unsigned char> GetLatoRegular()
{
  static std::vector<unsigned char> fontData;
  Base64Decode(fonts::k_LatoRegularBase64, fontData);
  return fontData;
}

std::vector<unsigned char> GetLatoBold()
{
  static std::vector<unsigned char> fontData;
  Base64Decode(fonts::k_LatoBoldBase64, fontData);
  return fontData;
}

} // namespace EbsdLib::fonts
