#pragma once

#include "EbsdLib/EbsdLib.h"

#include <vector>

namespace canvas_ity
{
class canvas;
}

namespace ebsdlib::fonts
{

/** @brief Return immutable embedded font bytes. Initialization and concurrent reads are thread-safe.
 * @return A shared reference valid until static destruction at program shutdown.
 */
EbsdLib_EXPORT const std::vector<unsigned char>& GetFiraSansRegular();

/** @brief Return immutable embedded font bytes. Initialization and concurrent reads are thread-safe.
 * @return A shared reference valid until static destruction at program shutdown.
 */
EbsdLib_EXPORT const std::vector<unsigned char>& GetLatoRegular();

/** @brief Return immutable embedded font bytes. Initialization and concurrent reads are thread-safe.
 * @return A shared reference valid until static destruction at program shutdown.
 */
EbsdLib_EXPORT const std::vector<unsigned char>& GetLatoBold();

/** @brief Load trusted embedded TTF bytes into a caller-owned canvas.
 * The canvas copies the required bytes. Concurrent calls require separate canvases.
 * @param context Canvas that receives the font.
 * @param data Embedded TTF bytes.
 * @param size Font size in pixels.
 * @return True on success; false for empty, oversized, or invalid font data.
 */
[[nodiscard]] EbsdLib_EXPORT bool SetFont(canvas_ity::canvas& context, const std::vector<unsigned char>& data, float size);

/** @brief Load an embedded font and assert on failure in Debug builds.
 * Release builds attempt the load and continue without reporting failure.
 * The canvas copies the required bytes. Concurrent calls require separate canvases.
 * @param context Canvas that receives the font.
 * @param data Embedded TTF bytes.
 * @param size Font size in pixels.
 */
EbsdLib_EXPORT void LoadEmbeddedFont(canvas_ity::canvas& context, const std::vector<unsigned char>& data, float size);

// Simple Base64 decoder.  This is used at startup to decode the string
// literals containing embedded resource data, namely font files in TTF form.
//
inline void Base64Decode(char const* input, std::vector<unsigned char>& output)
{
  int index = 0;
  int data = 0;
  int held = 0;
  while(int symbol = input[index++])
  {
    if(symbol == '=')
    {
      break;
    }
    /* clang-format off */
    const int value = ('A' <= symbol && symbol <= 'Z' ? symbol - 'A' :
                       'a' <= symbol && symbol <= 'z' ? symbol - 'a' + 26 :
                       '0' <= symbol && symbol <= '9' ? symbol - '0' + 52 :
                       symbol == '+'                  ? 62 :
                       symbol == '/'                  ? 63 :
                                                        0);
    /* clang-format on */
    data = data << 6 | value;
    held += 6;
    if(held >= 8)
    {
      held -= 8;
      output.push_back(static_cast<unsigned char>((data >> held) & 0xff));
      data &= (1 << held) - 1;
    }
  }
}
} // namespace ebsdlib::fonts
