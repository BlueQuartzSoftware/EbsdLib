#pragma once

#include "EbsdLib/Utilities/ODFSectionUtilities.h"

namespace ebsdlib
{
/**
 * @enum ODFScaleMode
 * @brief Selects the shared MUD color range.
 */
enum class ODFScaleMode : uint8_t
{
  Automatic = 0, ///< Uses zero and the maximum displayed MUD value.
  Manual = 1     ///< Uses the supplied MUD endpoints and clips values outside them.
};

/**
 * @struct ODFSectionConfiguration
 * @brief Supplies the borrowed grid and presentation settings for standard ODF sections.
 *
 * The grid values must remain valid until generateCompositeImage() returns.
 */
struct EbsdLib_EXPORT ODFSectionConfiguration
{
  ODFGridView grid;
  size_t sectionCount = 6;
  size_t sectionsPerRow = 3;
  int32_t sectionWidth = 512;
  ODFScaleMode scaleMode = ODFScaleMode::Automatic;
  double manualMinimumMUD = 0.0;
  double manualMaximumMUD = 1.0;
  /**
   * @brief At least two [position, red, green, blue] controls, with all components in [0, 1].
   * Positions must increase strictly. Values outside the control positions use the nearest endpoint color.
   */
  std::vector<float> colorControlPoints;
  std::string title;
  std::string materialName;
  int32_t phaseNumber = 1;
};

/**
 * @struct ODFSectionLayoutMetrics
 * @brief Contains deterministic page and panel dimensions in pixels.
 *
 * Zero page dimensions indicate invalid or unrepresentable layout inputs.
 */
struct EbsdLib_EXPORT ODFSectionLayoutMetrics
{
  int32_t panelWidth = 0;
  int32_t panelHeight = 0;
  int32_t columns = 0;
  int32_t rows = 0;
  float fontPtSize = 0.0f;
  float margin = 0.0f;
  float tickFontSize = 0.0f;
  float leftAxisGutter = 0.0f;
  float panelSlotWidth = 0.0f;
  float panelSlotHeight = 0.0f;
  float titleHeight = 0.0f;
  float legendWidth = 0.0f;
  int32_t pageWidth = 0;
  int32_t pageHeight = 0;
};

/**
 * @struct ODFSectionResult
 * @brief Contains an owned RGBA page, section metadata, or a validation error.
 */
struct EbsdLib_EXPORT ODFSectionResult
{
  UInt8ArrayType::Pointer image;
  int32_t width = 0;
  int32_t height = 0;
  std::vector<double> sectionAnglesDeg;
  double appliedMinimumMUD = 0.0;
  double appliedMaximumMUD = 0.0;
  int32_t errorCode = 0;
  std::string errorMessage;
  /**
   * @brief Reports whether composition produced an image without an error.
   * @return True if the result contains an image and a nonnegative error code.
   */
  explicit operator bool() const noexcept
  {
    return errorCode >= 0 && image != nullptr;
  }
};

/**
 * @brief Computes the page layout from the Laue limits and panel settings.
 * @param config Supplies the Laue index, section count, columns, and panel width.
 * @return Pixel metrics, or zero metrics if layout inputs are invalid or exceed canvas integer limits.
 */
EbsdLib_EXPORT ODFSectionLayoutMetrics ComputeODFSectionLayout(const ODFSectionConfiguration& config);

/**
 * @class ODFSectionCompositor
 * @brief Prepares standard ODF sections and draws one annotated RGBA page.
 */
class EbsdLib_EXPORT ODFSectionCompositor
{
public:
  /**
   * @brief Draws panels with a shared MUD color bar and embedded-font annotations.
   * @param config Supplies the borrowed source grid, color controls, scale, and labels.
   * @return Owned image and applied metadata, or a negative error code with a diagnostic message.
   * @note Errors -7510 through -7513 identify width/layout, columns, scale, and color-control failures.
   * Grid and section-count errors retain the PrepareODFSections() error codes.
   */
  ODFSectionResult generateCompositeImage(const ODFSectionConfiguration& config) const;
};
} // namespace ebsdlib
