#pragma once

#include "EbsdLib/Utilities/ODFSectionCompositor.h"

namespace canvas_ity
{
class canvas;
}

namespace ebsdlib
{
/**
 * @brief Formats the φ₂ section heading with a degree symbol.
 * @param angleDeg Section angle in degrees.
 * @return Heading with the angle in degrees.
 */
EbsdLib_EXPORT std::string FormatODFSectionTitle(double angleDeg);

/**
 * @brief Returns the horizontal Euler-axis title as UTF-8 bytes.
 * @return The φ₁ title.
 */
EbsdLib_EXPORT std::string GetODFHorizontalAxisTitle();

/**
 * @brief Returns the vertical Euler-axis title as UTF-8 bytes.
 * @return The Φ title.
 */
EbsdLib_EXPORT std::string GetODFVerticalAxisTitle();

/**
 * @brief Selects a tick interval from the angular range and axis length.
 * @param maximumDeg Positive axis maximum in degrees.
 * @param pixelLength Positive axis length in pixels.
 * @return Interval of 10 or 20 degrees.
 */
EbsdLib_EXPORT double SelectODFAxisTickInterval(double maximumDeg, int32_t pixelLength);

/**
 * @brief Generates axis ticks that include zero and the exact maximum.
 * @param maximumDeg Positive axis maximum in degrees.
 * @param pixelLength Positive axis length in pixels.
 * @return Tick angles in degrees, in increasing order.
 */
EbsdLib_EXPORT std::vector<double> GenerateODFAxisTicks(double maximumDeg, int32_t pixelLength);

/**
 * @brief Selects numeric labels with measured text widths and four pixels of padding.
 * @param maximumDeg Positive axis maximum in degrees.
 * @param pixelLength Positive axis length in pixels.
 * @param fontSize Tick font size in pixels, or zero to derive it from the axis length.
 * @return Labeled tick angles, including both endpoints when their text fits without overlap.
 */
EbsdLib_EXPORT std::vector<double> GenerateODFAxisLabelTicks(double maximumDeg, int32_t pixelLength, float fontSize = 0.0f);

/**
 * @brief Measures the gutter needed for separate vertical title and numeric-label regions.
 * @param fontSize Axis-title font size in pixels.
 * @param tickSize Numeric-label font size in pixels.
 * @return Gutter width in pixels, including space for tick hashes and text padding.
 */
EbsdLib_EXPORT float ComputeODFLeftAxisGutter(float fontSize, float tickSize);

/**
 * @brief Names the rendered MUD scale and identifies Count-Density sources.
 * @param sourceUnits Validated source-value units.
 * @return Color-bar title for the source units.
 */
EbsdLib_EXPORT std::string GetODFColorBarTitle(ODFValueUnits sourceUnits);

/**
 * @brief Returns the upper-left data-rectangle position for one section.
 * @param layout Validated page metrics.
 * @param sectionIndex Zero-based section index in row-major order.
 * @return Horizontal and vertical pixel coordinates.
 */
EbsdLib_EXPORT std::array<float, 2> GetODFSectionPanelOrigin(const ODFSectionLayoutMetrics& layout, size_t sectionIndex);

/**
 * @brief Draws embedded-font titles, axes, shared color bar, and ODF metadata over the panels.
 * @param context Receives the annotations on a white page with the data panels already drawn.
 * @param config Validated presentation settings and source-grid metadata.
 * @param sections Prepared angles and Laue limits.
 * @param layout Validated page metrics.
 * @param minimumMUD Applied lower scale endpoint.
 * @param maximumMUD Applied upper scale endpoint.
 * @param colorBar One-column RGBA image with panelHeight rows, from maximum to minimum MUD.
 */
EbsdLib_EXPORT void DrawODFSectionChrome(canvas_ity::canvas& context, const ODFSectionConfiguration& config, const PreparedODFSections& sections, const ODFSectionLayoutMetrics& layout,
                                         double minimumMUD, double maximumMUD, const std::vector<uint8_t>& colorBar);
} // namespace ebsdlib
