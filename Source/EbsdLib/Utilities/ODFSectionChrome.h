#pragma once

#include "EbsdLib/Utilities/ODFSectionCompositor.h"

namespace canvas_ity
{
class canvas;
}

namespace ebsdlib
{
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
