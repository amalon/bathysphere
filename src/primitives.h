/*
 * primitives.h
 *
 * Copyright (C) 2007-2014 James Hogan <james@albanarts.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 * (in the file called COPYING).
 *
 *
 * Drawing functions for mathematical shapes.
 *
 */

#ifndef _FILE_primitives_h
#define _FILE_primitives_h

namespace Drawing {

  /// Draw a torus centered on the Z axis.
  /**
  * @param ringRadius Radius of the main ring of the torus.
  * @param tubeRadius Radius of the tube of the main ring.
  * @param ringSections Number of sections around the main ring.
  * @param tubeSections Number of sections around the tube of the main ring.
  * @param z Optional z offset.
  * @param tubeSectionStart Start tube section to render.
  * @param tubeSectionEnd Last tube section to render.
  */
  void Torus(float ringRadius, float tubeRadius, unsigned int ringSections, unsigned int tubeSections, float z = 0.0f,  int tubeSectionStart = 0, int tubeSectionEnd = 0);
  
  /// Draw a simple solid circle.
  /**
   * @param radius Radius of the disc.
   * @param sections Number of verticies around the disk.
   * @param z Z coordinate of the disk.
   * @param flip Whether the disk should face downwards.
   */
  void Disk(float radius, unsigned int sections, float z = 0.0f, bool flip = false);
  
  /// Draw a disk with thickness.
  /**
   * @param radius Radius of the flat part of the disk.
   * @param thickness Half thickness of the sphere.
   * @param sections Number of sections around the disk.
   * @param z Z coordinate of the center of the disk.
   */
  void ThickDisk(float radius, float thickness, unsigned int sections, float z = 0.0f);
  
  /// Draw a cylinder.
  /**
   * @param topRadius Radius of the cylinder at the top.
   * @param bottomRadius Radius of the cylinder at the bottom.
   * @param topZ Z coordinate at the top.
   * @param bottomZ Z coordinate at the bottom.
   * @param ringSections Number of sections around the circle of the sphere.
   * @param capTop Whether to cap the top of the cylinder.
   * @param capBottom Whether to cap the bottom of the cylinder.
   */
  void Cylinder(float topRadius, float bottomRadius,
                float topZ, float bottomZ,
                unsigned int ringSections,
                bool capTop = false, bool capBottom = false);

  /// Draw an arrow.
  /**
   * @param lineRadius Radius of the line.
   * @param headRadius Radius of the arrow head.
   * @param headLength Length of the arrow head.
   * @param lineEnd End coordinate of the arrow.
   * @param lineStart Start coordinate of the arrow.
   * 
   * Draws a solid arrow pointing in the positive Z direction.
   */
  void Arrow(float lineRadius, float headRadius, float headLength, float lineEnd, float lineStart = 0.0f, int segments = 32);
}

#endif // _FILE_primitives_h
