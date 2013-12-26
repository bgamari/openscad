/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2009-2011 Clifford Wolf <clifford@clifford.at> and
 *                          Marius Kintel <marius@kintel.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <math.h>

#include <iostream>
#include <algorithm>

#include "Polygon2d.h"
#include "DrawingCallback.h"

DrawingCallback::DrawingCallback(unsigned long fn) : fn(fn),
	pen(Vector2d(0, 0)), offset(Vector2d(0, 0)), advance(Vector2d(0, 0))
{
}

DrawingCallback::~DrawingCallback()
{
}

void DrawingCallback::start_glyph()
{
	this->polygon = new Polygon2d();
	this->polygon->setSanitized(true);
}

void DrawingCallback::finish_glyph()
{
	if (this->outline.size() > 0) {
		this->polygon->addOutline(this->outline);
        this->outline.clear();
	}
	if (this->polygon->outlines().size() == 0) {
		delete this->polygon;
		this->polygon = NULL;
	}
	if (this->polygon) this->polygons.push_back(this->polygon);
}

std::vector<const Polygon2d *> DrawingCallback::get_result()
{
	return this->polygons;
}

void DrawingCallback::set_glyph_offset(double offset_x, double offset_y)
{
	offset = Vector2d(offset_x, offset_y);
}

void DrawingCallback::add_glyph_advance(double advance_x, double advance_y)
{
	advance += Vector2d(advance_x, advance_y);
}

void DrawingCallback::add_vertex(Vector2d v)
{
	this->outline.push_back(v + offset + advance);
}

void DrawingCallback::move_to(Vector2d to)
{
	if (this->outline.size() > 0) {
		this->polygon->addOutline(this->outline);
		this->outline.clear();
	}
	add_vertex(to);
	pen = to;
}

void DrawingCallback::line_to(Vector2d to)
{
	add_vertex(to);
	pen = to;
}

void DrawingCallback::curve_to(Vector2d c1, Vector2d to)
{
	for (unsigned long idx = 1;idx <= fn;idx++) {
		const double a = idx * (1.0 / (double)fn);
		const double x = pen[0] * t(a, 2) + c1[0] * 2 * t(a, 1) * a + to[0] * a * a;
		const double y = pen[1] * t(a, 2) + c1[1] * 2 * t(a, 1) * a + to[1] * a * a;
		add_vertex(Vector2d(x, y));
	}
	pen = to;
}

void DrawingCallback::curve_to(Vector2d c1, Vector2d c2, Vector2d to)
{
	for (unsigned long idx = 1;idx <= fn;idx++) {
		const double a = idx * (1.0 / (double)fn);
		const double x = pen[0] * t(a, 3) + c1[0] * 3 * t(a, 2) * a + c2[0] * 3 * t(a, 1) * a * a + to[0] * a * a * a;
		const double y = pen[1] * t(a, 3) + c1[1] * 3 * t(a, 2) * a + c2[1] * 3 * t(a, 1) * a * a + to[1] * a * a * a;
		add_vertex(Vector2d(x, y));
	}
	pen = to;
}
