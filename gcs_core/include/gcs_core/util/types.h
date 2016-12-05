//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 Carmelo J. Fernández-Agüera Tortosa
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
// Strategy testing and simulation environment for mbzirc competition
//----------------------------------------------------------------------------------------------------------------------
// Common types reused along the project
#ifndef _MBZIRC_UTIL_TYPES_H_
#define _MBZIRC_UTIL_TYPES_H_

#include <Eigen/Core>
#include <grvc_utils/units.h>
#include <grvc_quadrotor_uav/types.h>

namespace grvc { namespace mbzirc {

	//------------------------------------------------------------------------------------------------------------------
	typedef uav::WorldCoord	WorldCoord;

	//------------------------------------------------------------------------------------------------------------------
	typedef Eigen::Vector2d Vector2;

	//------------------------------------------------------------------------------------------------------------------
	struct Rectangle {
		Vector2 mMin, mMax;
		Vector2 clamp(const Vector2&) const;
		Vector2 center() const { return (mMax + mMin)*0.5f; }
		Vector2 size() const { return mMax - mMin; }
		bool contains(const Vector2& _p) const { return clamp(_p) == _p; }
	};

	//------------------------------------------------------------------------------------------------------------------
	inline Vector2 Rectangle::clamp(const Vector2& _pos) const {
		return Vector2(std::max(mMin.x(), std::min(mMax.x(), _pos.x())), 
						std::max(mMin.y(), std::min(mMax.y(), _pos.y())) );
	}

}} // namespace grvc::mbzirc

#endif // _MBZIRC_UTIL_TYPES_H_