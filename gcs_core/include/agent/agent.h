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
#ifndef _MBZIRC_AGENT_AGENT_H_
#define _MBZIRC_AGENT_AGENT_H_

#include <grvc_quadrotor_hal/types.h>
#include <gcs_core/util/types.h>

namespace grvc { namespace mbzirc {

	class Robot {
	public:
		// Chases a target and takes it to the drop zone.
		//void captureStaticTarget	(const Target::Desc&);
		void goTo					(const hal::Waypoint& _wp);
		// Alternative: goTo(2D) + setHeight + setAngle

		const Vector& position		() const;
		/// A rectangle that is guaranteed to be scanned by the view frustrum of the camera.
		/// It should be the bigest rectangle inscribed in the intersection of the frustrum with the floor,
		/// or the closest possible approximation
		Rectangle viewArea			() const;
		bool isOnTarget				() const;
		bool isBussy				() const;
		bool succeeded				() const;

		/// uses the camera to scan the terrain under the robot and search for targets
		/// \return the number of targets found
		//unsigned scanTerrain		(std::vector<Target::Desc>& _dst);
	};
}}	// namespace grvc::mbzirc

#endif // _MBZIRC_AGENT_AGENT_H_