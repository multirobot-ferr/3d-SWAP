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
#include <gcs_core/agent/agent.h>
#include <gcs_core/agent/arenaUav.h>
#include <gcs_core/agent/grvcUav.h>
#include <gcs_core/target/target.h>
#include <grvc_quadrotor_uav/server.h>

using namespace grvc::uav;

namespace grvc {
	namespace mbzirc {
		//--------------------------------------------------------------------------------------------------------------
		GrvcUav::GrvcUav(const std::string& _uri, const hal::Vec3& startPos) {
			mPos = startPos;
		}

		//--------------------------------------------------------------------------------------------------------------
		void GrvcUav::takeOff(double _height)  {
		}

		//--------------------------------------------------------------------------------------------------------------
		void GrvcUav::land()  {
		}

		//--------------------------------------------------------------------------------------------------------------
		void GrvcUav::goTo(const hal::Waypoint& _wp)  {
		}

		//--------------------------------------------------------------------------------------------------------------
		void GrvcUav::trackPath(const hal::WaypointList&)  {
		}

		//--------------------------------------------------------------------------------------------------------------
		const hal::Vec3& GrvcUav::position() const  {
			return mPos;
		}

		//--------------------------------------------------------------------------------------------------------------
		//Rectangle GrvcUav::viewArea() const  {
		//}
} }	// namespace grvc::mbzirc
